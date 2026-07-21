#!/usr/bin/env python3
"""
GLAZE — Autonomous Waste Collection Vessel
Core AI perception and navigation controller.

Pipeline: video source -> YOLOv8 inference -> vessel state machine -> Arduino serial link.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, NamedTuple, Optional, Tuple, Union, TYPE_CHECKING

import cv2
import numpy as np
import serial
from ultralytics import YOLO

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

logger = logging.getLogger("glaze")


def _configure_logging(level: int = logging.INFO) -> None:
    if logger.handlers:
        return
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    ))
    logger.addHandler(handler)
    logger.setLevel(level)
    logger.propagate = False


class VesselState(Enum):
    PATROL = auto()
    SEEK = auto()
    COLLECT = auto()
    EMERGENCY_STOP = auto()


class Detection(NamedTuple):
    class_id: int
    confidence: float
    xyxy: Tuple[float, float, float, float]


@dataclass(frozen=True)
class GlazeConfig:
    model_path: str = "weights/yolov8n.pt"
    camera_source: Union[int, str] = 0  # webcam index, or ESP32-CAM MJPEG URL

    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    serial_timeout: float = 1.0

    frame_width: int = 640
    frame_height: int = 480
    enable_display: bool = True
    command_hz: float = 15.0
    max_read_failures: int = 30

    waste_confidence: float = 0.55
    fauna_confidence: float = 0.35  # intentionally permissive: a missed animal costs far more than a false stop
    target_classes: Dict[int, str] = field(default_factory=lambda: {39: "bottle", 41: "cup"})  # COCO indices
    fauna_classes: Dict[int, str] = field(default_factory=lambda: {14: "bird"})  # COCO indices

    base_speed: int = 140
    max_speed: int = 220
    steering_gain: float = 90.0
    proximity_collect_ratio: float = 0.18

    patrol_amplitude: float = 45.0
    patrol_turn_rate: float = 0.05

    fauna_hold_seconds: float = 4.0  # safety latch: keeps the stop engaged briefly after fauna leaves frame

    def __post_init__(self) -> None:
        if not 0.0 <= self.waste_confidence <= 1.0:
            raise ValueError("waste_confidence must fall within [0, 1]")
        if not 0.0 <= self.fauna_confidence <= 1.0:
            raise ValueError("fauna_confidence must fall within [0, 1]")
        if self.base_speed > self.max_speed:
            raise ValueError("base_speed cannot exceed max_speed")


class HardwareLink:
    """Serial bridge to the Arduino motor/conveyor controller.

    Falls back to a headless simulation mode when the configured port is
    unavailable, so perception work is never blocked by absent hardware.
    """

    def __init__(self, config: GlazeConfig) -> None:
        self._config = config
        self._connection: Optional[serial.Serial] = None
        self._simulated = False
        self._connect()

    def _connect(self) -> None:
        try:
            self._connection = serial.Serial(
                port=self._config.serial_port,
                baudrate=self._config.baud_rate,
                timeout=self._config.serial_timeout,
            )
            time.sleep(2.0)  # Arduino resets on DTR toggle; bootloader needs time to clear
            logger.info("Serial link up on %s @ %d baud", self._config.serial_port, self._config.baud_rate)
        except (serial.SerialException, OSError) as exc:
            self._simulated = True
            logger.warning("Serial port unavailable (%s) — falling back to simulation mode", exc)

    @property
    def simulated(self) -> bool:
        return self._simulated

    def send_motion(self, left: float, right: float, conveyor: int) -> None:
        left_i = int(np.clip(left, 0, self._config.max_speed))
        right_i = int(np.clip(right, 0, self._config.max_speed))
        conveyor_i = int(np.clip(conveyor, 0, 255))

        if self._simulated:
            logger.debug("[SIM] M%d,%d,%d", left_i, right_i, conveyor_i)
            return

        try:
            self._connection.write(f"M{left_i},{right_i},{conveyor_i}\n".encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            logger.error("Serial write failed, dropping to simulation mode: %s", exc)
            self._simulated = True

    def emergency_stop(self) -> None:
        self.send_motion(0, 0, 0)

    def close(self) -> None:
        self.emergency_stop()
        if self._connection is not None and self._connection.is_open:
            self._connection.close()
            logger.info("Serial link closed")


class GlazeCore:
    """Perception-to-actuation pipeline: frame -> detections -> vessel command."""

    _TRASH_COLOR: Tuple[int, int, int] = (20, 255, 57)
    _FAUNA_COLOR: Tuple[int, int, int] = (0, 0, 255)

    def __init__(self, config: GlazeConfig, hardware: HardwareLink) -> None:
        self._config = config
        self._hardware = hardware
        self._capture: Optional[cv2.VideoCapture] = None
        self._state = VesselState.PATROL
        self._fauna_last_seen = 0.0
        self._patrol_phase = 0.0
        self._relevant_classes: List[int] = [*config.target_classes, *config.fauna_classes]
        # YOLO applies one global threshold at inference; per-class filtering happens in _classify
        self._min_confidence = min(config.waste_confidence, config.fauna_confidence)
        self._model = self._load_model()

    def _load_model(self) -> YOLO:
        try:
            return YOLO(self._config.model_path)
        except Exception as exc:
            logger.critical("Failed to load model from %s: %s", self._config.model_path, exc)
            raise

    def run(self) -> None:
        try:
            self._capture = self._open_capture()
            logger.info("GLAZE core online — hardware=%s", "SIMULATED" if self._hardware.simulated else "LIVE")

            frame_interval = 1.0 / self._config.command_hz
            read_failures = 0

            while True:
                loop_start = time.perf_counter()
                ok, frame = self._capture.read()

                if not ok:
                    read_failures += 1
                    logger.warning("Frame grab failed (%d/%d)", read_failures, self._config.max_read_failures)
                    if read_failures >= self._config.max_read_failures:
                        logger.error("Video source unresponsive — reconnecting")
                        self._capture.release()
                        self._capture = self._open_capture()
                        read_failures = 0
                    continue
                read_failures = 0

                try:
                    results = self._model(
                        frame,
                        stream=True,
                        verbose=False,
                        conf=self._min_confidence,
                        classes=self._relevant_classes,
                    )
                    for result in results:
                        annotated = self._process(result, frame)
                        if self._config.enable_display:
                            cv2.imshow("GLAZE", annotated)
                except Exception:
                    logger.exception("Unhandled fault in perception loop — engaging failsafe stop")
                    self._hardware.emergency_stop()

                if self._config.enable_display and cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Operator shutdown requested")
                    break

                self._throttle(loop_start, frame_interval)
        finally:
            self._shutdown()

    def _open_capture(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self._config.camera_source)
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open video source: {self._config.camera_source}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.frame_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.frame_height)
        return capture

    @staticmethod
    def _throttle(loop_start: float, interval: float) -> None:
        elapsed = time.perf_counter() - loop_start
        if elapsed < interval:
            time.sleep(interval - elapsed)

    def _process(self, result: Results, frame: np.ndarray) -> np.ndarray:
        fauna, waste = self._classify(result)

        now = time.time()
        if fauna:
            self._fauna_last_seen = now

        if now - self._fauna_last_seen < self._config.fauna_hold_seconds:
            self._state = VesselState.EMERGENCY_STOP
            self._hardware.emergency_stop()
            if fauna:
                logger.warning("Fauna in frame — safety stop engaged (%d subject(s))", len(fauna))
        elif waste:
            target = max(waste, key=lambda d: self._bbox_area(d.xyxy))
            self._state = self._pursue(target, frame.shape)
        else:
            self._state = VesselState.PATROL
            self._patrol()

        return self._draw_overlay(frame, fauna, waste)

    def _classify(self, result: Results) -> Tuple[List[Detection], List[Detection]]:
        fauna: List[Detection] = []
        waste: List[Detection] = []
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return fauna, waste

        for box in boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            xyxy: Tuple[float, float, float, float] = (x1, y1, x2, y2)

            if class_id in self._config.fauna_classes and confidence >= self._config.fauna_confidence:
                fauna.append(Detection(class_id, confidence, xyxy))
            elif class_id in self._config.target_classes and confidence >= self._config.waste_confidence:
                waste.append(Detection(class_id, confidence, xyxy))

        return fauna, waste

    @staticmethod
    def _bbox_area(xyxy: Tuple[float, float, float, float]) -> float:
        x1, y1, x2, y2 = xyxy
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def _pursue(self, target: Detection, frame_shape: Tuple[int, ...]) -> VesselState:
        height, width = frame_shape[:2]
        x1, _, x2, _ = target.xyxy
        center_x = (x1 + x2) / 2.0
        area_ratio = self._bbox_area(target.xyxy) / (width * height)

        error = (center_x - width / 2.0) / (width / 2.0)  # -1..1, positive => target right of center
        correction = error * self._config.steering_gain

        if area_ratio >= self._config.proximity_collect_ratio:
            left = right = self._config.base_speed * 0.5  # slow, controlled final approach
            conveyor = 255
            state = VesselState.COLLECT
        else:
            left = self._config.base_speed + correction
            right = self._config.base_speed - correction
            conveyor = 0
            state = VesselState.SEEK

        self._hardware.send_motion(left, right, conveyor)
        return state

    def _patrol(self) -> None:
        self._patrol_phase += self._config.patrol_turn_rate
        bias = math.sin(self._patrol_phase) * self._config.patrol_amplitude
        self._hardware.send_motion(
            self._config.base_speed - bias,
            self._config.base_speed + bias,
            0,
        )

    def _draw_overlay(self, frame: np.ndarray, fauna: List[Detection], waste: List[Detection]) -> np.ndarray:
        for detection in waste:
            label = self._config.target_classes.get(detection.class_id, "waste")
            self._draw_box(frame, detection, self._TRASH_COLOR, label)
        for detection in fauna:
            label = self._config.fauna_classes.get(detection.class_id, "fauna")
            self._draw_box(frame, detection, self._FAUNA_COLOR, label)

        state_color = self._FAUNA_COLOR if self._state == VesselState.EMERGENCY_STOP else (255, 255, 255)
        cv2.putText(frame, f"STATE: {self._state.name}", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2, cv2.LINE_AA)
        return frame

    @staticmethod
    def _draw_box(canvas: np.ndarray, detection: Detection, color: Tuple[int, int, int], label: str) -> None:
        x1, y1, x2, y2 = (int(v) for v in detection.xyxy)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)

        tag = f"{label} {detection.confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(canvas, (x1, y1 - th - 8), (x1 + tw + 6, y1), color, -1)
        cv2.putText(canvas, tag, (x1 + 3, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def _shutdown(self) -> None:
        if self._capture is not None:
            self._capture.release()
        cv2.destroyAllWindows()
        self._hardware.close()
        logger.info("GLAZE core offline")


def main() -> None:
    _configure_logging()
    config = GlazeConfig()
    hardware = HardwareLink(config)
    core = GlazeCore(config, hardware)

    try:
        core.run()
    except KeyboardInterrupt:
        logger.info("Interrupted by operator (Ctrl+C)")


if __name__ == "__main__":
    main()