// =========================================================
// 🌊 GLAZE - Autonomous Trash Skimmer Catamaran
// Прошивка для Arduino Uno R3 (Контроллер движителей и датчиков)
// Версия: 1.0.0
// =========================================================

#include <Servo.h>

// =========================================================
// 📌 ТАБЛИЦА НАЗНАЧЕНИЯ ПИНОВ (PINOUT)
// =========================================================
// [Arduino Pin] -> [Компонент/Функция] -> [Контакт компонента]
// ---------------------------------------------------------
// Питание:
//   Vin (DC Jack) -> 9V Крона (основное питание Arduino)
//
// Драйвер моторов L298N (Ходовые двигатели):
//   D5  -> Левый мотор PWM (ШИМ-скорость)   -> ENA (на L298N)
//   D6  -> Правый мотор PWM (ШИМ-скорость)  -> ENB (на L298N)
//   D7  -> Левый мотор Направление 1        -> IN1
//   D8  -> Левый мотор Направление 2        -> IN2
//   D9  -> Правый мотор Направление 1       -> IN3
//   D10 -> Правый мотор Направление 2       -> IN4
//
// Сервопривод руля SG90:
//   D3  -> Сигнальный провод (оранж/желт)   -> Signal (PWM)
//
// Модуль сборщика (Конвейер/Сетка):
//   D11 -> PWM/Цифровой выход на драйвер    -> Управление сборщиком
//
// Ультразвуковой дальномер JSN-SR04T (Водонепроницаемый):
//   D4  -> Trig (Отправка импульса)         -> Trig
//   D2  -> Echo (Прием отраженного сигнала) -> Echo
//
// Питание периферии:
//   Пин 5V  (Arduino) -> VCC дальномера (если тянет, иначе внешнее)
//   Пин GND (Arduino) -> GND дальномера, GND L298N (общая земля!)
//   Внешняя батарея 4xAA -> +12V (Vin) на L298N
// =========================================================

// --- Константы сервопривода ---
const int SERVO_PIN = 3;
const int SERVO_CENTER = 90;   // Центральное положение руля (прямо)
const int SERVO_MIN = 45;      // Максимально влево
const int SERVO_MAX = 135;     // Максимально вправо

// --- Константы моторов (L298N) ---
const int ENA = 5;   // PWM левый
const int ENB = 6;   // PWM правый
const int IN1 = 7;   // Левый вперед
const int IN2 = 8;   // Левый назад
const int IN3 = 9;   // Правый вперед
const int IN4 = 10;  // Правый назад
const int CONVEYOR_PIN = 11; // PWM сборщика

// --- Константы ультразвукового датчика ---
const int TRIG_PIN = 4;
const int ECHO_PIN = 2;
const unsigned long SONAR_INTERVAL = 100; // Опрос каждые 100 мс
const float SAFE_DISTANCE_CM = 30.0;       // Порог опасного сближения
const float SPEED_OF_SOUND = 0.0343;       // см/мкс (при 20°C)

// --- Таймаут связи (Failsafe) ---
const unsigned long COMMAND_TIMEOUT = 1500; // 1.5 секунды без команд -> стоп

// --- Глобальные переменные ---
Servo rudderServo;                // Объект сервопривода

unsigned long lastCommandTime = 0; // Время последней полученной команды (millis)
unsigned long lastSonarTime = 0;   // Время последнего замера дальномера
bool emergencyStop = false;       // Флаг аварийной остановки (по дальномеру или таймауту)

float currentDistance = 999.0;    // Текущая дистанция до препятствия (см)

// Переменные для хранения целевых скоростей из Serial
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int targetConveyorSpeed = 0;

// Буфер для сборки входящей строки из Serial
String inputBuffer = "";
boolean commandReady = false;

// =========================================================
// 🔧 ИНИЦИАЛИЗАЦИЯ
// =========================================================
void setup() {
  // Инициализация Serial-соединения с компьютером (Python)
  Serial.begin(115200);
  // Резервируем память под строку, чтобы избежать фрагментации
  inputBuffer.reserve(32);

  // Настройка пинов драйвера L298N
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(CONVEYOR_PIN, OUTPUT);

  // Настройка пинов ультразвука
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Инициализация сервопривода руля
  rudderServo.attach(SERVO_PIN);
  rudderServo.write(SERVO_CENTER); // Руль прямо

  // Принудительная остановка всех исполнительных механизмов
  stopAllMotors();
  analogWrite(CONVEYOR_PIN, 0);

  // Инициализация таймера команд (чтобы сразу не сработал failsafe)
  lastCommandTime = millis();

  Serial.println("GLAZE Firmware v1.0 READY");
}

// =========================================================
// 🔄 ОСНОВНОЙ ЦИКЛ (НЕБЛОКИРУЮЩИЙ)
// =========================================================
void loop() {
  // 1. Чтение входящих Serial-команд (по одному байту за итерацию)
  readSerialCommand();

  // 2. Если полная команда получена — обработать её
  if (commandReady) {
    processCommand(inputBuffer);
    inputBuffer = "";       // Очищаем буфер
    commandReady = false;
  }

  // 3. Проверка таймаута связи и аварийная остановка при необходимости
  checkFailsafe();

  // 4. Неблокирующее обновление показаний дальномера JSN-SR04T
  updateSonar();

  // 5. Применение рассчитанных скоростей к драйверам
  applyMotorControl();

  // 6. Обновление положения сервопривода руля на основе разницы скоростей
  updateRudder();
}

// =========================================================
// 📨 ЧТЕНИЕ SERIAL (Без блокировки)
// =========================================================
void readSerialCommand() {
  // Читаем по одному символу, пока они есть в буфере UART
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Если поймали символ конца строки \n — команда завершена
    if (inChar == '\n') {
      commandReady = true;
      break; // Выходим из while, чтобы обработать команду в loop()
    }
    // Игнорируем возврат каретки
    else if (inChar == '\r') {
      continue;
    }
    // Добавляем символ в буфер
    else {
      inputBuffer += inChar;
    }
  }
}

// =========================================================
// ⚙️ ОБРАБОТКА КОМАНДЫ "M<left>,<right>,<conveyor>\n"
// =========================================================
void processCommand(String cmd) {
  // Проверяем заголовок команды 'M'
  if (!cmd.startsWith("M")) {
    return; // Неизвестный формат
  }

  // Убираем букву 'M', оставляем только числа
  String data = cmd.substring(1);
  
  // Парсим три целых числа, разделенных запятыми
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  // Проверка валидности формата (должны быть две запятые)
  if (firstComma == -1 || secondComma == -1) {
    return;
  }

  // Извлекаем строки и преобразуем в целые числа
  String leftStr = data.substring(0, firstComma);
  String rightStr = data.substring(firstComma + 1, secondComma);
  String convStr = data.substring(secondComma + 1);

  int leftVal = leftStr.toInt();
  int rightVal = rightStr.toInt();
  int convVal = convStr.toInt();

  // Ограничиваем диапазон 0-255
  targetLeftSpeed = constrain(leftVal, 0, 255);
  targetRightSpeed = constrain(rightVal, 0, 255);
  targetConveyorSpeed = constrain(convVal, 0, 255);

  // Сбрасываем флаг аварийной остановки и обновляем время последней команды
  // (Если была блокировка по датчику, флаг снимется только при отправке новой команды)
  if (emergencyStop && targetLeftSpeed == 0 && targetRightSpeed == 0) {
    // Если Python отправил команду остановки, снимаем аварийный флаг
    emergencyStop = false;
  } else if (targetLeftSpeed > 0 || targetRightSpeed > 0) {
    // Если мы едем, аварийный флаг снимается только если дистанция позволяет
    // (Обновлено в updateSonar)
  }

  lastCommandTime = millis();
}

// =========================================================
// 🛑 ПРОВЕРКА ТАЙМАУТА (Failsafe)
// =========================================================
void checkFailsafe() {
  // Если с последней команды прошло больше COMMAND_TIMEOUT
  if (millis() - lastCommandTime >= COMMAND_TIMEOUT) {
    // Включаем экстренную остановку, если она ещё не активна
    if (!emergencyStop) {
      emergencyStop = true;
      // Принудительно обнуляем целевые скорости
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      targetConveyorSpeed = 0;
      Serial.println("FATAL: Connection timeout! Emergency STOP.");
    }
    // Даже если уже был emergencyStop, постоянно применяем нули
    // чтобы гарантировать остановку.
  }
}

// =========================================================
// 📡 НЕБЛОКИРУЮЩЕЕ ОБНОВЛЕНИЕ ДАННЫХ С JSN-SR04T
// =========================================================
void updateSonar() {
  // Проверяем, не пора ли сделать новый замер (интервал 100 мс)
  if (millis() - lastSonarTime < SONAR_INTERVAL) {
    return; // Время еще не пришло
  }
  lastSonarTime = millis();

  // --- Измерение дистанции ---
  // 1. Генерируем короткий импульс на Trig (10 мкс)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 2. Замеряем длительность ответного импульса на Echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Таймаут 30 мс (примерно 5 метров)

  // 3. Если ответ получен — рассчитываем расстояние
  if (duration > 0) {
    currentDistance = (duration * SPEED_OF_SOUND) / 2.0;
  } else {
    currentDistance = 999.0; // Нет эха (слишком далеко или ошибка)
  }

  // 4. Проверка порога безопасности (если ближе 30 см — стоп)
  if (currentDistance < SAFE_DISTANCE_CM && currentDistance > 0) {
    if (!emergencyStop) {
      emergencyStop = true;
      Serial.print("WARNING: Obstacle detected at ");
      Serial.print(currentDistance);
      Serial.println(" cm! Emergency STOP.");
    }
    // Принудительно зануляем скорости для безопасности
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    targetConveyorSpeed = 0;
  } 
  // Если расстояние снова стало безопасным, снимаем блокировку,
  // но только если с Python идут команды на движение.
  else if (emergencyStop && currentDistance > SAFE_DISTANCE_CM) {
    // Не снимаем автоматически! Ждем новой команды от Python (см. processCommand).
    // Это предотвращает неожиданное снятие с блокировки без ведома оператора.
  }
}

// =========================================================
// 🚀 ПРИМЕНЕНИЕ СКОРОСТЕЙ К ДРАЙВЕРАМ МОТОРОВ
// =========================================================
void applyMotorControl() {
  int leftPWM = 0;
  int rightPWM = 0;
  int convPWM = 0;

  // Если активна аварийная остановка — держим нули, иначе берем цели
  if (!emergencyStop) {
    leftPWM = targetLeftSpeed;
    rightPWM = targetRightSpeed;
    convPWM = targetConveyorSpeed;
  }

  // --- Управление Левым мотором ---
  // Всегда движемся только ВПЕРЕД (как указано в ТЗ)
  // IN1 = HIGH, IN2 = LOW -> вращение вперед
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else {
    // Стоп: тормозим мотор, подключая оба вывода к земле или установив ШИМ 0
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // --- Управление Правым мотором ---
  // IN3 = HIGH, IN4 = LOW -> вращение вперед
  if (rightPWM > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }

  // --- Управление конвейером-сборщиком ---
  analogWrite(CONVEYOR_PIN, convPWM);
}

// =========================================================
// 🛞 РАСЧЕТ И УСТАНОВКА УГЛА ПЕРА РУЛЯ (Сервопривод SG90)
// =========================================================
void updateRudder() {
  int angle = SERVO_CENTER;

  // Если моторы остановлены или авария — руль прямо (центр)
  if (emergencyStop) {
    rudderServo.write(SERVO_CENTER);
    return;
  }

  // Картируем разницу скоростей (right - left) на угол поворота.
  // Диапазон разницы: от -255 (полный левый) до +255 (полный правый).
  int diff = targetRightSpeed - targetLeftSpeed;
  
  if (diff > 0) {
    // Правый мотор быстрее -> нос влево -> руль вправо (угол > 90)
    // diff от 0 до 255 -> угол от 90 до SERVO_MAX
    angle = map(diff, 0, 255, SERVO_CENTER, SERVO_MAX);
  } else if (diff < 0) {
    // Левый мотор быстрее -> нос вправо -> руль влево (угол < 90)
    // diff от -255 до 0 -> угол от SERVO_MIN до 90
    angle = map(diff, -255, 0, SERVO_MIN, SERVO_CENTER);
  } else {
    // Скорости равны -> прямо
    angle = SERVO_CENTER;
  }

  // Ограничиваем угол и отправляем на сервопривод
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  rudderServo.write(angle);
}

// =========================================================
// 🛑 ПОЛНАЯ ОСТАНОВКА ВСЕХ МОТОРОВ (Вспомогательная функция)
// =========================================================
void stopAllMotors() {
  // Левый мотор
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  // Правый мотор
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  
  // Сборщик
  analogWrite(CONVEYOR_PIN, 0);
}
