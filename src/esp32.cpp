#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDRESS 0x08

// Пины энкодеров
const int encoderLeft1A = 34, encoderLeft1B = 35;
const int encoderRight1A = 32, encoderRight1B = 33;

// Пины ШИМ-сигналов (PWM) для управления моторами
const int pwmLeft1 = 25, pwmLeft2 = 26, pwmRight1 = 27, pwmRight2 = 14;

// Пины для управления направлением моторов
const int dirLeft1 = 12, dirLeft2 = 13;
const int dirRight1 = 15, dirRight2 = 16;

// Переменные для хранения данных энкодеров
volatile long encoderLeft1 = 0, encoderRight1 = 0;

// Целевые скорости моторов
int16_t targetLeft1 = 0, targetLeft2 = 0;
int16_t targetRight1 = 0, targetRight2 = 0;

// Коэффициенты PID-регулятора
float Kp1 = 1.0, Ki1 = 0.1, Kd1 = 0.05;
float Kp3 = 1.0, Ki3 = 0.1, Kd3 = 0.05;

// Ошибки PID
float error1, previousError1 = 0, integral1 = 0;
float error3, previousError3 = 0, integral3 = 0;

// Время последнего обновления
unsigned long lastTime = 0;

// Обработчики прерываний для энкодеров
void IRAM_ATTR handleEncoderLeft1A() { encoderLeft1 += (digitalRead(encoderLeft1A) == digitalRead(encoderLeft1B)) ? 1 : -1; }
void IRAM_ATTR handleEncoderRight1A() { encoderRight1 += (digitalRead(encoderRight1A) == digitalRead(encoderRight1B)) ? 1 : -1; }

// Получение данных по I2C
void receiveEvent(int howMany) {
    if (howMany >= 8) {
        Wire.read();
        targetLeft1 = Wire.read() << 8 | Wire.read();
        targetLeft2 = Wire.read() << 8 | Wire.read();
        targetRight1 = Wire.read() << 8 | Wire.read();
        targetRight2 = Wire.read() << 8 | Wire.read();
    }
}

// Настройка энкодеров
void setupEncoders() {
    pinMode(encoderLeft1A, INPUT_PULLUP);
    pinMode(encoderLeft1B, INPUT_PULLUP);
    pinMode(encoderRight1A, INPUT_PULLUP);
    pinMode(encoderRight1B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderLeft1A), handleEncoderLeft1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderRight1A), handleEncoderRight1A, CHANGE);
}

// Настройка ШИМ (PWM) для моторов
void setupPWM() {
    ledcSetup(0, 5000, 8);
    ledcAttachPin(pwmLeft1, 0);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(pwmLeft2, 1);
    ledcSetup(2, 5000, 8);
    ledcAttachPin(pwmRight1, 2);
    ledcSetup(3, 5000, 8);
    ledcAttachPin(pwmRight2, 3);
}

// Настройка пинов моторов
void setupMotors() {
    pinMode(dirLeft1, OUTPUT);
    pinMode(dirLeft2, OUTPUT);
    pinMode(dirRight1, OUTPUT);
    pinMode(dirRight2, OUTPUT);
}

// Управление скоростью и направлением мотора
void setMotorSpeed(int channel, int dirPin, int speed) {
    int pwmValue = constrain(abs(speed), 0, 255);
    digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
    ledcWrite(channel, pwmValue);
}

// Расчеты PID-регулятора
float computePID(float target, float speed, float &error, float &previousError, float &integral, float Kp, float Ki, float Kd, float deltaTime) {
    error = target - speed;
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return output;
}

// Основной цикл управления моторами
void loopMotors() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Читаем данные с энкодеров
    noInterrupts();
    long count1 = encoderLeft1, count3 = encoderRight1;
    encoderLeft1 = 0;
    encoderRight1 = 0;
    interrupts();

    // Вычисление скорости
    float speed1 = count1 / deltaTime;
    float speed3 = count3 / deltaTime;

    // Расчет PID для каждого мотора
    float output1 = computePID(targetLeft1, speed1, error1, previousError1, integral1, Kp1, Ki1, Kd1, deltaTime);
    float output3 = computePID(targetRight1, speed3, error3, previousError3, integral3, Kp3, Ki3, Kd3, deltaTime);

    setMotorSpeed(0, dirLeft1, output1);
    setMotorSpeed(2, dirRight1, output3);
    setMotorSpeed(1, dirLeft2, targetLeft2);
    setMotorSpeed(3, dirRight2, targetRight2);
}

// Базовая настройка
void setup() {
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
    setupEncoders();
    setupPWM();
    setupMotors();
    lastTime = millis();
}

// Основной цикл
void loop() {
    loopMotors();
}
