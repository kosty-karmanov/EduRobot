// Пины моторов
#define IN1 6
#define IN2 7
#define ENA 5

#define IN3 9
#define IN4 8
#define ENB 10


// Текущие скорости

int speedLeft = 0;
int speedRight = 0;

// Настройка пинов

void setup() {
    Serial.begin(9600);
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
}

// Читаем данные с Serial порта, получаем скорости от Raspberry pi и задаем их моторам

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        int spaceIndex = command.indexOf(' ');
        if (spaceIndex != -1) {
            speedLeft = command.substring(0, spaceIndex).toInt();
            speedRight = command.substring(spaceIndex + 1).toInt();

            controlMotors(speedLeft, speedRight);
        }
    }
}

// Задаем скорость моторам

void controlMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else if (leftSpeed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }

    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, abs(leftSpeed));
    analogWrite(ENB, abs(rightSpeed));
}
