# Подключаем необходимые библиотеки
import serial
import sys
import termios
import tty

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

min_speed = 50
max_speed = 200
turn_speed = 170
speed_step = 10

speed_left = 0
speed_right = 0
last_action = None


def send_speed() -> None:
    """
    Задает скорость моторам путем отправки данных на Arduino через Serial
    :return:
    """
    command = f"{speed_left} {speed_right}\n"
    arduino.write(command.encode())
    print(f"Отправлено: {command.strip()}")


def get_key() -> str:
    """
    Получает текущую нажатую клавишу на клавиатуре
    :return: Строковое представление нажатой клавиши
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


print("WASD для управления, Q - стоп, E - показать скорость")

while True:
    key = get_key().upper()

    if key == 'S':
        # Движение назад
        if last_action == "turn":
            base_speed = min_speed
        else:
            base_speed = min(max(speed_left + speed_step, min_speed), max_speed)
        speed_left = base_speed
        speed_right = base_speed
        last_action = "move"

    elif key == 'W':
        # Движение вперед
        if last_action == "turn":
            base_speed = -min_speed
        else:
            base_speed = max(min(speed_left - speed_step, -min_speed), -max_speed)
        speed_left = base_speed
        speed_right = base_speed
        last_action = "move"

    elif key == 'A':
        # Поворот влево
        speed_left = turn_speed
        speed_right = -turn_speed
        last_action = "turn"

    elif key == 'D':
        # Поворот вправо
        speed_left = -turn_speed
        speed_right = turn_speed
        last_action = "turn"

    elif key == 'Q':
        # Полная остановка
        speed_left = 0
        speed_right = 0
        last_action = None

    elif key == 'E':
        # Показывает текущие скорости
        print(f"Текущая скорость: Левый {speed_left}, Правый {speed_right}")
        continue

    elif key == '\x03':
        # Выход из программы
        break

    send_speed()
