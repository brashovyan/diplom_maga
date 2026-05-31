import socket
import time
import math
import random

# Настройки UDP
UDP_IP = "239.1.1.1"
UDP_PORT = 7856
MULTICAST_TTL = 2
MAX_PACKET_SIZE = 450  # Максимальный размер пакета (как в CoppeliaSim)

# Создаем UDP сокет для мультикаст рассылки
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def send_trajectory(trajectory_id, vectors):
    """Отправка траектории на робота с разбиением на пакеты"""
    
    # Разбиваем на части по границам векторов
    parts = []
    current_part = f"{trajectory_id};"
    
    for v in vectors:
        vector_str = f"{v['V']:.3f};{v['A']:.3f};{v['W']:.1f};{v['T']:.1f};"
        
        if len(current_part + vector_str) <= MAX_PACKET_SIZE:
            current_part += vector_str
        else:
            parts.append(current_part)
            current_part = f"{trajectory_id};" + vector_str
    
    # Добавляем последнюю часть
    if current_part != f"{trajectory_id};":
        parts.append(current_part)
    
    print(f"Траектория ID={trajectory_id}, векторов={len(vectors)}")
    print(f"Разбита на {len(parts)} частей (макс размер {MAX_PACKET_SIZE} байт)")
    
    # Отправляем все части
    for i, part in enumerate(parts, 1):
        sock.sendto(part.encode(), (UDP_IP, UDP_PORT))
        print(f"  Часть {i}/{len(parts)}: {len(part)} байт")
        time.sleep(1)  # Небольшая задержка между пакетами
    
    print("✅ Траектория отправлена")
    return len(parts)

def send_command(command):
    """Отправка команды: 1 - старт, 0 - стоп"""
    packet = f"command;{command};"
    sock.sendto(packet.encode(), (UDP_IP, UDP_PORT))
    print(f"Команда {'СТАРТ' if command == 1 else 'СТОП'} отправлена")

# ============================================================
# Траектория 1: Буква "Г" (вперёд → вправо → назад → влево)
# ============================================================
def generate_g_trajectory():
    """Генерация траектории в форме буквы Г"""
    trajectory = []
    
    # Движение вперёд (V=0.3 м/с, A=0°, T=0.1 сек, 30 шагов = 3 секунды)
    for _ in range(30):
        trajectory.append({"V": 0.3, "A": 0, "W": 0, "T": 0.1})
    
    # Поворот направо (W=1.57 рад/с, 10 шагов = 1 секунда)
    for _ in range(10):
        trajectory.append({"V": 0, "A": 0, "W": 1.57, "T": 0.1})
    
    # Движение вправо (V=0.3 м/с, A=1.57 рад, 30 шагов)
    for _ in range(30):
        trajectory.append({"V": 0.3, "A": 1.57, "W": 0, "T": 0.1})
    
    # Поворот вниз
    for _ in range(10):
        trajectory.append({"V": 0, "A": 0, "W": 1.57, "T": 0.1})
    
    # Движение вниз (V=0.3 м/с, A=3.14 рад, 30 шагов)
    for _ in range(30):
        trajectory.append({"V": 0.3, "A": 3.14, "W": 0, "T": 0.1})
    
    return trajectory

# ============================================================
# Траектория 2: Круг
# ============================================================
def generate_circle_trajectory(radius_m=0.5, speed_ms=0.3, duration_s=10):
    """Генерация траектории движения по кругу"""
    dt = 0.1  # шаг времени
    angular_speed = speed_ms / radius_m  # рад/с
    steps = int(duration_s / dt)
    
    trajectory = []
    for i in range(steps):
        current_angle = angular_speed * i * dt
        trajectory.append({
            "V": speed_ms,
            "A": current_angle + math.pi/2,  # касательная
            "W": angular_speed,
            "T": dt
        })
    
    return trajectory

# ============================================================
# Траектория 3: Квадрат
# ============================================================
def generate_square_trajectory(side_m=0.5, speed_ms=0.3):
    """Генерация траектории квадрата (сторона side_m метров)"""
    dt = 0.1
    time_per_side = side_m / speed_ms  # время прохождения одной стороны
    steps_per_side = int(time_per_side / dt)
    
    trajectory = []
    
    for _ in range(4):  # 4 стороны
        # Движение вперёд
        for _ in range(steps_per_side):
            trajectory.append({"V": speed_ms, "A": 0, "W": 0, "T": dt})
        
        # Поворот на 90° (π/2 рад)
        for _ in range(10):
            trajectory.append({"V": 0, "A": 0, "W": math.pi/2, "T": dt})
    
    return trajectory

# ============================================================
# Генерация траекторий
# ============================================================
trajectory_G = generate_g_trajectory()
trajectory_circle = generate_circle_trajectory(radius_m=0.5, speed_ms=0.3, duration_s=10)
trajectory_square = generate_square_trajectory(side_m=0.5, speed_ms=0.3)

# Простая траектория вперёд-назад
trajectory_simple = []
for _ in range(50):  # 5 секунд вперёд
    trajectory_simple.append({"V": 0.3, "A": 0, "W": 0, "T": 0.1})
for _ in range(50):  # 5 секунд назад
    trajectory_simple.append({"V": 0.3, "A": math.pi, "W": 0, "T": 0.1})

# ============================================================
# Основная программа
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("Управление роботом через UDP мультикаст")
    print("=" * 50)
    
    while True:
        print("\nВыберите действие:")
        print("1 - Отправить траекторию 'Г'")
        print("2 - Отправить траекторию 'Круг'")
        print("3 - Отправить траекторию 'Квадрат'")
        print("4 - Отправить траекторию 'Вперёд-Назад'")
        print("5 - Старт движение")
        print("6 - Стоп движение")
        print("0 - Выход")
        
        choice = input("> ").strip()
        
        if choice == "1":
            send_trajectory(random.randint(1000000000, 9999999999), trajectory_G)
            print("\nИспользуйте 'Старт' для начала движения")
        
        elif choice == "2":
            send_trajectory(random.randint(1000000000, 9999999999), trajectory_circle)
            print("\nИспользуйте 'Старт' для начала движения")
        
        elif choice == "3":
            send_trajectory(random.randint(1000000000, 9999999999), trajectory_square)
            print("\nИспользуйте 'Старт' для начала движения")
        
        elif choice == "4":
            send_trajectory(random.randint(1000000000, 9999999999), trajectory_simple)
            print("\nИспользуйте 'Старт' для начала движения")
        
        elif choice == "5":
            send_command(1)
        
        elif choice == "6":
            send_command(0)
        
        elif choice == "0":
            print("Выход")
            break
        
        else:
            print("Неверный выбор")
    
    sock.close()