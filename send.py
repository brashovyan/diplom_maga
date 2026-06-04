import socket
import time
import math

# Настройки UDP
UDP_IP = "239.1.1.1"
UDP_PORT = 7856
MULTICAST_TTL = 2
MAX_PACKET_SIZE = 400

# Создаем UDP сокет для мультикаст рассылки
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

def send_trajectory(trajectory_id, vectors):
    """Отправка траектории в формате CoppeliaSim: ID;V;A;W;T;V;A;W;T;..."""
    
    parts = []
    current_part = f"{trajectory_id};"
    
    for v in vectors:
        vector_str = f"{v['V']:.3f};{v['A']:.3f};{v['W']:.1f};{v['T']:.1f};"
        
        if len(current_part + vector_str) <= MAX_PACKET_SIZE:
            current_part += vector_str
        else:
            parts.append(current_part)
            current_part = f"{trajectory_id};" + vector_str
    
    if current_part != f"{trajectory_id};":
        parts.append(current_part)
    
    print(f"Траектория ID={trajectory_id}, векторов={len(vectors)}")
    print(f"Разбита на {len(parts)} частей")
    
    for i, part in enumerate(parts, 1):
        print(part)
        sock.sendto(part.encode(), (UDP_IP, UDP_PORT))
        print(f"  Часть {i}/{len(parts)}: {len(part)} байт")
        time.sleep(1.5)
    
    print("✅ Траектория отправлена")

def send_command(command):
    """Отправка команды: 1 - старт, 0 - стоп"""
    packet = f"command;{command};"
    sock.sendto(packet.encode(), (UDP_IP, UDP_PORT))
    print(f"Команда {'СТАРТ' if command == 1 else 'СТОП'} отправлена")

# ============================================================
# Простые траектории (каждая из одного вектора)
# ============================================================

def trajectory_forward(duration=2.0):
    """Вперёд: скорость 0.5 м/с, угол 0°, длительность duration"""
    return [{"V": 0.5, "A": 0, "W": 0, "T": duration}]

def trajectory_backward(duration=2.0):
    """Назад: скорость 0.5 м/с, угол 180°"""
    return [{"V": 0.5, "A": math.pi, "W": 0, "T": duration}]

def trajectory_left(duration=2.0):
    """Влево (крабом): скорость 0.5 м/с, угол 90°"""
    return [{"V": 0.5, "A": math.pi/2, "W": 0, "T": duration}]

def trajectory_right(duration=2.0):
    """Вправо (крабом): скорость 0.5 м/с, угол -90°"""
    return [{"V": 0.5, "A": -math.pi/2, "W": 0, "T": duration}]

def trajectory_rotate_left(angle_deg=90):
    """Поворот налево: угловая скорость 1.57 рад/с, время = угол/омега"""
    omega = 1.57  # рад/с
    duration = (angle_deg * math.pi / 180) / omega
    return [{"V": 0, "A": 0, "W": omega, "T": duration}]

def trajectory_rotate_right(angle_deg=90):
    """Поворот направо"""
    omega = -1.57
    duration = (angle_deg * math.pi / 180) / abs(omega)
    return [{"V": 0, "A": 0, "W": omega, "T": duration}]

# ============================================================
# Основная программа
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("Управление роботом через UDP мультикаст")
    print("=" * 50)
    
    # Фиксированный ID для простых команд (или генерируем случайный)
    import random
    last_id = random.randint(100000000, 900000000)
    
    while True:
        print("\nВыберите действие:")
        print("1 - Вперёд (2 сек)")
        print("2 - Назад (2 сек)")
        print("3 - Вправо (2 сек)")
        print("4 - Влево (2 сек)")
        print("5 - Поворот вправо 90°")
        print("6 - Поворот влево 90°")
        print("7 - Старт движение")
        print("8 - Стоп")
        print("0 - Выход")
        
        choice = input("> ").strip()
        
        if choice == "1":
            last_id += 1
            send_trajectory(last_id, trajectory_forward(2.0))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "2":
            last_id += 1
            send_trajectory(last_id, trajectory_backward(2.0))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "3":
            last_id += 1
            send_trajectory(last_id, trajectory_left(2.0))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "4":
            last_id += 1
            send_trajectory(last_id, trajectory_right(2.0))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "5":
            last_id += 1
            send_trajectory(last_id, trajectory_rotate_left(90))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "6":
            last_id += 1
            send_trajectory(last_id, trajectory_rotate_right(90))
            print("Траектория отправлена. Используйте 'Старт' для начала движения")
        
        elif choice == "7":
            send_command(1)
        
        elif choice == "8":
            send_command(0)
        
        elif choice == "0":
            print("Выход")
            break
        
        else:
            print("Неверный выбор")
    
    sock.close()