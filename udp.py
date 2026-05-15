import socket
import struct

MULTICAST_IP = "239.1.1.1"
PORT = 7856

# Создаем сокет
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

# Разрешаем переиспользовать адрес
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Привязываемся к ВСЕМ интерфейсам на указанный порт
sock.bind(('', PORT))  # ВАЖНО: '', а не мультикаст адрес!

# Подписываемся на мультикаст группу
mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print(f"Слушаю мультикаст {MULTICAST_IP}:{PORT}")

while True:
    data, addr = sock.recvfrom(65536)
    print(f"\nПолучено {len(data)} байт от {addr}")
    try:
        print("Начало:", data.decode('utf-8')[:100])
        print("Конец:", data.decode('utf-8')[-100:])
    except:
        print("(не удалось декодировать)")