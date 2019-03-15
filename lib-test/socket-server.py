import socket
from threading import Thread

class TCPSocketThread(Thread):

    def __init__(self, ip, port):
        Thread.__init__(self)

        self.ip = ip
        self.port = port
        print("[TCP] New thread listening on {}:{}".format(ip, port))

    def run(self):
        while True:
            data = conn.recv(2048)
            if not data:
                break

            print("[TCP] Received data: {}".format(data))
            print("[TCP] Sending data...")
            conn.send(data)  # echo data

class UDPSocketThread(Thread):

    def __init__(self, ip, port):
        Thread.__init__(self)

        self.ip = ip
        self.port = port
        print("[UDP] New thread listening on {}:{}".format(ip, port))

    def run(self):
        while True:
            data, addr = udp_sock.recvfrom(2048)
            if not data:
                break

            print("[UDP] Received data: {}".format(data))
            print("[UDP] Sending data...")
            udp_sock.sendto(data, addr)  # echo data

IP_ADDR = "0.0.0.0"
TCP_PORT = 1000
UDP_PORT = 1001
BACKLOG = 1

# open TCPSocket on IPv4
tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_sock.bind((IP_ADDR, TCP_PORT))
tcp_sock.listen(BACKLOG)

# open UDPSocket on IPv4
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_sock.bind((IP_ADDR, UDP_PORT))

threads = []

# listen for incoming connections
while True:

    print("Waiting for incoming connections - TCP: {}; UDP: {}".format(TCP_PORT, UDP_PORT))

    t = UDPSocketThread(IP_ADDR, UDP_PORT)
    t.start()

    threads.append(t)

    (conn, (ip, port)) = tcp_sock.accept()

    t = TCPSocketThread(ip, port)
    t.start()

    threads.append(t)

for t in threads:
    t.join()
