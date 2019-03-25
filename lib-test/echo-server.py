'''

Opens TCP/UDP sockets and uses TLS for testing esp32-lib

Configure TCP_PORT, UDP_PORT and TLS_PORT as needed
Configure CERT_PATH and CERT_KEY_PATH to point to certificates
    to be used for TLS

'''

import socket
import ssl
from threading import Thread

#
#   Configuration
#

IP_ADDR = "0.0.0.0"
TCP_PORT = 1000
UDP_PORT = 1001
TLS_PORT = 1002

RECV_SIZE = 2048
BACKLOG = 1

ROOT_CA_PATH = "certs/root_ca.crt"
CERT_PATH = "certs/server_cert.crt"
CERT_KEY_PATH = "certs/server_cert.key"

#
#   Initialization
#
tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Underlying TCP socket for TLS
tlssock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Set sockets to be reusable before timeout expires after disconnect
tcpsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udpsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tlssock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

tcpsock.bind((IP_ADDR, TCP_PORT))
udpsock.bind((IP_ADDR, UDP_PORT))
tlssock.bind((IP_ADDR, TLS_PORT))

tcpsock.listen()
tlssock.listen()

# Configure TLS context
tls_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
tls_context.verify_mode = ssl.CERT_REQUIRED
tls_context.load_cert_chain(certfile=CERT_PATH, keyfile=CERT_KEY_PATH)
tls_context.load_verify_locations(cafile=ROOT_CA_PATH)

class UDPSocketThread(Thread):

    def __init__(self, addr):
        Thread.__init__(self)

        print("[UDP] New thread listening on {}:{}".format(*addr))

    def run(self):
        while True:
            data, addr = udpsock.recvfrom(RECV_SIZE)
            if not data:
                break

            print("[UDP] Received data: {}".format(data))
            print("[UDP] Sending data...")

            # Echo data
            udpsock.sendto(data, addr)

class TCPSocketThread(Thread):

    def __init__(self, conn, addr, use_tls=False):
        Thread.__init__(self)

        self.conn = conn
        self.use_tls = use_tls
        self.protocol = "TLS" if use_tls else "TCP"

        print("[{}] New thread listening on {}:{}".format(self.protocol, *addr))

    def run(self):
        while True:
            data = self.conn.recv(RECV_SIZE)
            if not data:
                break

            print("[{}] Received data: {}".format(self.protocol, data))
            print("[{}] Sending data...".format(self.protocol))

            # Echo data
            self.conn.send(data)

# Waits for connections to establish and assigns the appropriate EchoThread
class SocketConnectThread(Thread):

    def __init__(self, port, protocol):
        Thread.__init__(self)

        self.port = port
        self.protocol = protocol

    def run(self):

        while True:
            if (self.protocol == "TCP"):
                (conn, addr) = tcpsock.accept()
                t = TCPSocketThread(conn, addr)

            elif (self.protocol == "TLS"):
                (conn, addr) = tlssock.accept()
                # Wrap underlying TCP Socket with TLS implementation
                tlsconn = tls_context.wrap_socket(conn, server_side=True)
                t = TCPSocketThread(tlsconn, addr, use_tls=True)

            else:
                return

            t.start()
            t.join()

def main():

    print("Waiting for incoming connections")
    print("TCP Port: {}".format(TCP_PORT))
    print("UDP Port: {}".format(UDP_PORT))
    print("TLS Port: {}".format(TLS_PORT))

    threads = []

    # UDP
    # SocketThread not needed as UDP does not establish connection
    t = UDPSocketThread((IP_ADDR, UDP_PORT))
    t.start()

    threads.append(t)

    # TCP
    t = SocketConnectThread(TCP_PORT, "TCP")
    t.start()

    threads.append(t)

    # TLS
    t = SocketConnectThread(TLS_PORT, "TLS")
    t.start()

    threads.append(t)

if __name__ == "__main__":
    main()
