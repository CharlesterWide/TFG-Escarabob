import socket

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket Creado")

serverSocket.bind(('', 8485))
print("Socket binded")

serverSocket.listen()
print("Socket a la escucha")

orden = "on"
dist = ""

while True:

    conn, addr = serverSocket.accept()

    serverSocket.setblocking(0)

    print("Conexion establecida con %s:%s" % (addr[0], addr[1]))

    conn.send(orden.encode())

    while True:
        try:
            dist = conn.recv(1024)
            print(dist.decode())
        except:
            conn.send(orden.encode())
