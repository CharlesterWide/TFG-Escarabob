import socket
import time

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket Creado")

serverSocket.bind(('', 8485))
print("Socket binded")

serverSocket.listen()
print("Socket a la escucha")

orden = "on"
dist = ""
fin = False


conn, addr = serverSocket.accept()

serverSocket.setblocking(0)

print("Conexion establecida con %s:%s" % (addr[0], addr[1]))

conn.send(orden.encode())

time.sleep(1)
orden = "CUE"
conn.send(orden.encode())
time.sleep(1)

time.sleep(1)
orden = "AVANCE"
conn.send(orden.encode())
time.sleep(1)

orden = "DERECHA"
conn.send(orden.encode())
time.sleep(0.4)

orden = "AVANCE"
conn.send(orden.encode())
time.sleep(1)

orden = "DERECHA"
conn.send(orden.encode())
time.sleep(0.4)

orden = "AVANCE"
conn.send(orden.encode())
time.sleep(1)

orden = "DERECHA"
conn.send(orden.encode())
time.sleep(0.4)

orden = "AVANCE"
conn.send(orden.encode())
time.sleep(1)

orden = "STOP"
conn.send(orden.encode())
time.sleep(1)

serverSocket.close()
