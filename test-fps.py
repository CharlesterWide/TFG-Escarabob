import multiprocessing
from multiprocessing import Queue
import time
import cv2 as cv
import socket
import pickle
import numpy as np
import struct
import zlib
import math


HOST = ''
PORT = 8485
font = cv.FONT_HERSHEY_SIMPLEX

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket de video creado')

s.bind((HOST, PORT))
print('Socket de video bind completado')
s.listen(10)
print('Socket de video en espera')

conn, addr = s.accept()
cont = 0
fps = 0
minuto = 1
fpsMed = 0
Med = 0
timer = time.time()

print("Conexion de video establecida con %s:%s" % (addr[0], addr[1]))

data = b""
payload_size = struct.calcsize(">L")
print("payload_size: {}".format(payload_size))



while True:

    while len(data) < payload_size:
        # print("Recv: {}".format(len(data)))
        data += conn.recv(4096)


    # print("Done Recv: {}".format(len(data)))
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    # print("msg_size: {}".format(msg_size))
    while len(data) < msg_size:
        data += conn.recv(4096)
    
    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    frame = cv.imdecode(frame, cv.IMREAD_COLOR)

    frame = cv.flip(frame, 0)
    frame = cv.flip(frame, 1)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    cv.putText(frame, str(fps), (150, 150),font, 5, (0, 255, 0),7)
    cv.putText(frame, str(Med), (400, 150),font, 5, (255, 0, 0),7)

    if time.time() > (timer+1):
        fps = cont
        cont = 0
        timer = time.time()
        minuto += 1
        fpsMed = (fpsMed+fps)/2
        if minuto == 60:
            Med = fpsMed
            Med = math.trunc(Med)
            fpsMed = 0
            minuto = 1

    else:
        cont += 1
        
    cv.imshow("FPS",frame)

    if cv.waitKey(1) == ord('q'):
        break

    cv.waitKey(1)







