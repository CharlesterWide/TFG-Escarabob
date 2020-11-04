import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import serial
import time
import sys


arduino = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.10', 8485))
connection = client_socket.makefile('wb')
client_socket.setblocking(0)

print("Conexion establecida")

data = ""
dist = ""
on = False


while True:
    try:
        data = client_socket.recv(1024)
        data = data.decode()
        print(data)
    except:
        data = ""

    if data == "on":
        arduino.write(str.encode('on\n'))
        on = True
    elif data == "AVANCE":
        arduino.write(str.encode('a60\n'))
    elif data == "REVERSA":
        arduino.write(str.encode('r60\n'))
    elif data == "DERECHA":
        arduino.write(str.encode('d80\n'))
    elif data == "IZQUIERDA":
        arduino.write(str.encode('i80\n'))
    elif data == "STOP":
        arduino.write(str.encode('s0\n'))
    elif data == "PINZA":
        arduino.write(str.encode('p175\n'))
    elif data == "CUE":
        arduino.write(str.encode('c100\n'))
    else:
        pass



    if on:
        dist = arduino.readline()

        if dist.decode() != '':
            print(dist.decode())
            client_socket.send(dist)


