import multiprocessing
from multiprocessing import Queue
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


def cam():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect(('192.168.1.10', 8485))
    connection = client_socket.makefile('wb')

    cam = cv2.VideoCapture(0)

    # cam.set(3, 320);
    # cam.set(4, 240);

    img_counter = 0

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    while True:
        ret, frame = cam.read()
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = zlib.compress(pickle.dumps(frame, 0))
        data = pickle.dumps(frame, 0)
        size = len(data)

        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1

    cam.release()


def cont():
    arduino = serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(2)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('192.168.1.10', 8486))
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
            pass

        if data == "on":
            arduino.write(str.encode('ON'))
            on = True

        if on:
            dist = arduino.readline()

            if dist.decode() != '':
                print(dist.decode())
                client_socket.send(dist)


if __name__ == '__main__':
    camara = multiprocessing.Process(target=cam, args=())
    control = multiprocessing.Process(target=cont, args=())

    camara.start()
    control.start()

    camara.join()
    control.join()

    exit()