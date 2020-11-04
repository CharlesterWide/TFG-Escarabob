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
import multiprocessing
from multiprocessing import Queue
import paho.mqtt.client as mqtt


Conected = False
ArduinoOn = False

def cam():
    global Conected
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect(('192.168.1.10', 8485))
    connection = client_socket.makefile('wb')

    cam = cv2.VideoCapture(0)


    img_counter = 0

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    Conected = True

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



host = "192.168.1.11"
puerto = 1883

def on_connect(client, userdata, flags, rc):

    if rc == 0:
        
        print("Connected to broker")
        client.subscribe("Control/orden")
        
    else:

        print("Connection failed")


def on_message(client, arduino, msg):
    print('%s %s' % (msg.topic, msg.payload.decode()))
    data = msg.payload.decode()
    
    

    if data == "on":
        arduino.write(str.encode('on\n'))
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


def on_publish(client, userdata, result):
    #print("Enviado")
    pass


def cont():

    global Conected

    arduino = serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(2)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_publish = on_publish
    client.user_data_set(arduino)



    client.connect(host, puerto)


    client.loop_start()

    while True:
        if Conected:
            client.publish("Control/distancia", '100')
            while True:
                dist = arduino.readline()
                if(dist.decode() != ''):
                    print(dist.decode())
                    client.publish("Control/distancia", dist.decode())



if __name__ == '__main__':
    camara = multiprocessing.Process(target=cam, args=())
    control = multiprocessing.Process(target=cont, args=())

    camara.start()
    control.start()

    camara.join()
    control.join()

    exit()