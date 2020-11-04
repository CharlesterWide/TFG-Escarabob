import numpy as np
import cv2 as cv
import paho.mqtt.client as mqtt
import socket
import sys
import cv2 as cv
import pickle
import numpy as np
import struct
import zlib
import multiprocessing
from multiprocessing import Queue
import queue
import math
import time

######################## DEFINICIONES PARA MQTT ####################

host = "192.168.1.11"
topic = "Control/orden"
puerto = 1883

distancia = Queue(10)


def on_connect(client, userdata, flags, rc):

    if rc == 0:

        print("Connected to broker")
        client.subscribe("Control/distancia")
    else:

        print("Connection failed")


def on_message(client, userdata, msg):
    m = str(msg.payload.decode("utf-8"))
    if userdata.full():
        pass
    else:
        userdata.put(m)
        #print('%s %s' % (msg.topic, msg.payload))


def on_publish(client, userdata, result):
    #print("Orden Enviada")
    pass


def cliente(cola_distancia, cola_objetivo):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_publish = on_publish
    client.user_data_set(cola_distancia)

    client.connect(host, puerto)
    client.loop_start()

    objetivo = ""
    adquirido = False
    contador = 10
    contadorREV = 10
    rotar = False
    rotarREV = False
    distancia = 0
    while True:

        if cola_objetivo.empty():
            objetivo = "NA"
        else:
            objetivo = cola_objetivo.get()


        if cola_distancia.empty():
            distancia = 0
        else:
            distancia = cola_distancia.get()
            if distancia == '':
                distancia = 0
            else:
                distancia = int(distancia)

        if distancia == 0:
            pass
        elif distancia == 100:
            client.publish(topic, "on")
        else:
            if objetivo == "NA":
                orden = "STOP"
                client.publish(topic, orden)
            else:
                color = objetivo[0]
                x = objetivo[1]
                area = objetivo[3]
                tipo = objetivo[4]

                if color == 'R':
                    if tipo == 'C':
                        #Perseguir al cuadrado rojo
                        if distancia < 30:
                            if not rotarREV:
                                orden = "REVE"
                                client.publish(topic, orden)
                                contadorREV -= 1
                                if contadorREV == 0:
                                    rotarREV = True
                                    contadorREV = 10
                            else:
                                orden = "DERE"
                                client.publish(topic, orden)
                                contadorREV -= 1
                                if contadorREV == 0:
                                    rotarREV = False
                                    contadorREV = 10
                        else:
                            centro = 300
                            margen = 80
                            distanciamin = 9000

                            if x < (centro - margen):
                                # Obejetivo a la derecha
                                orden = "IZQU"
                                client.publish(topic, orden)
                            elif x > (centro + margen):
                                # Objetivo a la izquierda
                                orden = "DERE"
                                client.publish(topic, orden)
                            else:
                                # Objetivo centrado
                                if area < distanciamin:
                                    orden = "AVAN"
                                    client.publish(topic, orden)
                                else:
                                    orden = "STOP"
                                    client.publish(topic, orden)


            

        time.sleep(0.05)


#################### SOCKET RECEPCION VIDEO #############
con = False


def videoRec(cola_framesR, cola_framesG, cola_framesB, cola_framesF):
    HOST = ''
    PORT = 8485

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket de video creado')

    s.bind((HOST, PORT))
    print('Socket de video bind completado')
    s.listen(10)
    print('Socket de video en espera')

    conn, addr = s.accept()

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

        cola_framesR.put(frame)
        cola_framesG.put(frame)
        cola_framesB.put(frame)
        cola_framesF.put(frame)

####################  OPENCV ####################


def nothing(x):
    pass


font = cv.FONT_HERSHEY_COMPLEX_SMALL


def buscador(cola_frames, val_ini, color, cola_obj):
    cv.namedWindow("Barras " + color)

    cv.createTrackbar('L-H', "Barras " + color, val_ini.l_h, 180, nothing)
    cv.createTrackbar('U-H', "Barras " + color, val_ini.u_h, 180, nothing)

    cv.createTrackbar('L-S', "Barras " + color, val_ini.l_s, 255, nothing)
    cv.createTrackbar('U-S', "Barras " + color, val_ini.u_s, 255, nothing)

    cv.createTrackbar('L-V', "Barras " + color, val_ini.l_v, 255, nothing)
    cv.createTrackbar('U-V', "Barras " + color, val_ini.u_v, 255, nothing)

    while True:
        if(cola_frames.empty()):
            nothing(1)
        else:
            frame = cola_frames.get()

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            l_h = cv.getTrackbarPos('L-H',  "Barras " + color)
            l_s = cv.getTrackbarPos('L-S',  "Barras " + color)
            l_v = cv.getTrackbarPos('L-V',  "Barras " + color)
            u_h = cv.getTrackbarPos('U-H',  "Barras " + color)
            u_s = cv.getTrackbarPos('U-S',  "Barras " + color)
            u_v = cv.getTrackbarPos('U-V',  "Barras " + color)

            color_min = np.array([l_h, l_s, l_v])
            color_max = np.array([u_h, u_s, u_v])

            mask = cv.inRange(hsv, color_min, color_max)
            kernel = np.ones((5, 5), np.uint8)

            mask = cv.erode(mask, kernel)

            contours, _ = cv.findContours(
                mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if(len(contours) < 1):
                if cola_obj.full():
                    pass
                else:
                    cola_obj.put([color, "NA"])
            else:
                areamax = 0
                for contour in contours:
                    area = cv.contourArea(contour)

                    approx = cv.approxPolyDP(
                        contour, 0.02*cv.arcLength(contour, True), True)

                    x = approx.ravel()[0]
                    y = approx.ravel()[1]

                    if(area > 400):
                        momentos = cv.moments(contour)
                        cx = int(momentos['m10']/momentos['m00'])
                        cy = int(momentos['m01']/momentos['m00'])
                        if len(approx) == 3:
                            cv.drawContours(
                                frame, [approx], 0, (val_ini.B, val_ini.G, val_ini.R), 2)
                            cv.circle(frame, (cx, cy), 3,
                                      (val_ini.B, val_ini.G, val_ini.R), -1)
                            if cola_obj.full():
                                pass
                            else:
                                cola_obj.put([color, area, "T", cx, cy])

                            cv.putText(frame, "Base", (x, y),
                                       font, 1, (255, 255, 255))
                        elif len(approx) >= 4 and len(approx) <= 8:
                            if area > areamax:
                                areamax = area
                                cv.drawContours(
                                    frame, [approx], 0, (val_ini.B, val_ini.G, val_ini.R), 2)
                                cv.circle(frame, (cx, cy), 3,
                                          (val_ini.B, val_ini.G, val_ini.R), -1)
                                if cola_obj.full():
                                    pass
                                else:
                                    cola_obj.put([color, area, "C", cx, cy])

                                cv.putText(frame, "Objetivo", (x, y),
                                           font, 1, (255, 255, 255))

            cv.imshow("Mask " + color, mask)
            cv.imshow("Siguiendo " + color, frame)

        if cv.waitKey(1) == ord('q'):
            break

        cv.waitKey(1)


class valores_ini:
    l_h = 0
    l_s = 0
    l_v = 0
    u_h = 0
    u_s = 0
    u_v = 0
    R = 0
    G = 0
    B = 0


"""val_R = valores_ini()

val_R.l_h = 0
val_R.l_s = 0
val_R.l_v = 0
val_R.u_h = 0
val_R.u_s = 0
val_R.u_v = 0
val_R.R = 0"""

val_R = valores_ini()

val_R.l_h = 140
val_R.l_s = 55
val_R.l_v = 0
val_R.u_h = 180
val_R.u_s = 229
val_R.u_v = 255
val_R.R = 255


"""val_G = valores_ini()

val_G.l_h = 45
val_G.l_s = 100
val_G.l_v = 40
val_G.u_h = 70
val_G.u_s = 255
val_G.u_v = 255
val_G.G = 255"""

val_G = valores_ini()

val_G.l_h = 0
val_G.l_s = 0
val_G.l_v = 0
val_G.u_h = 0
val_G.u_s = 0
val_G.u_v = 0
val_G.G = 0

"""val_B = valores_ini()

val_B.l_h = 80
val_B.l_s = 80
val_B.l_v = 40
val_B.u_h = 100
val_B.u_s = 255
val_B.u_v = 255
val_B.B = 255"""

val_B = valores_ini()

val_B.l_h = 0
val_B.l_s = 0
val_B.l_v = 0
val_B.u_h = 0
val_B.u_s = 0
val_B.u_v = 0
val_B.B = 0


########### CONTROL PRINCIPAL ################

class objets:
    color = ""
    tipo = ""
    area = 0
    x = 0
    y = 0


def control(cola_frames, cola_objR, cola_objG, cola_objB, cola_objetivo):

    mas_cercano = objets()
    objR = objets()
    objG = objets()
    objB = objets()

    objetivo = ""

    while True:

        recR = []
        recG = []
        recB = []
        try:
            recR = cola_objR.get()
            recG = cola_objG.get()
            recB = cola_objB.get()
        except:
            recR[1] = "NA"
            recG[1] = "NA"
            recB[1] = "NA"

        if recR[1] == "NA":
            objR = objets()
        else:
            objR.area = recR[1]
            objR.color = 'R'
            objR.x = recR[3]
            objR.y = recR[4]
            objR.tipo = recR[2]

        if recG[1] == "NA":
            objG = objets()
        else:
            objG.area = recG[1]
            objG.color = 'G'
            objG.x = recG[3]
            objG.y = recG[4]
            objG.tipo = recG[2]

        if recB[1] == "NA":
            objB = objets()
        else:
            objB.area = recB[1]
            objB.color = 'B'
            objB.x = recB[3]
            objB.y = recB[4]
            objB.tipo = recB[2]

        if objR.area > objG.area and objR.area > objB.area and objR.area > 0:

            objetivo = "R"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put(
                    [objetivo, objR.x, objR.y, objR.area, objR.tipo])

        elif objG.area > objB.area and objG.area > 0:

            objetivo = "G"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put(
                    [objetivo, objG.x, objG.y, objG.area, objG.tipo])
        elif objB.area > 0:

            objetivo = "B"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put(
                    [objetivo, objB.x, objB.y, objB.area, objB.tipo])
        else:
            objetivo = ""


########### MAIN ######################
if __name__ == '__main__':
    framesR = Queue()
    framesG = Queue()
    framesB = Queue()
    framesF = Queue()
    objetosR = Queue(10)
    objetosG = Queue(10)
    objetosB = Queue(10)
    objetivo = Queue(10)

    cliente_thread = multiprocessing.Process(
        target=cliente, args=(distancia, objetivo))

    videoRec_thread = multiprocessing.Process(
        target=videoRec, args=(framesR, framesG, framesB, framesF))

    R_thread = multiprocessing.Process(
        target=buscador, args=(framesR, val_R, "R", objetosR))

    G_thread = multiprocessing.Process(
        target=buscador, args=(framesG, val_G, "G", objetosG))

    B_thread = multiprocessing.Process(
        target=buscador, args=(framesB, val_B, "B", objetosB))

    control_thread = multiprocessing.Process(
        target=control, args=(framesF, objetosR, objetosG, objetosB, objetivo))

    cliente_thread.start()
    videoRec_thread.start()
    R_thread.start()
    G_thread.start()
    B_thread.start()
    control_thread.start()

    cliente_thread.join()
    videoRec_thread.join()
    R_thread.join()
    G_thread.join()
    B_thread.join()
    control_thread.join()

    exit()
