import numpy as np
import cv2 as cv
import socket
import pickle
import struct
import multiprocessing
from multiprocessing import Queue
import time

######################## DEFINICIONES PARA MQTT ####################


def cliente(cola_objetivo, cola_base):
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket de control creado")

    serverSocket.bind(('', 8486))
    print("Socket de control bind completado")

    serverSocket.listen()
    print("Socket de control a la escucha")

    orden = "on"

    objetivo = ""
    base = ""
    adquirido = False
    contador = 10
    contadorREV = 10
    rotar = False
    rotarREV = False
    distancia = 0

    while True:
        conn, addr = serverSocket.accept()

        serverSocket.setblocking(0)

        print("Conexion de control establecida con %s:%s" % (addr[0], addr[1]))

        conn.send(orden.encode())
        while True:

            try:
                distancia = conn.recv(4)
                distancia = distancia.decode()
                if distancia == "HELLO":
                    print(distancia)
                    distancia = 0
                elif distancia == '':
                    distancia = 0
                else:
                    distancia = distancia.replace('\n','')
                    distancia = distancia.replace('\r','')
                    distancia = int(distancia)

                print(distancia)
            except:
                distancia = 0

            if cola_objetivo.empty():
                objetivo = "NA"
            else:
                objetivo = cola_objetivo.get()
                print(objetivo[3])

            if cola_base.empty():
                base = "NA"
            else:
                base = cola_base.get()


            if distancia <= 0:
                pass
            elif distancia >= 100:
                #Codigo en caso de codigo de mensajes de la rasp
                pass
            else:
                if not adquirido:
                    #### Busqueda de pieza
                    if objetivo == "NA":
                        # NO ENCUENTRA PIEZA
                        #Objeto delante
                        if distancia < 20:
                            if not rotarREV:
                                orden = "REVE"
                                conn.send(orden.encode())
                                contadorREV -= 1
                                if contadorREV == 0:
                                    rotarREV = True
                                    contadorREV = 10
                            else:
                                orden = "DERE"
                                conn.send(orden.encode())
                                contadorREV -= 1
                                if contadorREV == 0:
                                    rotarREV = False
                                    contadorREV = 10

                        else:
                            if not rotar:
                                orden = "AVAN"
                                conn.send(orden.encode())
                                contador = contador-1
                                if contador == 0:
                                    contador = 10
                                    rotar = True
                            else:
                                orden = "DERE"
                                conn.send(orden.encode())
                                contador = contador-1
                                if contador == 0:
                                    contador = 10
                                    rotar = False
                    else:
                        # Hay objetivo
                        x = objetivo[1]
                        centro = 300
                        margen = 80
                        area = objetivo[3]
                        distanciamin = 9500
                        if x < (centro - margen):
                            # Obejetivo a la izquierda
                            orden = "IZQU"
                            conn.send(orden.encode())
                        elif x > (centro + margen):
                            # Objetivo a la derecha
                            orden = "DERE"
                            conn.send(orden.encode())
                        else:
                            # Objetivo centrado
                            if area < distanciamin:
                                orden = "AVAN"
                                conn.send(orden.encode())
                            else:
                                orden = "STOP"
                                conn.send(orden.encode())
                                #adquirido = True
                                #time.sleep(2)
                else:
                    # Cubo en las pinzas moviendose a la base
                    if base == "NA":
                        # No encuentra la base
                        if distancia < 20:
                            orden = "REVE"
                            conn.send(orden.encode())
                            time.sleep(0.5)
                            orden = "DERE"
                            conn.send(orden.encode())
                            time.sleep(0.3)
                            orden = "STOP"
                            conn.send(orden.encode())
                        else:
                            if not rotar:
                                orden = "AVAN"
                                conn.send(orden.encode())
                                contador = contador-1
                                if contador == 0:
                                    contador = 10
                                    rotar = True
                            else:
                                orden = "DERE"
                                conn.send(orden.encode())
                                contador = contador-1
                                if contador == 0:
                                    contador = 10
                                    rotar = False
                    else:
                        # Hay objetivo
                        x = base[1]
                        centro = 300
                        margen = 80
                        area = base[3]
                        distanciamin = 9500
                        if x < (centro - margen):
                            # Obejetivo a la izquierda
                            orden = "IZQU"
                            conn.send(orden.encode())
                        elif x > (centro + margen):
                            # Objetivo a la derecha
                            orden = "DERE"
                            conn.send(orden.encode())
                        else:
                            # Objetivo centrado
                            if area < distanciamin:
                                orden = "STOP"
                                conn.send(orden.encode())
                            else:
                                orden = "STOP"
                                conn.send(orden.encode())
                                #adquirido = False
                                #time.sleep(2)

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



val_R = valores_ini()

val_R.l_h = 0
val_R.l_s = 0
val_R.l_v = 0
val_R.u_h = 0
val_R.u_s = 0
val_R.u_v = 0
val_R.R = 0

"""val_R = valores_ini()

val_R.l_h = 0
val_R.l_s = 60
val_R.l_v = 165
val_R.u_h = 38
val_R.u_s = 215
val_R.u_v = 255
val_R.R = 255"""


"""val_G = valores_ini()

val_G.l_h = 45
val_G.l_s = 80
val_G.l_v = 40
val_G.u_h = 65
val_G.u_s = 255
val_G.u_v = 255
val_G.G = 255
"""
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


def control(cola_frames, cola_objR, cola_objG, cola_objB, cola_objetivo, cola_base):

    cuboR = objets()
    cuboG = objets()
    cuboB = objets()
    baseR = objets()
    baseG = objets()
    baseB = objets()
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
            cuboR = objets()
            baseR = objets()
        else:
            if recR[2] == "T":
                baseR.area = recR[1]
                baseR.x = recR[3]
                baseR.y = recR[4]
            else:
                cuboR.area = recR[1]
                cuboR.x = recR[3]
                cuboR.y = recR[4]

        if recG[1] == "NA":
            cuboG = objets()
            baseG = objets()
        else:
            if recG[2] == "T":
                baseG.area = recG[1]
                baseG.x = recG[3]
                baseG.y = recG[4]
            else:
                cuboG.area = recG[1]
                cuboG.x = recG[3]
                cuboG.y = recG[4]

        if recB[1] == "NA":
            cuboB = objets()
            baseB = objets()
        else:
            if recB[2] == "T":
                baseB.area = recB[1]
                baseB.x = recB[3]
                baseB.y = recB[4]
            else:
                cuboB.area = recB[1]
                cuboB.x = recB[3]
                cuboB.y = recB[4]

        if cuboR.area > cuboG.area and cuboR.area > cuboB.area and cuboR.area > 0:

            objetivo = "R"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put([objetivo, cuboR.x, cuboR.y, cuboR.area])

        elif cuboG.area > cuboB.area and cuboG.area > 0:

            objetivo = "G"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put([objetivo, cuboG.x, cuboG.y, cuboG.area])
        elif cuboB.area > 0:

            objetivo = "B"
            if cola_objetivo.full():
                pass
            else:
                cola_objetivo.put([objetivo, cuboB.x, cuboB.y, cuboB.area])
        else:
            objetivo = ""

        if baseR.area > 0 and objetivo == "R":

            if cola_base.full():
                pass
            else:
                cola_base.put([objetivo, baseR.x, baseR.y, baseR.area])
        elif baseG.area > 0 and objetivo == "G":

            if cola_base.full():
                pass
            else:
                cola_base.put([objetivo, baseG.x, baseG.y, baseG.area])
        elif baseB.area > 0 and objetivo == "B":

            if cola_base.full():
                pass
            else:
                cola_base.put([objetivo, baseB.x, baseB.y, baseB.area])


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
    base = Queue(10)

    cliente_thread = multiprocessing.Process(
        target=cliente, args=(objetivo, base))

    videoRec_thread = multiprocessing.Process(
        target=videoRec, args=(framesR, framesG, framesB, framesF))

    R_thread = multiprocessing.Process(
        target=buscador, args=(framesR, val_R, "R", objetosR))

    G_thread = multiprocessing.Process(
        target=buscador, args=(framesG, val_G, "G", objetosG))

    B_thread = multiprocessing.Process(
        target=buscador, args=(framesB, val_B, "B", objetosB))

    control_thread = multiprocessing.Process(
        target=control, args=(framesF, objetosR, objetosG, objetosB, objetivo, base))

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
