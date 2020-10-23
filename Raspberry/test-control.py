import serial
import time
import paho.mqtt.client as mqtt
import sys

host = "192.168.1.11"
puerto = 1883
arduino = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)

Connected = False

def on_connect(client, userdata, flags, rc):

    if rc == 0:
        
        print("Connected to broker")
        client.subscribe("Control/orden")
        
    else:

        print("Connection failed")


def on_message(client, userdata, msg):
    global Connected
    print('%s %s' % (msg.topic, msg.payload.decode()))
    mensaje = msg.payload.decode()
    if mensaje == "on":
        arduino.write(str.encode('ON'))
        Connected = True
    else:
        pass
    #arduino.write(str.encode(mensaje))


def on_publish(client, userdata, result):
    #print("Enviado")
    pass


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish



client.connect(host, puerto)


client.loop_start()

while True:
    if(Connected == True):
        dist = arduino.readline()


        if(dist.decode() != ''):
            print(dist.decode())
            client.publish("Control/distancia", dist.decode())
    
