import paho.mqtt.client as mqtt
import time


host = "192.168.1.11"
topic = "Control/orden"
puerto = 1883


def on_connect(client, userdata, flags, rc):

    if rc == 0:

        print("Connected to broker")
        client.subscribe("Control/distancia")
        client.publish(topic, "on")
    else:

        print("Connection failed")


def on_message(client, userdata, msg):
    pass
    #print('%s %s' % (msg.topic, msg.payload))


def on_publish(client, userdata, result):
    print("Orden Enviada")
    pass


if __name__ == '__main__':
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_publish = on_publish

    client.connect(host, puerto)
    client.loop_start()
    client.publish(topic, "on")
    input()
    print("ON")

    client.publish(topic, "a60")

    time.sleep(2)

    client.publish(topic, "i100")

    time.sleep(0.1)

    client.publish(topic, "a60")

    time.sleep(1)

    client.publish(topic, "r60")   

    time.sleep(1)

    client.publish(topic, "d100")

    time.sleep(0.1)

    client.publish(topic, "a60")

    time.sleep(2)

    client.publish(topic, "s0")


    input()
