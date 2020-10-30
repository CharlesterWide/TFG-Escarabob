import serial
import time

arduino = serial.Serial('COM3', 115200)
time.sleep(2)

arduino.write(str.encode('on\n'))
time.sleep(1)
leido = arduino.readline().decode()
print(leido)
leido = ''


cont = 0

while cont < 10:
    leido = arduino.readline().decode()
    print(leido)
    arduino.write(str.encode('i100\n'))
    time.sleep(0.05)
    arduino.write(str.encode('s0\n'))
    time.sleep(0.5)
    cont += 1



arduino.write(str.encode('s0\n'))
    





        


