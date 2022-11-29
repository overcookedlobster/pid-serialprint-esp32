import serial.tools.list_ports
import math 
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from time import sleep
time = [0]
out = [0]
x_curr = 0
x_prev = 0
y_curr = 0
dt = 0.01
t = 0
y_prev = 0
distance = 0
ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portList = []

for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

val = input("select Port: /dev/ttyUSB")

serialInst.baudrate = input("select baudrate: ")
serialInst.port = ("/dev/ttyUSB" + val)
serialInst.open()

def animate(i):
    plt.cla()
    plt.plot(time, out)

while t<60:
    if serialInst.in_waiting:
        packet = serialInst.readline()
        packet_string = packet.decode('utf-8').rstrip("\n")
        x_curr = float(packet_string)
        y_curr = (0.9917*y_prev) + (0.00415)*(x_curr + x_prev)
        t += dt
        distance += y_curr*dt
        y_prev = y_curr
        x_prev = x_curr
        time.append(t)
        out.append(y_curr)
        print(str(round(t, 3)) +" PID_out: "+ str(round(x_curr, 7)) + " Kecepatan Sistem: " + str(round(y_curr, 5)) + " distance: "+ str(distance)) 

    serialInst.write(str(round(y_curr, 5)).encode('utf-8'))
    sleep(0.01)
ani = FuncAnimation(plt.gcf(), animate, interval = 100)

plt.tight_layout()
plt.show()
