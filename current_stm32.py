import serial
import time
# import psutil
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def get_value():
    with serial.Serial(port="COM13") as ser:
        data = ser.readline().decode().replace('\x00','').replace('\n','')
        data = int(data)/126.2 #voltage in mV
        # if data !=0:
        #     data = 3/data   #current in mAmps
    return data



fig = plt.figure()
plt.xlim([-33000,33000])
ax = fig.add_subplot(111)
fig.show()
i = 0
x, y = [], []

def animate(i):
    x.append(i)
    y.append(get_value())
    ax.clear()
    ax.plot(x[-200:],y[-200:])

ani = animation.FuncAnimation(fig, animate, interval = 100)
plt.tight_layout()
plt.show()