import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np



def update(i):
    line.set_xdata(x + i/10)
    return line,

def init():
    line.set_xdata(x)
    return line,


fig, ax = plt.subplots()

x = np.arange(0, 10, 0.01)
y = 2*x

line, = ax.plot(x, y, color='pink')

ani = animation.FuncAnimation(fig=fig, func=update, init_func=init, interval=20, blit=False)

plt.show()
