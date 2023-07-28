import time
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ANCHOR_0_COORDINATES = (615, 0, 273)
ANCHOR_1_COORDINATES = (615, 520, 263)
ANCHOR_2_COORDINATES = (0, 0, 277)
ANCHOR_3_COORDINATES = (123, 520, 263)

g_x = 0
g_y = 0

coordinate_q: queue.Queue = queue.Queue()

def push_coordinates( x, y, theta,
                      r0, phi0,
                      r1, phi1,
                      r2, phi2,
                      r3, phi3,
                      los0, los1, los2, los3,
                      critical_anchor ):
    coordinate_q.put(( x, y, theta,
                      r0, phi0,
                      r1, phi1,
                      r2, phi2,
                      r3, phi3,
                      los0, los1, los2, los3,
                      critical_anchor ))

# Dummy function to simulate the user's position (replace this with your server data)
def get_user_position():
    return g_x, g_y

# Function to update the position of the dot in the plot
def update_dot(frame):
    x, y, theta, \
    r0, phi0, \
    r1, phi1, \
    r2, phi2, \
    r3, phi3, \
    los0, los1, los2, los3, \
    critical_anchor = coordinate_q.get()  # Get the user's current position

    anchor0.set_radius(r0)
    anchor1.set_radius(r1)
    anchor2.set_radius(r2)
    anchor3.set_radius(r3)

    # if los0:
    #     anchor0.set_color('g')
    # else:
    #     anchor0.set_color('r')

    # if los1:
    #     anchor1.set_color('g')
    # else:
    #     anchor1.set_color('r')

    # if los2:
    #     anchor2.set_color('g')
    # else:
    #     anchor2.set_color('r')

    # if los3:
    #     anchor3.set_color('g')
    # else:
    #     anchor3.set_color('r')

    for i, a in enumerate([anchor0, anchor1, anchor2, anchor3]):
        if i == critical_anchor:
            a.set_fill('g')
            a.set_alpha(0.3)
        else:
            a.set_fill(False)
            a.set_alpha(1)


    dot1.set_data(x, y)  # Update the dot's position

def update_dot_thread():
    global g_x, g_y

    while (1):
        g_x += 0.1
        g_y += 0.1
        time.sleep(0.2)

# t = threading.Thread(target=update_dot_thread, daemon=True)
# t.start()

# Create a figure and axis
fig, ax = plt.subplots()
# ax.set_xlim(0, 621)  # Set the X-axis limits
# ax.set_ylim(0, 520)    # Set the Y-axis limits
ax.set_xlim(0, 700)  # Set the X-axis limits
ax.set_ylim(0, 700)  # Set the X-axis limits

# Create the dot as a scatter plot
dot1, = ax.plot([], [], 'ro', markersize=8)
# dot2, = ax.plot([], [], 'ro', markersize=8)

# bottom left
table_dot0, = ax.plot([], [], 'bo', markersize=8)
table_dot0.set_data(216, 164)

table_dot1, = ax.plot([], [], 'bo', markersize=8)
table_dot1.set_data(385, 168)

table_dot2, = ax.plot([], [], 'bo', markersize=8)
table_dot2.set_data(216, 229)

table_dot3, = ax.plot([], [], 'bo', markersize=8)
table_dot3.set_data(385, 232)


path_dot1 = plt.Circle([138, 138], 40, color='g', fill=False)
ax.add_patch(path_dot1)

path_dot2 = plt.Circle([437, 79], 40, color='g', fill=False)
ax.add_patch(path_dot2)

path_dot2 = plt.Circle([436, 308], 40, color='g', fill=False)
ax.add_patch(path_dot2)



anchor0 = plt.Circle(ANCHOR_0_COORDINATES, 20, color='b', fill=False)
anchor0.set_radius(100)
ax.add_patch(anchor0)

anchor1 = plt.Circle(ANCHOR_1_COORDINATES, 20, color='b', fill=False)
anchor1.set_radius(100)
ax.add_patch(anchor1)

anchor2 = plt.Circle(ANCHOR_2_COORDINATES, 20, color='b', fill=False)
anchor2.set_radius(100)
ax.add_patch(anchor2)

anchor3 = plt.Circle(ANCHOR_3_COORDINATES, 20, color='b', fill=False)
anchor3.set_radius(100)
ax.add_patch(anchor3)

# Create the animation
animation = FuncAnimation(fig, update_dot, frames=range(200), interval=100)

def run():
    plt.show()

if __name__ == "__main__":
    run()