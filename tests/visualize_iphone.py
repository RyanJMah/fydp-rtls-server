import time
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ANCHOR_0_COORDINATES = (0, 0)
ANCHOR_1_COORDINATES = (276, 520)
ANCHOR_2_COORDINATES = (617, 0)

g_x = 0
g_y = 0

coordinate_q: queue.Queue = queue.Queue()

def push_coordinates(x, y, r0, r1, r2):
    coordinate_q.put( (x, y, r0, r1, r2) )

# Dummy function to simulate the user's position (replace this with your server data)
def get_user_position():
    return g_x, g_y

# Function to update the position of the dot in the plot
def update_dot(frame):
    x, y, r0, r1, r2 = coordinate_q.get()  # Get the user's current position

    # x, y = get_user_position()  # Get the user's current position

    anchor0.set_radius(r0)
    anchor1.set_radius(r1)
    anchor2.set_radius(r2)

    dot.set_data(x, y)  # Update the dot's position
    return dot,

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
dot, = ax.plot([], [], 'ro', markersize=8)

TABLE_X = 230
TABLE_Y = 112

table_dot0, = ax.plot([], [], 'bo', markersize=8)
table_dot0.set_data(TABLE_X, TABLE_Y)

table_dot1, = ax.plot([], [], 'bo', markersize=8)
table_dot1.set_data(TABLE_X + 178, TABLE_Y + 178)

table_dot2, = ax.plot([], [], 'bo', markersize=8)
table_dot2.set_data(TABLE_X, TABLE_Y + 178)

table_dot3, = ax.plot([], [], 'bo', markersize=8)
table_dot3.set_data(TABLE_X + 178, TABLE_Y)

anchor0 = plt.Circle(ANCHOR_0_COORDINATES, 20, color='b', fill=False)
anchor0.set_radius(100)
ax.add_patch(anchor0)

anchor1 = plt.Circle(ANCHOR_1_COORDINATES, 20, color='b', fill=False)
anchor1.set_radius(100)
ax.add_patch(anchor1)

anchor2 = plt.Circle(ANCHOR_2_COORDINATES, 20, color='b', fill=False)
anchor2.set_radius(100)
ax.add_patch(anchor2)

# Create the animation
animation = FuncAnimation(fig, update_dot, frames=range(200), interval=100)

def run():
    plt.show()

if __name__ == "__main__":
    run()
