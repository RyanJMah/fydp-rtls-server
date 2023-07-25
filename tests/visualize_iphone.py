import time
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

g_x = 0
g_y = 0

coordinate_q: queue.Queue = queue.Queue()

def push_coordinates(x, y):
    coordinate_q.put( (x, y) )

# Dummy function to simulate the user's position (replace this with your server data)
def get_user_position():
    return g_x, g_y

# Function to update the position of the dot in the plot
def update_dot(frame):
    x, y = coordinate_q.get()  # Get the user's current position

    # x, y = get_user_position()  # Get the user's current position

    dot.set_data(x, y)  # Update the dot's position
    return dot,

def update_dot_thread():
    global g_x, g_y
    while (1):
        g_x += 0.1
        g_y += 0.1
        time.sleep(0.2)

t = threading.Thread(target=update_dot_thread, daemon=True)
t.start()

# Create a figure and axis
fig, ax = plt.subplots()
ax.set_xlim(0, 621)  # Set the X-axis limits
ax.set_ylim(0, 520)    # Set the Y-axis limits

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


# Create the animation
animation = FuncAnimation(fig, update_dot, frames=range(200), interval=100)

def run():
    plt.show()

if __name__ == "__main__":
    run()
