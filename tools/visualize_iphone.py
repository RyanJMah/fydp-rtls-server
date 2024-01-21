import time
import sys
import socket
import json
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Tuple, Any

# suppress the matplotlib warning
import warnings
warnings.filterwarnings("ignore")   # this suppresses all warnings, this is bad, who cares!

import includes
from localization_service import LocalizationService_State  # type: ignore
from gl_conf import GL_CONF                                 # type: ignore

ENDPOINT_HOST = "localhost"
ENDPOINT_PORT = GL_CONF.debug_endpoint.port

ANCHOR_0_COORDINATES = GL_CONF.anchors[0].get_coords()
ANCHOR_1_COORDINATES = GL_CONF.anchors[1].get_coords()
ANCHOR_2_COORDINATES = GL_CONF.anchors[2].get_coords()
ANCHOR_3_COORDINATES = GL_CONF.anchors[3].get_coords()

TABLE_DOT0 = (435, 240)
TABLE_DOT1 = (525, 240)
TABLE_DOT2 = (435, 360)
TABLE_DOT3 = (525, 360)

# FLOORPLAN_WIDTH  = 700
# FLOORPLAN_HEIGHT = 800

FLOORPLAN_WIDTH  = 1140
FLOORPLAN_HEIGHT = 1200

# ANCHOR_0_COORDINATES = (615, 0, 273)
# ANCHOR_1_COORDINATES = (615, 520, 263)
# ANCHOR_2_COORDINATES = (0, 0, 277)
# ANCHOR_3_COORDINATES = (123, 520, 263)

def connect_to_server():
    while (1):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            s.connect( (ENDPOINT_HOST, ENDPOINT_PORT) )
            print('Connected to server')

            return s

        except:
            print('Failed to connect to server, retrying...')
            time.sleep(1)

def receive_debug_packet(sock) -> bytes:
    data = sock.recv(4)

    if len(data) == 0:
        return bytes()

    length = int.from_bytes(data, "big")
    data = sock.recv(length)

    return data

sock = connect_to_server()

g_path: List[ Tuple[float, float] ] = []
g_path_ax_dots: List[Any] = []

g_target_heading: float = 0.0

g_x = 0
g_y = 0

# Function to update the position of the dot in the plot
def update_dot(frame):
    global sock
    global g_path, g_path_ax_dots
    global g_x, g_y
    global g_target_heading

    data = receive_debug_packet(sock)

    if len(data) == 0:
        print('Server disconnected, reconnecting...')
        sock = connect_to_server()

    try:
        data = json.loads(data.decode())
    except Exception as e:
        print(f"WARNING: dropped packet, failed to parse json: {e}")
        return

    tag  = data["tag"]
    data = data["data"]

    if tag == "loc_state":
        data = LocalizationService_State(**data)

        g_x = data.x
        g_y = data.y

        x = data.x
        y = data.y
        z = data.z
        theta = data.angle_deg

        r0 = data.r0
        r1 = data.r1
        r2 = data.r2
        r3 = data.r3

        phi0 = data.phi0
        phi1 = data.phi1
        phi2 = data.phi2
        phi3 = data.phi3

        los0 = data.los0
        los1 = data.los1
        los2 = data.los2
        los3 = data.los3

        critical_anchor = data.critical_anchor

        anchor0.set_radius(r0)
        anchor1.set_radius(r1)
        anchor2.set_radius(r2)
        anchor3.set_radius(r3)

        if los0:
            anchor0.set_color('g')
        else:
            anchor0.set_color('r')

        if los1:
            anchor1.set_color('g')
        else:
            anchor1.set_color('r')

        if los2:
            anchor2.set_color('g')
        else:
            anchor2.set_color('r')

        if los3:
            anchor3.set_color('g')
        else:
            anchor3.set_color('r')

        for i, a in enumerate([anchor0, anchor1, anchor2, anchor3]):
            if i == critical_anchor:
                a.set_fill(True)
                a.set_alpha(0.3)
            else:
                a.set_fill(False)
                a.set_alpha(1)

        dot1.set_data(x, y)  # Update the dot's position

        label.set_text(f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}, Theta: {theta:.2f}")

    elif tag == "path":
        g_path = data

        # clear the previous path dots
        for p in g_path_ax_dots:
            p.remove()

        g_path_ax_dots.clear()

        for p in g_path:
            x = p[0]
            y = p[1]

            path_dot, = ax.plot([], [], 'yo', markersize=8)
            path_dot.set_data(x, y)

            g_path_ax_dots.append(path_dot)

    elif tag == "target_heading":
        g_target_heading = data


def update_path_line_thread():
    global g_target_heading, g_x, g_y

    while (1):
        target_heading = g_target_heading + 90  # 0 degrees should point North, not East

        # Draw line from the user's current position to some point in the direction of the target heading
        path_line_x0 = g_x
        path_line_y0 = g_y

        line_length = 100
        path_line_x1 = g_x + line_length * np.cos(np.deg2rad(target_heading))
        path_line_y1 = g_y + line_length * np.sin(np.deg2rad(target_heading))

        path_line.set_data([path_line_x0, path_line_x1], [path_line_y0, path_line_y1])
    

        time.sleep(GL_CONF.update_period_secs)


# Create a figure and axis
fig, ax = plt.subplots()

# ax.set_xlim(0, 700)  # Set the X-axis limits
# ax.set_ylim(0, 800)  # Set the X-axis limits

ax.set_xlim(0, FLOORPLAN_WIDTH)  # Set the X-axis limits
ax.set_ylim(0, FLOORPLAN_HEIGHT)  # Set the X-axis limits



# Create the dot as a scatter plot
dot1, = ax.plot([], [], 'ro', markersize=8)
# dot2, = ax.plot([], [], 'ro', markersize=8)

# bottom left
table_dot0, = ax.plot([], [], 'bo', markersize=8)
table_dot0.set_data( *TABLE_DOT0 )


table_dot1, = ax.plot([], [], 'bo', markersize=8)
table_dot1.set_data( *TABLE_DOT1 )

table_dot2, = ax.plot([], [], 'bo', markersize=8)
table_dot2.set_data( *TABLE_DOT2 )

table_dot3, = ax.plot([], [], 'bo', markersize=8)
table_dot3.set_data( *TABLE_DOT3 )


path_line = plt.Line2D([], [], color='g', linewidth=2)
ax.add_line(path_line)

# path_dot1 = plt.Circle([138, 138], 50, color='g', fill=False)
# ax.add_patch(path_dot1)

# path_dot2 = plt.Circle([437, 79], 50, color='g', fill=False)
# ax.add_patch(path_dot2)

# path_dot2 = plt.Circle([436, 308], 50, color='g', fill=False)
# ax.add_patch(path_dot2)



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

ax.set_aspect("auto")

label = ax.text(FLOORPLAN_WIDTH/2, FLOORPLAN_HEIGHT - 20, "", ha='center', va='center', fontsize=14, color='r')

# Create the animation
animation = FuncAnimation(fig, update_dot, frames=range(200), interval=100)

# Create a thread to update the path line
thread = threading.Thread(target=update_path_line_thread)
thread.start()

def run():
    plt.show()

if __name__ == "__main__":
    run()
