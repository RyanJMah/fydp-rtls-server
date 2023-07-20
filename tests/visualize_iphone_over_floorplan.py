import time
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

g_x = 0
g_y = 0

# Dummy function to simulate the user's position (replace this with your server data)
def get_user_position():
    return g_x, g_y

# Function to update the position of the dot in the plot
def update_dot(frame):
    x, y = get_user_position()  # Get the user's current position
    dot.set_data(x, y)  # Update the dot's position
    return dot,

def update_dot_thread():
    global g_x, g_y
    while (1):
        g_x += 0.5
        g_y += 0
        time.sleep(1)

t = threading.Thread(target=update_dot_thread, daemon=True)
t.start()

# Load the floorplan image (replace 'floorplan.jpg' with your actual image path)
floorplan_img = plt.imread('/Users/ryanmah/Downloads/067E7_04FLR-1.png')

# Get the aspect ratio of the image and calculate the scaling factors
aspect_ratio = floorplan_img.shape[1] / floorplan_img.shape[0]

# Calculate the scaling factor (meters per pixel)
scale_factor = 0.03785488958990536

# Create a figure and axis with the calculated aspect ratio and scaling factors
fig, ax = plt.subplots()
ax.set_aspect(aspect_ratio)
ax.set_xlim(0, floorplan_img.shape[1] * scale_factor)
ax.set_ylim(0, floorplan_img.shape[0] * scale_factor)

# Display the floorplan image as the background
ax.imshow(floorplan_img, extent=[0, floorplan_img.shape[1] * scale_factor, 0, floorplan_img.shape[0] * scale_factor])

# Create the dot as a scatter plot
dot, = ax.plot([], [], 'ro', markersize=10)

# Create the animation
animation = FuncAnimation(fig, update_dot, frames=range(200), interval=100)

plt.show()

