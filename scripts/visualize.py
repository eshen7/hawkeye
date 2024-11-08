import matplotlib.pyplot as plt
import sys
import json
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Data samples (truncated for brevity, include all your data samples here)
with open(sys.argv[1], 'r') as file:
    data = json.load(file)

# Extract the X, Y, and Z positions from the data
x_data = [sample["robotPoseX"] for sample in data["samples"]]
y_data = [sample["robotPoseY"] for sample in data["samples"]]
z_data = [sample["robotPoseZ"] for sample in data["samples"]]

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set axis limits based on the data range
ax.set_xlim(min(x_data) - 0.2, max(x_data) + 0.2)
ax.set_ylim(min(y_data) - 0.2, max(y_data) + 0.2)
ax.set_zlim(min(z_data) - 0.2, max(z_data) + 0.2)

# Initialize the plot with an empty line
line, = ax.plot([], [], [], lw=2)


# Function to initialize the plot
def init():
    line.set_data([], [])
    line.set_3d_properties([])
    return line,


# Function to update the plot at each frame
def update(frame):
    line.set_data(x_data[:frame], y_data[:frame])
    line.set_3d_properties(z_data[:frame])
    return line,


# Create the animation
ani = FuncAnimation(fig, update, frames=len(x_data), init_func=init, blit=True, interval=5)

# Set labels and title
ax.set_xlabel('Robot Position X')
ax.set_ylabel('Robot Position Y')
ax.set_zlabel('Robot Position Z')
ax.set_title('3D Robot Movement Replay')

# Show the plot
plt.show()
