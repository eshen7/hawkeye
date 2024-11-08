import matplotlib.pyplot as plt
import sys
import json
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# The data provided
with open(sys.argv[1], 'r') as file:
    data = json.load(file)

# Extract robot positions and AprilTag positions
robot_positions_x = [sample["robotPoseX"] for sample in data["samples"]]
robot_positions_y = [sample["robotPoseY"] for sample in data["samples"]]
robot_positions_z = [sample["robotPoseZ"] for sample in data["samples"]]

# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set axis labels and plot limits based on the data
ax.set_xlim(min(robot_positions_x) - 0.1, max(robot_positions_x) + 0.1)
ax.set_ylim(min(robot_positions_y) - 0.1, max(robot_positions_y) + 0.1)
ax.set_zlim(min(robot_positions_z) - 0.1, max(robot_positions_z) + 0.1)
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('Robot and AprilTag Positions Replay (3D)')


# Function to plot the current state of the robot and AprilTags in 3D
def update_frame(i):
    # Clear the previous frame
    ax.cla()

    # Set axis limits and labels again (since we clear the plot)
    ax.set_xlim(min(robot_positions_x) - 0.1, max(robot_positions_x) + 0.1)
    ax.set_ylim(min(robot_positions_y) - 0.1, max(robot_positions_y) + 0.1)
    ax.set_zlim(min(robot_positions_z) - 0.1, max(robot_positions_z) + 0.1)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Robot and AprilTag Positions Replay (3D)')

    # Get the current sample
    sample = data["samples"][i]

    # Plot the robot's current position in 3D
    robot_x = sample["robotPoseX"]
    robot_y = sample["robotPoseY"]
    robot_z = sample["robotPoseZ"]
    ax.scatter(robot_x, robot_y, robot_z, color='blue', s=100, zorder=2)
    ax.text(robot_x, robot_y, robot_z, f'Robot\n({robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f})', fontsize=8,
            color='blue', zorder=3)

    # Plot the tags detected in this frame in 3D
    for tag in sample["tags"]:
        tag_x = tag["apriltagX"]
        tag_y = tag["apriltagY"]
        tag_z = tag["apriltagZ"]
        ax.scatter(tag_x, tag_y, tag_z, color='red', s=50, zorder=1)
        ax.text(tag_x, tag_y, tag_z, f'Tag {tag["apriltagId"]}', fontsize=8, color='red', zorder=3)


# Create an animation that updates the plot
ani = FuncAnimation(fig, update_frame, frames=len(data["samples"]), interval=5)

# Show the animated plot
plt.show()
