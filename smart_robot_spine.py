# I have a robot joint module for a robot spine
# this joint has 2 revolute joints.
# each of the rev joints is sensed by a hall effect encoder (same as a rotary encoder)
# on the joint bracket, it also has an IMU that mearsures the orientation of the joint bracket in yaw, pitch, roll
# The module will feedback the position of the two revolute joints and the orientation of the joint bracket
# feedback will be via USB serial communication. We can use pyserial to communicate with the module
# The module will feedback at 20 ms intervals
# the feedback format will be as follows:
# |ENCODER1| 4141.80	-138.30	-2093.70	91.91	 |ENCODER2| -4742.55	-849.00	-231.90	100.15	 |IMU| 48.85	12.65	2.36
# it is magnetic strenth for x,y,z and the shaft angle in degrees for each encoder
# for IMU, it is yaw, pitch, roll in degrees
# the resting angle for both encoder is 90 degrees. where the joint rod should be staright pointing down, and two rings share plane with the joint bracket.

# We want to implement a visualizer based on matplotlib to visualize the joint positions and orientations in real-time
# now let's implement the code for this

# Note: I want to draw the robot joint in 3D space, not just show the plots of the values
# We can show a ring bracket as the joint base. The IMU is installed on the bracket, so joint base orientation is the IMU orientation
# Then, the two revolute joints center cross the ring bracket center
# the two revolute joint axes are perpendicular to each other.
# show a second ring bracket for the first revolute joint, rotated by encoder1 angle around the first joint axis
# then show a third ring bracket for the second revolute joint, rotated by encoder2 angle
# then, show a rod perpendicular to the second joint bracket as the end of the joint module

import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
from matplotlib.animation import FuncAnimation
import threading
import re
import queue
from scipy.spatial.transform import Rotation
# Serial port configuration
SERIAL_PORT = 'COM10'  # Change this to your serial port
BAUD_RATE = 115200
READ_INTERVAL = 0.02  # 20 ms
# Queue to hold incoming data
data_queue = queue.Queue()
# Function to read data from serial port
def read_serial_data():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            data_queue.put(line)
        time.sleep(READ_INTERVAL)
# Function to parse the incoming data
def parse_data(line):
    # Pattern now matches: |ENCODER1| x y z angle |ENCODER2| x y z angle |IMU| yaw pitch roll
    pattern = r'\|ENCODER1\|\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+\|ENCODER2\|\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+\|IMU\|\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)'
    match = re.match(pattern, line)
    if match:
        # Extract encoder angles (4th value for each encoder) - SWAPPED!
        encoder1_angle = float(match.group(8))  # Now reads ENCODER2 value
        encoder2_angle = float(match.group(4))  # Now reads ENCODER1 value
        
        # Adjust for 90-degree resting position (subtract 90 to make resting position = 0)
        encoder1 = -(encoder1_angle - 90.0)  # Inverted to reverse direction
        encoder2 = encoder2_angle - 90.0
        
        # Extract IMU values
        imu_yaw = float(match.group(9))
        imu_pitch = float(match.group(10))
        imu_roll = float(match.group(11))
        
        return encoder1, encoder2, imu_yaw, imu_pitch, imu_roll
    return None

# Current sensor values
current_encoder1 = 0.0
current_encoder2 = 0.0
current_imu_yaw = 0.0
current_imu_pitch = 0.0
current_imu_roll = 0.0

# Initialize matplotlib 3D figure
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Robot Spine Joint 3D Visualization', fontsize=16)
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

# Helper function to create a ring bracket
def create_ring(radius=0.5, num_points=20):
    """Create a ring bracket as a 3D circle"""
    theta = np.linspace(0, 2*np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.zeros_like(theta)
    return x, y, z

# Helper function to apply rotation matrix
def apply_rotation(points, yaw, pitch, roll):
    """Apply yaw, pitch, roll rotation to points"""
    # Convert degrees to radians
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)
    
    # Rotation matrices
    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                   [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                   [0, 0, 1]])
    
    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                   [0, 1, 0],
                   [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll_rad), -np.sin(roll_rad)],
                   [0, np.sin(roll_rad), np.cos(roll_rad)]])
    
    # Combined rotation: Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    
    # Apply rotation
    return R @ points

# Helper function to rotate around an axis
def rotate_around_axis(points, angle, axis):
    """Rotate points around a given axis by angle (in degrees)"""
    angle_rad = np.radians(angle)
    axis = axis / np.linalg.norm(axis)  # Normalize
    
    # Rodrigues' rotation formula
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    
    R = np.eye(3) + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)
    
    return R @ points

start_time = time.time()

# Animation update function
def update_plot(frame):
    global current_encoder1, current_encoder2, current_imu_yaw, current_imu_pitch, current_imu_roll
    
    # Process all available data from queue
    while not data_queue.empty():
        try:
            line = data_queue.get_nowait()
            parsed_data = parse_data(line)
            
            if parsed_data:
                current_encoder1, current_encoder2, current_imu_yaw, current_imu_pitch, current_imu_roll = parsed_data
        except queue.Empty:
            break
    
    # Clear the plot
    ax.clear()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Robot Spine Joint 3D\nEnc1: {current_encoder1:.1f}° Enc2: {current_encoder2:.1f}°\nYaw: {current_imu_yaw:.1f}° Pitch: {current_imu_pitch:.1f}° Roll: {current_imu_roll:.1f}°')
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    
    # Create base ring bracket (largest)
    ring_x, ring_y, ring_z = create_ring(radius=0.5)
    base_ring = np.vstack([ring_x, ring_y, ring_z])
    
    # Apply IMU orientation to base ring (yaw, pitch, roll)
    base_ring_rotated = apply_rotation(base_ring, current_imu_yaw, current_imu_pitch, current_imu_roll)
    
    # Plot base ring (blue, thickest)
    ax.plot(base_ring_rotated[0, :], base_ring_rotated[1, :], base_ring_rotated[2, :], 'b-', linewidth=5, label='Base Bracket (IMU)')
    
    # Define first joint axis (Y-axis - rotated 90° CCW from X-axis in base frame)
    joint1_axis = np.array([0, 1, 0])
    joint1_axis_world = apply_rotation(joint1_axis.reshape(3, 1), current_imu_yaw, current_imu_pitch, current_imu_roll).flatten()
    
    # Create second ring bracket (half radius of base)
    ring2_x, ring2_y, ring2_z = create_ring(radius=0.25)
    ring2 = np.vstack([ring2_x, ring2_y, ring2_z])
    
    # First apply encoder1 rotation around Y-axis
    ring2_rotated = rotate_around_axis(ring2, current_encoder1, np.array([0, 1, 0]))
    # Then apply IMU orientation
    ring2_rotated = apply_rotation(ring2_rotated, current_imu_yaw, current_imu_pitch, current_imu_roll)
    
    # Plot second ring (red, medium thickness)
    ax.plot(ring2_rotated[0, :], ring2_rotated[1, :], ring2_rotated[2, :], 'r-', linewidth=3, label='Joint 1 Bracket')
    
    # Define second joint axis (-X-axis - rotated 90° CCW from Y-axis after first rotation)
    joint2_axis = np.array([-1, 0, 0])
    # Apply encoder1 rotation to get the axis orientation
    joint2_axis_local = rotate_around_axis(joint2_axis.reshape(3, 1), current_encoder1, np.array([0, 1, 0])).flatten()
    # Then apply IMU orientation
    joint2_axis_world = apply_rotation(joint2_axis_local.reshape(3, 1), current_imu_yaw, current_imu_pitch, current_imu_roll).flatten()
    
    # Create third ring bracket (half radius of red, quarter of base)
    ring3_x, ring3_y, ring3_z = create_ring(radius=0.125)
    ring3 = np.vstack([ring3_x, ring3_y, ring3_z])
    
    # Apply encoder1 rotation first around Y-axis
    ring3_rotated = rotate_around_axis(ring3, current_encoder1, np.array([0, 1, 0]))
    # Then apply encoder2 rotation around -X-axis
    ring3_rotated = rotate_around_axis(ring3_rotated, current_encoder2, np.array([-1, 0, 0]))
    # Finally apply IMU orientation
    ring3_rotated = apply_rotation(ring3_rotated, current_imu_yaw, current_imu_pitch, current_imu_roll)
    
    # Plot third ring (green, thinner)
    ax.plot(ring3_rotated[0, :], ring3_rotated[1, :], ring3_rotated[2, :], 'g-', linewidth=2, label='Joint 2 Bracket')
    
    # Create end rod perpendicular to third bracket (along negative Z-axis, pointing down)
    rod_length = 0.8
    rod_start = np.array([0, 0, 0])
    rod_end = np.array([0, 0, -rod_length])  # Inverted to point down
    
    # Apply all rotations to rod
    rod_points = np.vstack([rod_start, rod_end]).T
    rod_rotated = rotate_around_axis(rod_points, current_encoder1, np.array([0, 1, 0]))
    rod_rotated = rotate_around_axis(rod_rotated, current_encoder2, np.array([-1, 0, 0]))
    rod_rotated = apply_rotation(rod_rotated, current_imu_yaw, current_imu_pitch, current_imu_roll)
    
    # Plot rod (magenta)
    ax.plot(rod_rotated[0, :], rod_rotated[1, :], rod_rotated[2, :], 'm-', linewidth=5, label='End Rod')
    
    # Add coordinate frame at origin
    axis_length = 0.3
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.3, alpha=0.3)
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.3, alpha=0.3)
    ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.3, alpha=0.3)
    
    ax.legend()
    
    return ax,

# Main execution
if __name__ == '__main__':
    # Start serial reading thread
    serial_thread = threading.Thread(target=read_serial_data, daemon=True)
    serial_thread.start()
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False)
    
    plt.tight_layout()
    plt.show()
