"""
3D Joint Visualizer for Robot Spine
Reads joint angles and IMU data from shared multiprocessing list
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation


def create_ring(radius=0.5, num_points=20):
    """Create a ring bracket as a 3D circle"""
    theta = np.linspace(0, 2*np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.zeros_like(theta)
    return x, y, z


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


def mp_joint_visualizer(mp_joint_data):
    """
    Multiprocessing visualizer function
    mp_joint_data: shared list [shaft_a_angle, shaft_b_angle, imu_yaw, imu_pitch, imu_roll]
    """
    
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
    
    def update_plot(frame):
        """Animation update function"""
        # Read current values from shared list
        try:
            current_encoder2 = mp_joint_data[0]  # shaft A
            current_encoder1 = mp_joint_data[1]  # shaft B
            current_imu_yaw = mp_joint_data[2]
            current_imu_pitch = mp_joint_data[3]
            current_imu_roll = mp_joint_data[4]

            current_encoder1 = -(current_encoder1 - 90.0)
            current_encoder2 = current_encoder2 - 90.0

        except (IndexError, TypeError):
            # If data not ready, use defaults
            current_encoder1 = 0.0
            current_encoder2 = 0.0
            current_imu_yaw = 0.0
            current_imu_pitch = 0.0
            current_imu_roll = 0.0
        
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
        
        # Create second ring bracket (half radius of base)
        ring2_x, ring2_y, ring2_z = create_ring(radius=0.25)
        ring2 = np.vstack([ring2_x, ring2_y, ring2_z])
        
        # First apply encoder1 rotation around Y-axis
        ring2_rotated = rotate_around_axis(ring2, current_encoder1, np.array([0, 1, 0]))
        # Then apply IMU orientation
        ring2_rotated = apply_rotation(ring2_rotated, current_imu_yaw, current_imu_pitch, current_imu_roll)
        
        # Plot second ring (red, medium thickness)
        ax.plot(ring2_rotated[0, :], ring2_rotated[1, :], ring2_rotated[2, :], 'r-', linewidth=3, label='Joint 1 Bracket')
        
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
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=10, blit=False)
    
    plt.tight_layout()
    plt.show()
