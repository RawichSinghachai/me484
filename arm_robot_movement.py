import numpy as np
import matplotlib.pyplot as plt

def inverse_kinematics(x, y, l1=18, l2=17):
    # Calculate joint angles using inverse kinematics
    r = np.sqrt(x**2 + y**2)
    try:
        theta3 = np.arccos((r**2 - l1**2 - l2**2) / (2 * l1 * l2))
        theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta3), l1 + l2 * np.cos(theta3))

        theta1 = np.degrees(theta1)
        theta3 = np.degrees(theta3)
        theta2 = 0

        if np.isnan(theta1) or np.isnan(theta3):
            raise ValueError("Computed angles resulted in NaN")

    except ValueError as e:
        print(f"Error in inverse kinematics: {e}")
        return None, None, None

    return theta1, theta2, theta3 # Results in degrees

def forward_kinematics(theta1, theta3, l1=18, l2=17):
    # Calculate end effector position using forward kinematics
    x = l1 * np.cos(np.radians(theta1)) + l2 * np.cos(np.radians(theta1 + theta3))
    y = l1 * np.sin(np.radians(theta1)) + l2 * np.sin(np.radians(theta1 + theta3))
    return x, y

def plot_robot_arm(target_x, target_y, l1=18, l2=17):
    # Calculate joint angles
    # theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, l1, l2)
    theta1, _, theta3 = inverse_kinematics(target_x, target_y, l1, l2)
    print(f"theta1 : {theta1} ,theta2 : {0} ,theta3 : {theta3} , ")

    # Calculate positions for plotting
    x0, y0 = 0, 0  # Base position
    x1, y1 = l1 * np.cos(np.radians(theta1)), l1 * np.sin(np.radians(theta1))  # First joint position
    x2, y2 = forward_kinematics(theta1, theta3, l1, l2)  # End effector position

    # Plotting
    plt.figure(figsize=(4, 4))
    plt.plot([x0, x1], [y0, y1], 'b-', linewidth=2, label='Link 1')
    plt.plot([x1, x2], [y1, y2], 'g-', linewidth=2, label='Link 2')
    plt.plot(x2, y2, 'ro', markersize=10, label='End Effector')
    plt.plot(target_x, target_y, 'mx', markersize=10, label='Target')

    plt.title('2-Link Robot Arm - Inverse Kinematics')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.legend()

    # Center the plot by adjusting the axis limits
    axis_limit = max(abs(target_x), abs(target_y), l1 + l2) + 1
    plt.xlim(-axis_limit, axis_limit)
    plt.ylim(-axis_limit, axis_limit)
    plt.axis('equal')

    print(f"Joint angles: theta1 = {theta1:.2f}°, theta3 = {theta3:.2f}°")
    print(f"End effector position: ({x2:.2f}, {y2:.2f})")

    plt.show()
#plot_robot_arm(14,12)