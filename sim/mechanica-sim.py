import numpy as np

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

import tkinter as tk
from tkinter import ttk, Button

def dh_transform(a, alpha, d, theta):
    """Compute the Denavit-Hartenberg (DH) transformation matrix"""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    T = np.array([[cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
                  [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
                  [0, sin_alpha, cos_alpha, d],
                  [0, 0, 0, 1]])
    return T

def forward_kinematics(dh_params):
    """Perform forward kinematics based on the DH parameters"""
    T = np.eye(4)
    transformations = []
    for params in dh_params:
        T = np.dot(T, dh_transform(*params))
        transformations.append(T)
    return transformations

def calculate_joint_angles(frame, behavior):
    """Calculate joint angles based on the desired behavior"""
    num_joints = len(behavior)
    joint_angles = np.zeros(num_joints)
    for i in range(num_joints):
        start_angle, end_angle, direction = behavior[i]
        angle_range = end_angle - start_angle
        if direction == 'clockwise':
            joint_angles[i] = np.radians(start_angle + (frame / 180) * angle_range)
        else:
            joint_angles[i] = np.radians(end_angle - (frame / 180) * angle_range)
    return joint_angles


def animate_arm(dh_params, behavior, num_frames = 360):
    global animation_running, animation

    num_joints = len(dh_params)

    # Create the main application window
    root = tk.Tk()
    root.title("Arm Simulation")
    root.geometry("1000x800")
 
    # Create a frame to hold the plots
    plots_frame = ttk.Frame(root, width=600, height=500)
    plots_frame.pack(side=tk.TOP, pady=10)

    # Create the 3D plot
    fig = plt.figure(figsize=(10, 6))
    ax_3d = fig.add_subplot(2, 2, 1, projection='3d')

    # Create the 2D plots
    ax_xy = fig.add_subplot(2, 2, 2)
    ax_yz = fig.add_subplot(2, 2, 3)
    ax_xz = fig.add_subplot(2, 2, 4)

    def start_pause_simulation():
        global animation, animation_running
        if animation is None:
            return
        
        if animation_running:
            animation.event_source.stop()  # Pause the animation
        else:
            animation.event_source.start()  # Resume the animation
        animation_running = not animation_running
        start_pause_button.config(text="Pause" if animation_running else "Start")

    def restart_simulation():
        global animation
        if animation is not None:
            # animation.event_source.stop()  # Stop the existing animation
            animation.frame_seq = animation.new_frame_seq()

    start_pause_button = Button(root, text="Start/Pause", command=start_pause_simulation)
    start_pause_button.pack(side=tk.LEFT, padx=10)

    restart_button = Button(root, text="Restart", command=restart_simulation)
    restart_button.pack(side=tk.LEFT, padx=10)

    # Create a frame to hold the joint angle display
    text_frame = ttk.Frame(root)
    text_frame.pack(side=tk.BOTTOM, pady=10)

    # Create a label for joint angle display
    joint_angle_label = ttk.Label(text_frame, text="Joint Angles:")
    joint_angle_label.pack(side=tk.TOP)
    
    # Create a text widget for joint angle display
    joint_angle_text = tk.Text(text_frame, width=40, height=10)

    def ctrlEvent(event):
        if event.state == 4 and event.keysym == 'c':
            content = joint_angle_text.selection_get()
            root.clipboard_clear()
            root.clipboard_append(content)
            return "break"
        elif event.state == 4 and event.keysym == 'v':
            joint_angle_text.insert('end', root.selection_get(selection='CLIPBOARD'))
            return "break"
        else:
            return "break"

    joint_angle_text.bind("<Key>", lambda e: ctrlEvent(e))
    joint_angle_text.pack(side=tk.TOP)

    def update_text(joint_angles):
        # Clear the text widget
        joint_angle_text.delete(1.0, tk.END)
        
        # Update joint angle display text
        for i, angle in enumerate(np.degrees(joint_angles)):
            joint_angle_text.insert(tk.END, f"Joint {i+1}: {angle:.2f}°\n")


    def update(frame):
        joint_angles = calculate_joint_angles(frame, behavior)
        update_text(joint_angles)

        for i in range(num_joints):
            dh_params[i][3] = joint_angles[i] # Update joint angles
        transformations = forward_kinematics(dh_params)
         
        # update arm configuration
        arm_config = np.zeros((num_joints + 1, 3))
        arm_config[1:, :] = np.array([T[:3, 3] for T in transformations])

        # calculate arm length
        arm_length = sum([params[0] for params in dh_params])

        # clear plots
        ax_3d.clear()
        ax_xy.clear()
        ax_yz.clear()
        ax_xz.clear()

        # update plots
        plot_arm_3d(ax_3d, arm_config, arm_length)
        plot_arm_2d(ax_xy, arm_config[:, :2], arm_length, 'X', 'Y')
        plot_arm_2d(ax_yz, arm_config[:, 1:], arm_length, 'Y', 'Z')
        plot_arm_2d(ax_xz, np.column_stack((arm_config[:, 0], arm_config[:, 2])), arm_length, 'X', 'Z')

    def plot_arm_3d(ax, arm_config, arm_length):
        # plot the 3D arm configuration on the given subplot
        ax.plot(arm_config[:, 0], arm_config[:, 1], arm_config[:, 2])
        
        # set plot limits and labels
        ax.set_xlim(-arm_length - 1, arm_length + 1)
        ax.set_ylim(-arm_length - 1, arm_length + 1)
        ax.set_zlim(-arm_length - 1, arm_length + 1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    def plot_arm_2d(ax, arm_config, arm_length, x_label, y_label):
        # plot the 2D arm configuration on the given subplot
        ax.plot(arm_config[:, 0], arm_config[:, 1])

        # set plot limits and labels
        ax.set_xlim(-arm_length - 1, arm_length + 1)
        ax.set_ylim(-arm_length - 1, arm_length + 1)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)

    # Create the FigureCanvasTkAgg widget
    canvas = FigureCanvasTkAgg(fig, master=plots_frame)
    canvas.draw()

    # Embed the plot in the Tkinter window
    canvas.get_tk_widget().pack()

    # Create animation
    animation = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

    # Start the main Tkinter event loop
    root.mainloop()


# Example usage
dh_parameters = [
    [0, np.pi/2, 0.5, 0],   # Joint 1: a, alpha, d, theta
    [1, 0, 0, 0],           # Joint 2: a, alpha, d, theta
    [1, 0, 0, 0],           # Joint 3: a, alpha, d, theta
    [0, np.pi/2, 0, 0],     # Joint 4: a, alpha, d, theta
    [0, -np.pi/2, 0, 0]     # Joint 5: a, alpha, d, theta
]

behavior = [
    (0, 90, 'clockwise'),  # Joint 1 sweeps from 0° to 90° in a clockwise direction
    (-45, 45, 'counterclockwise'),  # Joint 2 sweeps from -45° to 45° in a counterclockwise direction
    (0, 180, 'counterclockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    (0, 180, 'clockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    (0, 180, 'counterclockwise')  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
]

animation_running = True
animation = None

animate_arm(dh_parameters, behavior)

