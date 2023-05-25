##################################################
## mechanica-sim.py 
## python forward/inverse kinematics simulator
##################################################
## Author: Brian Costantino
## License: MIT, see footer for more info
## Version: 0.1.0
## Status: Active
##################################################

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

def calculate_jacobian(fk_result):
    # Extract the number of joints
    num_joints = len(fk_result)

    # Initialize the Jacobian matrix
    jacobian = np.zeros((6, num_joints))

    # Extract the end effector position and orientation
    end_effector_position = fk_result[-1][:3, 3]
    end_effector_orientation = fk_result[-1][:3, :3]

    for i in range(num_joints):
        # Extract the relevant information for the joint
        joint_transform = fk_result[i]
        joint_axis = joint_transform[:3, 2]
        joint_position = joint_transform[:3, 3]

        # Calculate the linear velocity component of the Jacobian
        linear_velocity = np.cross(joint_axis, end_effector_position - joint_position)

        # Calculate the angular velocity component of the Jacobian
        angular_velocity = joint_axis

        # Combine the linear and angular velocities into the Jacobian matrix
        jacobian[:3, i] = linear_velocity
        jacobian[3:, i] = np.dot(end_effector_orientation, angular_velocity)

    return jacobian

def inverse_kinematics(dh_parameters, end_effector):
    # Extract the number of joints
    num_joints = len(dh_parameters)

    # Initialize the joint angles
    theta = np.zeros(num_joints)

    # Set the convergence threshold and maximum iterations
    threshold = 1e-6
    max_iterations = 100

    # Initialize the iteration counter and the error
    iterations = 0
    error = np.inf

    while error > threshold and iterations < max_iterations:
        # Perform forward kinematics with the current joint angles
        fk_result = forward_kinematics(dh_parameters)
        current_end_effector = fk_result[-1][:3, 3]

        # Calculate the error
        error = np.linalg.norm(end_effector - current_end_effector)

        if error <= threshold:
            break

        # Calculate the Jacobian matrix
        jacobian = calculate_jacobian(fk_result)

        # Update the joint angles using the pseudoinverse of the Jacobian
        delta_theta = np.linalg.pinv(jacobian) @ (end_effector - current_end_effector)
        theta += delta_theta

        # Update the DH parameters
        for i in range(num_joints):
            dh_parameters[i][3] = theta[i]

        iterations += 1

    if iterations == max_iterations:
        print("Inverse kinematics did not converge.")

    return np.degrees(theta)

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

def calculate_end_effector_coordinates(dh_parameters):
    # Initialize the transformation matrix
    T = np.eye(4)
    coords = []
    # Iterate over the DH parameters
    for i, dh in enumerate(dh_parameters):
        a, alpha, d, theta = dh

        # Compute the transformation matrix for the current joint
        dh_matrix = dh_transform(a, alpha, d, theta)

        # Multiply the transformation matrix with the previous one
        T = np.matmul(T, dh_matrix)
        end_effector_coords = T[:3,3]

        coords.append((end_effector_coords[0], end_effector_coords[1], end_effector_coords[2]))

    return coords

def calculate_end_effector_coordinates_2(dh_params):
    num_joints = len(dh_params)
    end_effector_coords = np.zeros((num_joints + 1, 3))  # Initialize end effector coordinates

    for i in range(num_joints):
        # Extract DH parameters for the current joint
        a, alpha, d, theta = dh_params[i]

        # Calculate transformation matrix for the current joint
        transformation_matrix = dh_transform(a, alpha, d, theta)

        # Extract the translation component from the transformation matrix
        translation_vector = transformation_matrix[:3, 3]

        # Compute the end effector coordinates by accumulating the translations
        end_effector_coords[i+1] = end_effector_coords[i] + translation_vector

    return end_effector_coords


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
        if animation is None or animation.event_source is None:
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

    

    data_display_frame = tk.Frame(root)
    #controls_frame.grid(row=1, column=0)
    data_display_frame.pack()

    controls_frame = tk.Frame(data_display_frame)
    controls_frame.grid(row=0, column=0, padx=10, pady=10)

    start_pause_button = Button(controls_frame, text="Start/Pause", command=start_pause_simulation)
    start_pause_button.pack(side=tk.LEFT, padx=10)

    restart_button = Button(controls_frame, text="Restart", command=restart_simulation)
    restart_button.pack(side=tk.LEFT, padx=10)
    
    timestep_text_box = tk.Text(controls_frame, height=4, width=30, state='disabled')
    timestep_text_box.pack(side=tk.TOP)


    # initialize kinematic data tree
    _data_display_frame = tk.Frame(data_display_frame)
    _data_display_frame.grid(row=0, column=1, padx=10, pady=10)

    data_display_label = ttk.Label(_data_display_frame, text="Kinematic Data")
    data_display_label.pack(side=tk.TOP)

    tree = ttk.Treeview(_data_display_frame)
    tree["columns"] = ("θ", "X", "Y", "Z")
    
    tree.heading("#0", text="Joint")
    tree.column("#0", width=50, anchor="center")
    for column in tree["columns"]:
        tree.heading(column, text=column)
        tree.column(column, width=80, anchor="center")
    
    tree.pack()

    def update_data_display(frame, joint_angles, end_effector_coordinates):
        dt = 1
        text = f"Frame: {frame}\n"
        text += f"Timestep: {frame * dt:.2f}s"  # Add the current timestep information

        timestep_text_box.configure(state='normal')  # Enable text box for modification
        timestep_text_box.delete('1.0', tk.END)  # Clear previous contents
        timestep_text_box.insert(tk.END, text)  # Insert new text
        timestep_text_box.configure(state='disabled')  # Disable text box for readonly

        # Update joint data table
        tree.delete(*tree.get_children())  # Clear previous contents
        joint_angles = np.degrees(joint_angles)
        for i in range(num_joints):
            tree.insert("", "end", text=str(i), values=(f'{joint_angles[i]:.2f}°',
                        str(end_effector_coordinates[i][0]), str(end_effector_coordinates[i][1]), str(end_effector_coordinates[i][2])))


    def update(frame):
        joint_angles = calculate_joint_angles(frame, behavior)

        for i in range(num_joints):
            dh_params[i][3] = joint_angles[i] # Update joint angles
        transformations = forward_kinematics(dh_params)
        end_effector_coordinates = calculate_end_effector_coordinates(dh_params)

        update_data_display(frame, joint_angles, end_effector_coordinates)

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
        ax.plot(arm_config[:, 0], arm_config[:, 1], arm_config[:, 2], marker='o')
       
        _lim = arm_length + 0.5

        # set plot limits and labels
        ax.set_xlim(-_lim, _lim)
        ax.set_ylim(-_lim, _lim)
        ax.set_zlim(-_lim, _lim)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    def plot_arm_2d(ax, arm_config, arm_length, x_label, y_label):
        # plot the 2D arm configuration on the given subplot
        ax.plot(arm_config[:, 0], arm_config[:, 1], marker='o')

        _lim = arm_length + 0.5

        # set plot limits and labels
        ax.set_xlim(-_lim, _lim)
        ax.set_ylim(-_lim, _lim)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid()

    # Create the FigureCanvasTkAgg widget
    canvas = FigureCanvasTkAgg(fig, master=plots_frame)
    canvas.draw()

    # Embed the plot in the Tkinter window
    canvas.get_tk_widget().pack()

    # Create animation
    animation = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

    # Start the main Tkinter event loop
    root.mainloop()

# @dataclass
# class sim_conf:
#     dh_parameters: list[list[float]]
#     command: str
#     behaviour: 


if __name__=='__main__':
    # Example usage
    dh_parameters = [
        [0, np.pi/2, 0.5, 0],   # Joint 1: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 2: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 3: a, alpha, d, theta
        [0, np.pi/2, 0, 0],     # Joint 4: a, alpha, d, theta
        [0, -np.pi/2, 0.5, 0]     # Joint 5: a, alpha, d, theta
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

################################################################################
## Copyright (c) 2023 Brian Costantino 
## 
## Permission is hereby granted, free of charge, to any person obtaining
## a copy of this software and associated documentation files (the
## "Software"), to deal in the Software without restriction, including
## without limitation the rights to use, copy, modify, merge, publish,
## distribute, sublicense, and/or sell copies of the Software, and to
## permit persons to whom the Software is furnished to do so, subject to
## the following conditions:
## 
## The above copyright notice and this permission notice shall be
## included in all copies or substantial portions of the Software.
## 
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
## EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
## MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
## NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
## LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
## OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
## WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
################################################################################

