##################################################
## mechanica-sim.py 
## python forward/inverse kinematics simulator
##################################################
## Author: Brian Costantino
## License: MIT, see footer for more info
## Version: 0.1.0
## Status: Active
##################################################

import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

import tkinter as tk
from tkinter import ttk, Button

from src.kinematics import *

def pretty_print_matrix(matrix):
    s = [[str(e) for e in row] for row in matrix]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    print('\n'.join(table))

def extract_euler_angles(rotation_matrix: list[list[float]]) -> list[float]:
    # Calculate the sin of the pitch angle
    sy = math.sqrt(rotation_matrix[0][0] * rotation_matrix[0][0] + rotation_matrix[1][0] * rotation_matrix[1][0])

    if sy > 1e-6:
        # Case when sy is not close to zero
        # Calculate roll, pitch, and yaw using atan2
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
        pitch = math.atan2(-rotation_matrix[2][0], sy)
        yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
    else:
        # Case when sy is close to zero (singularity)
        # Set yaw to 0 and calculate roll and pitch
        roll = math.atan2(-rotation_matrix[1][2], rotation_matrix[1][1])
        pitch = math.atan2(-rotation_matrix[2][0], sy)
        yaw = 0.0

    # Return Euler angles as [pitch, roll, yaw]
    return [roll, pitch, yaw]



def inverse_kinematics(dh_parameters, end_effector):
    print('starting inverse kinemetics for end effector position: ', end_effector[:3])

    # Extract the number of joints
    num_joints = len(dh_parameters)

    # Initialize the joint angles
    theta = dh_parameters[:,3] # np.zeros(num_joints)

    # Set the convergence threshold and maximum iterations
    threshold = 1e-6
    max_iterations = 100

    # Initialize the iteration counter and the error
    iterations = 0
    error = np.inf

    while error > threshold and iterations < max_iterations:
        # Perform forward kinematics with the current joint angles
        transformations = forward_kinematics(dh_parameters)
        
        end_effector_translation_vector = transformations[-1][:3, 3]
        # end_effector_rotation_matrix = transformations[-1][:3,:3]
        # end_effector_orientation = extract_euler_angles(end_effector_rotation_matrix)

        # Calculate the difference in Euler angles
        # euler_diff = np.array(end_effector[3:6]) - np.array(end_effector_orientation)

        # calculate the difference in translation vectors
        translation_diff = np.array(end_effector[:3]) - np.array(end_effector_translation_vector)

        # calculate the overall error as the Euclidean distance between the differences
        error = np.linalg.norm(translation_diff)
        if error <= threshold:
            break

        # Calculate the Jacobian matrix
        jacobian = calculate_jacobian_fin_diff(dh_parameters) # calculate_jacobian(transformations)

        # Update the joint angles using the pseudoinverse of the Jacobian
        delta_theta = np.linalg.pinv(jacobian) @ (translation_diff)
        theta += delta_theta

        # print(f'interation #{iterations}:')
        # print('desired end effector position:', end_effector[0:3])  # current_end_effector)
        # #print('desired end effector orientation: ', end_effector[3:6])
        # print('working theta: ', dh_parameters[:num_joints][3])
        # print(f'end effector position: ({", ".join(str(round(i, 2)) for i in end_effector_translation_vector)})')
        # #print('end effector position: ', end_effector_orientation)
        # print('error: ', error)
        # print('new theta: ', theta)
        # print('\n')

        # Update the DH parameters
        for i in range(num_joints):
            dh_parameters[i][3] = theta[i] # dh_parameters[:num_joints][3] = theta
        iterations += 1

    if iterations == max_iterations:
        print("[WARN]: Inverse kinematics did not converge.")
    else:
        print(f'[INFO]: Converged after {iterations} iterations; last error: ', error)
        print(f'[INFO]: Achieved final end effector position: ({", ".join(str(round(i, 4)) for i in end_effector_translation_vector)})',
              f'theta: ({", ".join(str(round(math.degrees(i), 4)) + "°" for i in theta)})')

    return theta

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
    root.geometry("1200x1000")
 
    # Create a frame to hold the plots
    plots_frame = ttk.Frame(root, width=600, height=500)
    plots_frame.pack(side=tk.TOP, pady=10)

    # Create the 3D plot
    fig =  plt.figure(figsize=(8, 6))
    
    # Create a gridspec layout with 2 rows and 2 columns
    gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])
    
    ax_3d = fig.add_subplot(gs[0,0], projection='3d')

    # Create the 2D plots
    ax_xy = fig.add_subplot(gs[0,1])
    ax_yz = fig.add_subplot(gs[1,0])
    ax_xz = fig.add_subplot(gs[1,1])

    ax_xy.set_box_aspect(1)  # Set aspect ratio for xy subplot
    ax_xz.set_box_aspect(1)  # Set aspect ratio for xz subplot
    ax_yz.set_box_aspect(1)  # Set aspect ratio for yz subplot

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
            animation.frame_seq = animation.new_frame_seq() # reset animation frame sequence

    data_display_frame = tk.Frame(root)
    data_display_frame.pack()

    controls_frame = tk.Frame(data_display_frame)
    controls_frame.grid(row=0, column=0, padx=10, pady=10)

    start_pause_button = Button(controls_frame, text="Start/Pause", command=start_pause_simulation)
    start_pause_button.pack(padx=10)

    restart_button = Button(controls_frame, text="Restart", command=restart_simulation)
    restart_button.pack(padx=10)
    
    timestep_text_box = tk.Text(controls_frame, height=2, width=30, state='disabled')
    timestep_text_box.pack()

    # initialize kinematic data tree
    _data_display_frame = tk.Frame(data_display_frame)
    _data_display_frame.grid(row=0, column=1, padx=10, pady=10)


    ttk.Label(_data_display_frame, text="End Effector").pack(side=tk.TOP)
    end_effector_text_box = tk.Text(_data_display_frame, height=2, width=50, state='disabled')
    end_effector_text_box.pack()

    ttk.Label(_data_display_frame, text="Kinematic Data").pack(side=tk.TOP)
    tree = ttk.Treeview(_data_display_frame, height=num_joints)
    tree["columns"] = ("θ", "X", "Y", "Z")
    
    tree.heading("#0", text="Joint")
    tree.column("#0", width=50, anchor="center")
    for column in tree["columns"]:
        tree.heading(column, text=column)
        tree.column(column, width=80, anchor="center")
    
    tree.pack()

    def update_readonly_textbox(textbox: tk.Text, text: str):
        textbox.configure(state='normal')  # Enable text box for modification
        textbox.delete('1.0', tk.END)  # Clear previous contents
        textbox.insert(tk.END, text)  # Insert new text
        textbox.configure(state='disabled')  # Disable text box for readonly

    def update_data_display(frame, joint_angles, end_effector_transformations):
        dt = 1
        timestep_text = f"Frame: {frame}\n"
        timestep_text += f"Timestep: {frame * dt:.2f}s\n"  # Add the current timestep information
        update_readonly_textbox(timestep_text_box, timestep_text)

        # Update joint data table
        coord_num_digits = 4
        tree.delete(*tree.get_children())  # Clear previous contents
        joint_angles = np.degrees(joint_angles)
        for i in range(num_joints):
            end_effector_coordinates = end_effector_transformations[i][:3,3]
            end_effector_orientation = extract_euler_angles(end_effector_transformations[i])
            tree.insert("", "end", text=str(i), values=(f'{joint_angles[i]:.2f}°',
                        str(round(end_effector_coordinates[0], coord_num_digits)),
                        str(round(end_effector_coordinates[1], coord_num_digits)),
                        str(round(end_effector_coordinates[2], coord_num_digits))))
        
        end_effector_text = f'POS[xyz]: ({", ".join(str(round(i, 2)) for i in end_effector_coordinates)})\n'
        end_effector_text += f'ATT[rpy]: ({", ".join(str(round(math.degrees(i), 2)) + "°" for i in end_effector_orientation)})'
        update_readonly_textbox(end_effector_text_box, end_effector_text)

    def update_old(frame):
        print(f'[DBG]: Figure animation frame #{frame}')
        cmd = 'inverse-kinematics' 
        
        joint_angles = dh_params[:,3]
        if cmd == 'inverse-kinematics':
            joint_angles = inverse_kinematics(dh_params, np.array([0.5,1.2,1,0,0,0]))

            if joint_angles is not None:
                print('got joint angles: ', joint_angles)
            
            input('press to continue...')

        elif cmd == 'forward-kinematics':
            joint_angles = calculate_joint_angles(frame, behavior)

        #for i in range(num_joints):
        dh_params[:,3] = joint_angles # Update joint angles
        transformations = forward_kinematics(dh_params)
        
        end_effector_coordinates = calculate_end_effector_coordinates(dh_params)
        update_data_display(frame, joint_angles, transformations)

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
    
    def update(frame):
        # print(f'[DBG]: Figure animation frame #{frame}')
        cmd = 'inverse-kinematics' # 'forward-kinematics' 
        
        joint_angles = dh_params[:,3]
        if cmd == 'inverse-kinematics':
            start = np.array([-1, -1, -1])
            end = np.array([1, 1, 1])
            _range = end - start
            target_coords = np.concatenate((start + (frame/360) * _range, [0,0,0]))# np.array([x,1.2,1,0,0,0])
            _joint_angles = inverse_kinematics(dh_params, target_coords)

            if _joint_angles is not None:
                joint_angles = _joint_angles
            
            # input('press to continue...')
        
        elif cmd == 'forward-kinematics':
            joint_angles = calculate_joint_angles(frame, behavior)

        dh_params[:,3] = joint_angles # Update joint angles
        transformations = forward_kinematics(dh_params)
        
        update_plot(dh_params, transformations)
        update_data_display(frame, joint_angles, transformations)


    def update_plot(dh_params, _transformations = None):
        transformations = forward_kinematics(dh_params) if _transformations is None else _transformations
        num_joints = len(transformations)

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
        ax.plot(arm_config[:, 0], arm_config[:, 1], arm_config[:, 2], marker='o', color='0.4')
       
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
        ax.plot(arm_config[:, 0], arm_config[:, 1], marker='o', color='0.4')

        _lim = arm_length + 0.5

        # set plot limits and labels
        ax.set_xlim(-_lim, _lim)
        ax.set_ylim(-_lim, _lim)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid()

    def mouse_move(event):
        if animation_running:
            return
        
        x, y = event.xdata, event.ydata
        if event.inaxes is None or x is None or y is None:
            return

        print('mouse move event triggered: ', event)


    # Create the FigureCanvasTkAgg widget
    canvas = FigureCanvasTkAgg(fig, master=plots_frame)
    canvas.draw()
    
    # register mouse hover event for inverse kinematics
    canvas.mpl_connect('motion_notify_event', mouse_move)

    # Embed the plot in the Tkinter window
    canvas.get_tk_widget().pack()

    # Create animation
    animation = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

    # Start the main Tkinter event loop
    root.mainloop()


if __name__=='__main__':

    # Example usage
    dh_parameters = np.array([
        [0, np.pi/2, 0.5, 0],   # Joint 1: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 2: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 3: a, alpha, d, theta
        [0, np.pi/2, 0, 0],     # Joint 4: a, alpha, d, theta
        [0, -np.pi/2, 0.5, 0]     # Joint 5: a, alpha, d, theta
    ])
     
    behavior = [
        (0, 90, 'clockwise'),  # Joint 1 sweeps from 0° to 90° in a clockwise direction
        (-45, 45, 'counterclockwise'),  # Joint 2 sweeps from -45° to 45° in a counterclockwise direction
        (0, 180, 'counterclockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (0, 180, 'clockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (0, 180, 'counterclockwise')  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    ]
     
    animation_running = True
    animation = None
    
    print('[INFO]: Initial DH parameters:')
    pretty_print_matrix(dh_parameters)
    animate_arm(dh_parameters, behavior)
    
    # import numpy as np
    # from scipy.spatial.transform import Rotation as R
    # 
    # # DH parameters
    # dh_parameters = [
    #     [0, np.pi/2, 0.5, 0],   # Joint 1: a, alpha, d, theta
    #     [1, 0, 0, 0],           # Joint 2: a, alpha, d, theta
    #     [1, 0, 0, 0],           # Joint 3: a, alpha, d, theta
    #     [0, np.pi/2, 0, 0],     # Joint 4: a, alpha, d, theta
    #     [0, -np.pi/2, 0.5, 0]   # Joint 5: a, alpha, d, theta
    # ]
    # 
    # # Joint angles in radians
    # joint_angles = np.radians([15, 30, 150, 30, 150])
    # 
    # # Transformation matrices for each joint
    # transform_matrices = []
    # for i in range(len(dh_parameters)):
    #     a, alpha, d, theta = dh_parameters[i]
    #     transform_matrix = dh_transform(a, alpha, d, theta + joint_angles[i])
    #     transform_matrices.append(transform_matrix)
    # 
    # # End effector transformation matrix
    # end_effector_matrix = np.eye(4)
    # for transform_matrix in transform_matrices:
    #     end_effector_matrix = np.dot(end_effector_matrix, transform_matrix)
    # 
    # # Extract position from the end effector transformation matrix
    # end_effector_position = end_effector_matrix[:3, 3]
    # 
    # # Extract orientation (roll, pitch, yaw) from the end effector transformation matrix
    # rotation = R.from_matrix(end_effector_matrix[:3, :3])
    # end_effector_orientation = rotation.as_euler('xyz', degrees=True)
    # 
    # print("Calculated End Effector Position (x, y, z):", tuple(end_effector_position))
    # print("Calculated End Effector Attitude (p, r, y) in degrees:", tuple(end_effector_orientation))

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

