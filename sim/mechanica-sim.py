##################################################
## mechanica-sim.py 
## python forward/inverse kinematics simulator
##################################################
## Author: Brian Costantino
## Version: 0.1.0
## Last Updated: 06.2023
## Status: Active
## License: MIT, see footer for more info
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
#from src.kinematics import _inverse_kinematics

def pretty_print_matrix(matrix):
    s = [[str(e) for e in row] for row in matrix]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    print('\n'.join(table))

def weight_matrix(matrix, weight_vector):
    # Ensure that the dimensions match
    assert matrix.shape[0] == weight_vector.shape[0], "Matrix and weight vector dimensions don't match"

    # Perform element-wise multiplication
    weighted_matrix = matrix * weight_vector[:, np.newaxis]

    return weighted_matrix

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
    
    ax_3d = fig.add_subplot(gs[0,1], projection='3d')

    # create the 2D plots
    ax_xy = fig.add_subplot(gs[0,0])
    ax_xz = fig.add_subplot(gs[1,0])
    ax_yz = fig.add_subplot(gs[1,1])

    ax_xy.set_box_aspect(1)  # set aspect ratio for xy subplot
    ax_xz.set_box_aspect(1)  # set aspect ratio for xz subplot
    ax_yz.set_box_aspect(1)  # set aspect ratio for yz subplot

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


    # initialize controls UI container and contents
    controls_frame = tk.Frame(data_display_frame)
    controls_frame.grid(row=0, column=0, padx=10, pady=10)

    # Create a sub-frame for the buttons
    buttons_frame = tk.Frame(controls_frame)
    buttons_frame.pack(side="top", pady=10)
    
    # Create the Start/Pause button
    start_pause_button = Button(buttons_frame, text="Start/Pause", command=start_pause_simulation)
    start_pause_button.pack(side="left", padx=10)
    
    # Create the Restart button
    restart_button = Button(buttons_frame, text="Restart", command=restart_simulation)
    restart_button.pack(side="left", padx=10)
 
    timestep_text_box = tk.Text(controls_frame, height=2, width=30, state='disabled')
    timestep_text_box.pack(padx=10, pady=(0, 10))
    
    def on_simulation_type_change(selection):
        if selection == "FK":
            # Update controls for forward kinematics simulation
            # Example: Show joint angle expression controls
            joint_angle_frame.pack()
            end_effector_frame.pack_forget()
        elif selection == "IK":
            # Update controls for inverse kinematics simulation
            # Example: Show end effector position/orientation controls
            joint_angle_frame.pack_forget()
            end_effector_frame.pack()


    # Create the dropdown selection for simulation type
    simulation_types = ["FK", "IK"]  # Add more simulation types if needed
    selected_simulation_type = tk.StringVar(controls_frame)
    selected_simulation_type.set(simulation_types[0])  # Set initial simulation type
    
    simulation_type_dropdown = tk.OptionMenu(controls_frame, selected_simulation_type, *simulation_types, command=on_simulation_type_change)
    simulation_type_dropdown.pack(side=tk.LEFT)
    
    # Create sub-frames for simulation type specific controls
    joint_angle_frame = tk.Frame(controls_frame)
    joint_angle_frame.pack()
    
    end_effector_frame = tk.Frame(controls_frame)
    
    # Add controls specific to forward kinematics simulation (FK)
    joint_angle_label = tk.Label(joint_angle_frame, text="Joint Angle Expression:")
    joint_angle_label.pack()
    
    # Add controls specific to inverse kinematics simulation (IK)
    end_effector_label = tk.Label(end_effector_frame, text="End Effector Position/Orientation:")
    end_effector_label.pack()


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
            end_effector_orientation = extract_euler_angles_zyx(end_effector_transformations[i])
            tree.insert("", "end", text=str(i), values=(f'{joint_angles[i]:.2f}°',
                        str(round(end_effector_coordinates[0], coord_num_digits)),
                        str(round(end_effector_coordinates[1], coord_num_digits)),
                        str(round(end_effector_coordinates[2], coord_num_digits))))
        
        end_effector_text = f'POS[xyz]: ({", ".join(str(round(i, 2)) for i in end_effector_coordinates)})\n'
        end_effector_text += f'ATT[ypr]: ({", ".join(str(round(math.degrees(i), 2)) + "°" for i in end_effector_orientation)})'
        update_readonly_textbox(end_effector_text_box, end_effector_text)



    def update(frame):
        # cmd = 'inverse-kinematics'
        cmd = 'inverse-kinematics'

        joint_angles = dh_params[:,3]
        if cmd == 'inverse-kinematics':
            start = np.array([-1, -1, 2])
            end = np.array([1, 1, 2])
            _range = end - start
            target_coords = start + (frame/360) * _range 
            target_pose = np.concatenate((target_coords, [math.radians(i) for i in [0, 45, 0]]))
            
            #_joint_angles = inverse_kinematics_dls(dh_params, target_pose)
            _joint_angles = inverse_kinematics_('dls', dh_params, target_pose, damping_constant=0.01)

            if _joint_angles is not None:
                joint_angles = _joint_angles
            
            # input('press to continue...')
        
        elif cmd == 'forward-kinematics':
            joint_angles = calculate_joint_angles(frame, behavior)

        dh_params[:,3] = joint_angles # Update joint angles
        transformations = forward_kinematics(dh_params) # [_T for _T, q in forward_kinematics_quat(dh_params)]
        
        update_plot(dh_params, transformations)
        update_data_display(frame, joint_angles, transformations)

        #input("press <Enter> to update plot...")


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
     
    # behavior = [
    #     (0, 90, 'clockwise'),  # Joint 1 sweeps from 0° to 90° in a clockwise direction
    #     (-45, 45, 'counterclockwise'),  # Joint 2 sweeps from -45° to 45° in a counterclockwise direction
    #     (0, 180, 'counterclockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    #     (0, 180, 'clockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    #     (0, 180, 'counterclockwise')  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    # ]
     
    behavior = [
        (-90, 90, 'clockwise'),  # Joint 1 sweeps from 0° to 90° in a clockwise direction
        (0, 0, 'counterclockwise'),  # Joint 2 sweeps from -45° to 45° in a counterclockwise direction
        (0, 0, 'counterclockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (90, 90, 'clockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (0, 0, 'counterclockwise')  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    ]


    animation_running = True
    animation = None
    
    print('[INFO]: Initial DH parameters:')
    pretty_print_matrix(dh_parameters)
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

