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
from typing import Any, NamedTuple, Callable, Union
from recordclass import dataobject

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

import tkinter as tk
from tkinter import ttk, Button

from src.kinematics import *
from src.timer import RepeatingTimer



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


def start_pause_simulation():
    global animation
    if not animation: # or animation.event_source is None:
        return
       
    if animation.is_running:
        animation.stop()    # animation.event_source.stop()  # Pause the animation
    else:
        animation.start() # animation.event_source.start()  # Resume the animation
    return animation.is_running

def restart_simulation():
    global animation
    if animation:
        animation.reset() # animation.frame_seq = animation.new_frame_seq() # reset animation frame sequence


def on_simulation_type_change(_type: str) -> None:
    global animation, sim_config

    if sim_config.s_type == _type:
        return

    print(f"[INFO]: simulation type changed to {_type}")
    
    sim_config.s_type = _type
    animation.reset()



def update_readonly_textbox(textbox: tk.Text, text: str):
    textbox.configure(state='normal')  # Enable text box for modification
    textbox.delete('1.0', tk.END)  # Clear previous contents
    textbox.insert(tk.END, text)  # Insert new text
    textbox.configure(state='disabled')  # Disable text box for readonly

def en_children(parent, enabled = True):
    for child in parent.winfo_children():
        wtype = child.winfo_class()
        #print (wtype)
        if wtype not in ('Frame','Labelframe','TFrame','TLabelframe'):
            child.configure(state='normal' if enabled else 'disable')
        else:
            en_children(child, enabled)


class AppWindow(NamedTuple):
    
    # general tkinter
    root: tk.Tk
    fig: plt.FigureBase

    set_sim_data_display:           Callable[[int, int, float], None]
    render_kinematic_data_display:  Callable[[bool, int], bool]
    set_kinematic_data_display:     Callable
    set_plot_data:                  Callable[[np.ndarray, list], None]



class SimConfig(dataobject):

    s_type:     str
    num_frames: int
    kwargs:     Union[dict[str, Any], None]

    dh_params:  np.ndarray
    behavior:   list[tuple]



def create_window():

    # Create the main application window
    root = tk.Tk()
    root.title("Mechanica Simulator")
    root.geometry("1200x1000")

    # create a frame to hold the plots
    plots_frame = ttk.Frame(root, width=600, height=500)
    plots_frame.pack(side=tk.TOP, pady=10)

    # create container for data
    data_display_frame = tk.Frame(root)
    data_display_frame.pack()


    
    ######################################################
    # Controls
    #######################################################

    # initialize controls UI container and contents
    controls_frame = tk.Frame(data_display_frame)
    controls_frame.grid(row=0, column=0, padx=10, pady=10)

    # Create a sub-frame for the buttons
    buttons_frame = tk.Frame(controls_frame)
    buttons_frame.pack(side="top", pady=10)

    def _start_pause_sim():
        # start_pause_simulation()
        start_pause_button.config(text="Pause" if (anim_running := start_pause_simulation()) else "Start")

        # enable/disable sim config
        en_children(sim_config_frame, not anim_running) 

    def _on_sim_type_change(selection):
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
        else:
            return

        on_simulation_type_change(selection)

    # Create the Start/Pause button
    start_pause_button = Button(buttons_frame, text="Start/Pause", command=_start_pause_sim)
    start_pause_button.pack(side="left", padx=10)
    
    # Create the Restart button
    restart_button = Button(buttons_frame, text="Restart", command=restart_simulation)
    restart_button.pack(side="left", padx=10)
 
    timestep_text_box = tk.Text(controls_frame, height=2, width=30, state='disabled')
    timestep_text_box.pack(padx=10, pady=(0, 10))
   
    sim_config_frame = tk.Frame(controls_frame)
    sim_config_frame.pack()

    # Create the dropdown selection for simulation type
    simulation_types = ["FK", "IK"]  # Add more simulation types if needed
    selected_simulation_type = tk.StringVar(sim_config_frame) #controls_frame)
    selected_simulation_type.set(simulation_types[0])  # Set initial simulation type
    
    simulation_type_dropdown = tk.OptionMenu(sim_config_frame, selected_simulation_type, 
                                             *simulation_types, command=_on_sim_type_change)
    simulation_type_dropdown.pack(side=tk.LEFT)
    
    # Create sub-frames for simulation type specific controls
    joint_angle_frame = tk.Frame(sim_config_frame)
    joint_angle_frame.pack()
    
    end_effector_frame = tk.Frame(sim_config_frame)
    
    # Add controls specific to forward kinematics simulation (FK)
    joint_angle_label = tk.Label(joint_angle_frame, text="Joint Angle Expression:")
    joint_angle_label.pack()
    
    # Add controls specific to inverse kinematics simulation (IK)
    end_effector_label = tk.Label(end_effector_frame, text="End Effector Position/Orientation:")
    end_effector_label.pack()
    
    def set_sim_data_display(frame, frame_count, frame_interval_ms): 
        dt = frame_interval_ms
        timestep_text = f"Frame: {frame} / {frame_count}\n"
        timestep_text += f"Timestep: {frame * dt:.4f}s\n"  # Add the current timestep information
        update_readonly_textbox(timestep_text_box, timestep_text)



    ######################################################
    # Kinematic Data
    #######################################################
    
    # initialize kinematic data container
    _data_display_frame = tk.Frame(data_display_frame)
    _data_display_frame.grid(row=0, column=1, padx=10, pady=10)

    # create end effector display
    ttk.Label(_data_display_frame, text="End Effector").pack(side=tk.TOP)
    end_effector_text_box = tk.Text(_data_display_frame, height=2, width=50, state='disabled')
    tree = ttk.Treeview()

    def render_kinematic_data_display(render: bool, num_joints: int):
        global is_kinematic_data_rendered
        nonlocal tree

        # break if widget already in target state
        if is_kinematic_data_rendered == render:
            return is_kinematic_data_rendered
        
        # hide kinematic data
        if is_kinematic_data_rendered:
            _list = _data_display_frame.grid_slaves()
            for l in _list:
                l.destroy()

        # show kinematic data
        else:
            end_effector_text_box.pack()

            # create joint table
            ttk.Label(_data_display_frame, text="Kinematic Data").pack(side=tk.TOP)
            tree = ttk.Treeview(_data_display_frame, height=num_joints)
            tree["columns"] = ("θ", "X", "Y", "Z")

            tree.heading("#0", text="Joint")
            tree.column("#0", width=50, anchor="center")
            for column in tree["columns"]:
                tree.heading(column, text=column)
                tree.column(column, width=80, anchor="center")
            
            tree.pack()

        
        is_kinematic_data_rendered = not is_kinematic_data_rendered
        return is_kinematic_data_rendered


    def set_kinematic_data_display(joint_angles, transformations):

        # Update joint data table
        coord_num_digits = 4
        tree.delete(*tree.get_children())  # Clear previous contents
        joint_angles = np.degrees(joint_angles)
        for i in range(num_joints):
            end_effector_coordinates = transformations[i][:3,3]
            end_effector_orientation = extract_euler_angles_zyx(transformations[i])
            tree.insert("", "end", text=str(i), values=(f'{joint_angles[i]:.2f}°',
                        str(round(end_effector_coordinates[0], coord_num_digits)),
                        str(round(end_effector_coordinates[1], coord_num_digits)),
                        str(round(end_effector_coordinates[2], coord_num_digits))))
        
        end_effector_text = f'POS[xyz]: ({", ".join(str(round(i, 2)) for i in end_effector_coordinates)})\n'
        end_effector_text += f'ATT[ypr]: ({", ".join(str(round(math.degrees(i), 2)) + "°" for i in end_effector_orientation)})'
        update_readonly_textbox(end_effector_text_box, end_effector_text)

    
    def plot_arm_3d(ax, arm_config, arm_length):

        ax.clear()

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
        
        ax.clear()

        # plot the 2D arm configuration on the given subplot
        ax.plot(arm_config[:, 0], arm_config[:, 1], marker='o', color='0.4')

        _lim = arm_length + 0.5

        # set plot limits and labels
        ax.set_xlim(-_lim, _lim)
        ax.set_ylim(-_lim, _lim)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid()

    def set_plot_data(dh_params, transformations):
        #transformations = forward_kinematics(dh_params) if _transformations is None else _transformations
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
    
        plt.draw()



    # create the plot figure 
    fig =  plt.figure(figsize=(8, 6))
    
    # create a gridspec layout with 2 rows and 2 columns
    gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])
    
    # create the plot objects
    ax_3d = fig.add_subplot(gs[0,1], projection='3d')
    ax_xy = fig.add_subplot(gs[0,0])
    ax_xz = fig.add_subplot(gs[1,0])
    ax_yz = fig.add_subplot(gs[1,1])

    # set aspect ratio for 2d plots
    ax_xy.set_box_aspect(1)
    ax_xz.set_box_aspect(1)
    ax_yz.set_box_aspect(1)

    # Create the FigureCanvasTkAgg widget
    canvas = FigureCanvasTkAgg(fig, master=plots_frame)
    canvas.draw()
    
    # register mouse hover event for inverse kinematics
    # canvas.mpl_connect('motion_notify_event', mouse_move)

    # Embed the plot in the Tkinter window
    canvas.get_tk_widget().pack()

    # define window handle
    _w = AppWindow(root=root, fig=fig, \
            set_sim_data_display=set_sim_data_display, \
            render_kinematic_data_display=render_kinematic_data_display, \
            set_kinematic_data_display=set_kinematic_data_display, \
            set_plot_data=set_plot_data)

    return _w # root, tree, render_kinematic_data_display, set_kinematic_data_display


def get_next_joint_angles(frame: int, sim_config: SimConfig) -> np.ndarray:
    dh_params = sim_config.dh_params 
    behavior = sim_config.behavior

    if sim_config.s_type == 'IK':
        start = np.array([-1, -1, 2])
        end = np.array([1, 1, 2])
        _range = end - start
        target_coords = start + (frame/360) * _range 
        target_pose = np.concatenate((target_coords, [math.radians(i) for i in [0, 45, 0]]))
            
        _joint_angles = inverse_kinematics_('dls', dh_params, target_pose, 
                                            damping_constant=0.01, **(sim_config.kwargs if sim_config.kwargs is not None else {}))

        if _joint_angles is not None:
            joint_angles = _joint_angles
            
    elif sim_config.s_type == 'FK':
        joint_angles = calculate_joint_angles(frame, behavior)
    
    else:
        raise Exception(f"invalid simulation type: '{sim_config.s_type}'")

    return joint_angles


def start_simulation(app_window: AppWindow, sim_config: SimConfig):
    global animation

    if getattr(animation, 'is_running', False):
        print('[WARN]: sim already running, skipping...')
        return
    
    # parse arguments
    sim_interval = sim_config.kwargs['sim_interval'] or 10     # in ms

    def update(frame):
        #global sim_config, app_window
        dh_params = sim_config.dh_params 
         
        #joint_angles = dh_params[:,3]
        dh_params[:,3] = get_next_joint_angles(frame, sim_config) # joint_angles # Update joint angles
        transformations = forward_kinematics(dh_params)
        
        app_window.set_plot_data(dh_params, transformations)
        app_window.set_sim_data_display(frame, sim_config.num_frames, sim_interval / 1000)
        app_window.set_kinematic_data_display(dh_params[:,3], transformations)

    print(f'[INFO]: starting {sim_config.s_type} sim. frame_count: {sim_config.num_frames}, frame_interval: {sim_inverval} ms')
    animation = RepeatingTimer(sim_inverval, update)



is_kinematic_data_rendered: bool = False
animation: RepeatingTimer = None
app_window: AppWindow
sim_config: SimConfig

if __name__=='__main__':

    dh_parameters = np.array([
    #    a,     alpha,      d,      theta
        [0,     np.pi/2,    0.5,    0],
        [1,     0,          0,      0],
        [1,     0,          0,      0],
        [0,     np.pi/2,    0,      0],
        [0,    -np.pi/2,    0.5,    0]
    ])

    num_joints = len(dh_parameters)
    num_frames = 360
    sim_inverval = 1 # ms
    sim_length = num_frames * sim_inverval / 1000 # s

    behavior = [
        (-90, 90, 'clockwise'),  # Joint 1 sweeps from 0° to 90° in a clockwise direction
        (0, 0, 'clockwise'),  # Joint 2 sweeps from 0° to 90° in a clockwise direction
        (0, 0, 'counterclockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (90, 90, 'clockwise'),  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
        (0, 0, 'counterclockwise')  # Joint 3 sweeps from 0° to 180° in a counterclockwise direction
    ]

    joint_limits = [
        (-180, 180),
        (0, 180),
        (-90, 90),
        (-90, 90),
        (-180, 180)
    ]

    kwargs = {
        'num_frames': num_frames,
        'sim_interval': sim_inverval,
        'theta_limits': [(np.radians(lower), np.radians(upper)) for lower, upper in joint_limits]
    }

    sim_config = SimConfig(s_type='FK', \
            kwargs=kwargs, dh_params=dh_parameters, \
            behavior=behavior, num_frames=num_frames) 

    
    print('[INFO]: Initial DH parameters:')
    pretty_print_matrix(dh_parameters)
    print('\n')

    app_window = create_window()    
    app_window.render_kinematic_data_display(True, num_joints) 

    # Create animation
    #animation = FuncAnimation(app_window.fig, update, frames=range(num_frames), 
    #                          interval=50, blit=False)
    #animation.event_source.stop() # stop animation immediately
    # animation = RepeatingTimer(sim_inverval, update)
    # animation.start()

    start_simulation(app_window, sim_config)

    app_window.root.mainloop() 




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

