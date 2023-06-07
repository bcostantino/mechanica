##################################################
## src/kinematics.py 
## mathematics functions for kinematics simulations
##################################################
## Author: Brian Costantino
## Version: 0.1.0
## Last Updated: 06.2023
## Status: Active
## License: MIT, see footer for more info
##################################################


import numpy as np
from scipy.spatial.transform import Rotation as R
import copy
import math
import quaternion


def is_singular(rotation_matrix):
    """Determine if rotation matrix is singular (lost rank) based on determinant"""
    determinant = np.linalg.det(rotation_matrix)
    epsilon = 1e-6  # Define a small threshold to consider matrix as singular
    return abs(determinant) < epsilon

def normalize_quaternion(q):
    """Normalize quaternion to unit quaternion"""
    magnitude = np.sqrt(np.sum(np.square(q)))   # np.linalg.norm(q)
    normalized_q = q / magnitude
    return normalized_q

def dh_transform(a, alpha, d, theta):
    """Compute the Denavit-Hartenberg (DH) transformation matrix"""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    T = np.array([[cos_theta,   -sin_theta * cos_alpha, sin_theta * sin_alpha,  a * cos_theta   ],
                  [sin_theta,   cos_theta * cos_alpha,  -cos_theta * sin_alpha, a * sin_theta   ],
                  [0,           sin_alpha,              cos_alpha,              d               ],
                  [0,           0,                      0,                      1               ]])
    
    if is_singular(R := T[:3,:3]):
        print(f"[WARN]: detected singular matrix in dh_transform, det(R) = {np.linalg.det(R)}")

    return T

def dh_transform_quaternion(a, alpha, d, theta):
    cos_theta_half = np.cos(theta / 2)
    sin_theta_half = np.sin(theta / 2)
    cos_alpha_half = np.cos(alpha / 2)
    sin_alpha_half = np.sin(alpha / 2)

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # create quaternion
    # q = quaternion.from_rotation_vector([alpha, 0, theta])
    # q = quaternion.from_euler_angles([alpha, theta, 0])
    #q = quaternion.from_euler_angles([0, alpha, theta])

    # Create the quaternion representation
    q = quaternion.quaternion(
        cos_theta_half * cos_alpha_half,
        sin_theta_half * sin_alpha_half,
        sin_theta_half * cos_alpha_half,
        cos_theta_half * sin_alpha_half
    )

    q = normalize_quaternion(q)

    # construct the transformation matrix from DH parameters and quaternion
    T = np.eye(4)
    T[:3, :3] = quaternion.as_rotation_matrix(q.conjugate()) # add rotation matrix
    T[:3,3] = [a * cos_theta, a * sin_theta, d] # add translation vector

    if is_singular(R := T[:3,:3]):
        print(f"[WARN]: detected singular matrix in dh_transform_quaternion, det(R) = {np.linalg.det(R)}")
    
    return T, q

def forward_kinematics(dh_params):
    """Perform forward kinematics based on the DH parameters"""
    T = np.eye(4)
    transformations = []
    for i, params in enumerate(dh_params):
        T = np.dot(T, dh_transform(*params))
        transformations.append(T)


        if is_singular(R := T[:3,:3]):
            print(f"[WARN]: detected singular matrix in forward_kinematics, joint #{i + 1}, det(R) = {np.linalg.det(R)}")


    return transformations

def forward_kinematics_quat(dh_params):
    """Perform forward kinematics based on the DH parameters (using quaternions)"""
    T = np.eye(4)
    q = quaternion.quaternion(1, 0, 0, 0)
    transformations = []
    for params in dh_params:
        _T, _q = dh_transform_quaternion(*params)
        T = T @ _T # construct_transformation_matrix(q, a, d)
        q = q * _q

        transformations.append((T, q))
    return transformations


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

# THIS IS NOT CURRENTLY WORKING
def extract_euler_angles(rotation_matrix: np.ndarray) -> tuple[float, float, float]:
    # Calculate the sin of the pitch angle
    sy = math.sqrt(rotation_matrix[0][0]**2 + rotation_matrix[1][0]**2)

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
    return (roll, pitch, yaw)


def extract_euler_angles_zyx(rotation_matrix: np.ndarray):
    r = R.from_matrix(rotation_matrix[:3,:3])
    yaw, pitch, roll = r.as_euler('ZYX', degrees=False)
    return np.array((yaw, pitch, roll))

# THIS IS NOT CURRENTLY WORKING
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


def calculate_jacobian_fin_diff(dh_parameters: np.ndarray, h: float = 1e-6) -> np.ndarray:
    """Calculate end-effector Jacobian matrix numerically using finite-differences"""

    N = len(dh_parameters)
    jacobian = np.zeros((6, N))

    for i in range(N):
        _dh = copy.deepcopy(dh_parameters)  # dh_parameters.copy()
        
        # find initial transformation matrix for end effector
        ee_trans = forward_kinematics(_dh)[-1]
        ee_position = ee_trans[:3,3]
        ee_orientation = np.array(extract_euler_angles_zyx(ee_trans[:3,:3]))
        
        _dh[i][3] += h  # perturb parameter
        
        # find final transformation matrix
        ee_f_trans = forward_kinematics(_dh)[-1]
        ee_f_position = ee_f_trans[:3,3]   # find final transformation matrix
        ee_f_orientation = extract_euler_angles_zyx(ee_f_trans[:3,:3])

        # calculate position and orientation partial derivatives
        dp_dt = np.subtract(ee_f_position, ee_position) / h
        da_dt = np.subtract(ee_f_orientation, ee_orientation) / h

        jacobian[:3,i] = dp_dt
        jacobian[3:6,i] = da_dt

    return jacobian


def update_joint_angles(theta, delta_theta, joint_limits):
    """
    Updates the joint angles 'theta' with the desired change 'delta_theta' while considering the joint limits.
    
    Args:
        theta (numpy.ndarray): Current joint angles.
        delta_theta (numpy.ndarray): Desired change in joint angles.
        joint_limits (list): Joint limits represented as a list of tuples (lower_bound, upper_bound).
        
    Returns:
        numpy.ndarray: Updated joint angles within the specified limits.
    """
    new_theta = theta + delta_theta
    for i in range(len(theta)):
        lower_bound, upper_bound = joint_limits[i]
        
        # Check if the desired change exceeds the joint limits
        if delta_theta[i] > 0 and new_theta[i] > upper_bound:
            # Reverse the desired change and adjust the joint angle
            new_theta[i] = upper_bound - (new_theta[i] - upper_bound)
        elif delta_theta[i] < 0 and new_theta[i] < lower_bound:
            # Reverse the desired change and adjust the joint angle
            new_theta[i] = lower_bound - (new_theta[i] - lower_bound)
        
        # Ensure the updated joint angle stays within the limits
        if new_theta[i] < lower_bound:
            new_theta[i] = lower_bound
        elif new_theta[i] > upper_bound:
            new_theta[i] = upper_bound
    
    return new_theta

def calculate_max_reach(dh_parameters):
    """
    Calculate the maximum reach of the arm in the direction of the target position.
    
    Args:
        dh_parameters (numpy.ndarray): DH parameters representing the robotic arm.
        
    Returns:
        float: Maximum reach of the arm.
    """
    # Get the lengths (a values) from the DH parameters
    lengths = dh_parameters[:, 0]
    
    # Calculate the sum of the lengths
    max_reach = np.sum(lengths)
    
    return max_reach


def scale_target_position(target_position, max_reach):
    """
    Scale down the target position if it's outside the maximum reach of the arm.
    
    Args:
        target_position (numpy.ndarray): Desired target position (x, y, z) coordinates.
        max_reach (float): Maximum reach of the arm.
        
    Returns:
        numpy.ndarray: Scaled target position lying on the sphere of maximum reach.
    """
    distance = np.linalg.norm(target_position)
    
    if distance > max_reach:
        scale_factor = max_reach / distance
        scaled_position = target_position * scale_factor
        return scaled_position
    else:
        return target_position



def inverse_kinematics_(method: str, dh_parameters: np.ndarray, target_pose: np.ndarray, **kwargs):
    """Compute optimal joint angles to achieve target end-effector pose"""

    # initialize arguments
    theta = kwargs.get('theta_init', dh_parameters[:,3])
    pose_weights = kwargs.get('pose_weights', np.array([1,1,1,0,0,0])) 
    theta_limits: list[tuple[float]] = kwargs.get('theta_limits', None)

    # set the convergence threshold, step size and maximum iterations
    threshold = 1e-6
    max_iterations = 100

    # snitialize iteration counter and error
    iterations = 0
    error = np.inf

    while error > threshold and iterations < max_iterations:
        
        # perform forward kinematics with the current joint angles
        transformations = forward_kinematics(dh_parameters)
        current_position = transformations[-1][:3,3]
        current_orientation = extract_euler_angles_zyx(transformations[-1][:3,:3])
        current_pose = np.concatenate((current_position, current_orientation))

        # calculate error
        error_vector = target_pose - current_pose
        error_vector = np.multiply(pose_weights, error_vector)
        error =  np.linalg.norm(error_vector)
        if error <= threshold:
            break

        jacobian = calculate_jacobian_fin_diff(dh_parameters)
        
        # mask using pose_weights
        j = jacobian * pose_weights.reshape(-1,1)


        # jacobian pseudo-inverse method
        if method.lower() == 'jpi':
            delta_theta = np.linalg.pinv(j) @ (error_vector)
        
        # damped least squares method
        elif method.lower() == 'dls':
            damping_constant = kwargs['damping_constant'] or 0.01
            delta_theta = j.T @ np.linalg.inv(j @ j.T + damping_constant**2 * np.eye(len(j))) @ (error_vector)

        else:
            raise Exception(f"invalid method '{method}'")

        # enforce joint constraints
        if theta_limits is None:
            theta += delta_theta
        else:
            theta = update_joint_angles(theta, delta_theta, theta_limits)
            #print('updating with hard limits: ', ', '.join(str(round(i, 2)) for i in theta))

        dh_parameters[:,3] = theta
        iterations += 1

    if iterations == max_iterations:
        print("[WARN]: Inverse kinematics did not converge... last error: ", error_vector, error)
    # else:
        # print(f'[INFO]: Converged after {iterations} iterations; last error: ', error_vector, error)
        # print(f'[INFO]: Achieved final end effector pose: ({", ".join(str(round(i, 4)) for i in current_pose)})',
        #       f'theta: ({", ".join(str(round(math.degrees(i), 4)) + "°" for i in theta)})')

    return theta




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

