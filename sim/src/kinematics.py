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

def inverse_kinematics_jpi(dh_parameters, end_effector):
    """Compute optimal joint angles for DH model and desired end-effector positon using JPI"""

    # Initialize the joint angles
    theta = dh_parameters[:,3] # np.zeros(num_joints)

    # Set the convergence threshold, step size and maximum iterations
    threshold = 1e-6
    max_iterations = 100

    # Initialize the iteration counter and the error
    iterations = 0
    error = np.inf

    while error > threshold and iterations < max_iterations:
        
        # perform forward kinematics with the current joint angles
        transformations = forward_kinematics(dh_parameters)        
        end_effector_translation_vector = transformations[-1][:3,3]

        # calculate the difference in translation vectors
        translation_diff = end_effector - end_effector_translation_vector

        # calculate the overall error as the Euclidean distance between the differences
        error =  np.linalg.norm(translation_diff) 
        if error <= threshold:
            break

        # Calculate the Jacobian matrix
        jacobian = calculate_jacobian_fin_diff(dh_parameters)

        # update the joint angles using the pseudoinverse of the Jacobian
        delta_theta = np.linalg.pinv(jacobian[:3,:]) @ (translation_diff)
        theta += delta_theta

        # update the DH parameters
        dh_parameters[:,3] = theta
        iterations += 1

    if iterations == max_iterations:
        print("[WARN]: Inverse kinematics did not converge... last error: ", error)
    #else:
        #print(f'[INFO]: Converged after {iterations} iterations; last error: ', error)
        #print(f'[INFO]: Achieved final end effector position: ({", ".join(str(round(i, 4)) for i in end_effector_translation_vector)})',
        #      f'theta: ({", ".join(str(round(math.degrees(i), 4)) + "°" for i in theta)})')

    return theta

def inverse_kinematics_dls(dh_parameters: np.ndarray, end_effector: np.ndarray) -> np.ndarray:
    """
    Compute optimal joint angles for DH model and desired end-effector 
    positon / orienation using damped least squares
    """
    
    # Initialize the joint angles
    theta = dh_parameters[:,3] # np.zeros(num_joints)

    # Set the convergence threshold and maximum iterations
    threshold = 1e-6
    max_iterations = 100
    damping_constant = 0.01

    # Initialize the iteration counter and the error
    iterations = 0
    error = np.inf

    while error > threshold and iterations < max_iterations:
        # perform forward kinematics with the current joint angles
        transformations = forward_kinematics(dh_parameters)
        end_effector_translation_vector = transformations[-1][:3,3]
        end_effector_orientation_matrix = transformations[-1][:3,:3]
        end_effector_orientation = np.array(extract_euler_angles_zyx(end_effector_orientation_matrix))

        # calculate the difference in translation vectors
        translation_diff = end_effector[:3] - end_effector_translation_vector
        _translation_diff = np.multiply([1,1,1], translation_diff)

        orientation_diff = end_effector[3:] - end_effector_orientation
        _orientation_diff = np.multiply([0,1,0], orientation_diff)

        # calculate the overall error as the Euclidean distance between the differences 
        # print(translation_diff, _translation_diff)
        # print(orientation_diff, _orientation_diff)
        error_vector = np.concatenate((_translation_diff, _orientation_diff))
        error =  np.linalg.norm(error_vector)
        if error <= threshold:
            break

        # Calculate the Jacobian matrix
        jacobian = calculate_jacobian_fin_diff(dh_parameters)

        # update the joint angles using the pseudo-inverse of the Jacobian
        # delta_theta = np.linalg.pinv(jacobian[:3,:]) @ (translation_diff)
        j = jacobian # [:3,:]

        delta_theta = j.T @ np.linalg.inv(j @ j.T + damping_constant**2 * np.eye(6)) @ error_vector

        theta += delta_theta

        # update the DH parameters
        dh_parameters[:,3] = theta
        iterations += 1

    if iterations == max_iterations:
        print("[WARN]: Inverse kinematics did not converge... last error: ", error)
    #else:
        #print(f'[INFO]: Converged after {iterations} iterations; last error: ', error)
        #print(f'[INFO]: Achieved final end effector position: ({", ".join(str(round(i, 4)) for i in end_effector_translation_vector)})',
        #      f'theta: ({", ".join(str(round(math.degrees(i), 4)) + "°" for i in theta)})')

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

