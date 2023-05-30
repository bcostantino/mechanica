import numpy as np
import copy
import math

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

def calculate_jacobian_fin_diff_old(dh_parameters, h = 1e-6):
    N = len(dh_parameters)
    jacobian = np.zeros((3, N))

    for i in range(N):
        _dh = copy.deepcopy(dh_parameters)                          # dh_parameters.copy()
        ee_position = forward_kinematics(_dh)[-1][:3,3]             # find initial position
        _dh[i][3] += h                                              # perturb parameter
        ee_perturbed_position = forward_kinematics(_dh)[-1][:3,3]   # find final position
        pd = np.subtract(ee_perturbed_position, ee_position) / h    # calculate partial derivative
        jacobian[:3,i] = pd

    return jacobian

def calculate_jacobian_fin_diff(dh_parameters, h = 1e-6):
    N = len(dh_parameters)
    jacobian = np.zeros((6, N))

    for i in range(N):
        _dh = copy.deepcopy(dh_parameters)  # dh_parameters.copy()
        
        # find initial transformation matrix for end effector
        ee_trans = forward_kinematics(_dh)[-1]
        ee_position = ee_trans[:3,3]
        ee_orientation = extract_euler_angles(ee_trans[:3,:3])
        
        _dh[i][3] += h  # perturb parameter
        
        # find final transformation matrix
        ee_f_trans = forward_kinematics(_dh)[-1]
        ee_f_position = ee_f_trans[:3,3]   # find final transformation matrix
        ee_f_orientation = extract_euler_angles(ee_f_trans[:3,:3])

        # calculate position and orientation partial derivatives
        dp_dt = np.subtract(ee_f_position, ee_position) / h
        da_dt = np.subtract(ee_f_orientation, ee_orientation) / h

        jacobian[:3,i] = dp_dt
        jacobian[3:6,i] = da_dt

    return jacobian


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


def inverse_kinematics_jpi(dh_parameters, end_effector):
    # print('starting inverse kinemetics for end effector position: ', end_effector[:3])

    # Extract the number of joints

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


def inverse_kinematics_dls(dh_parameters, end_effector):

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

        # calculate the difference in translation vectors
        translation_diff = end_effector - end_effector_translation_vector

        # calculate the overall error as the Euclidean distance between the differences
        error =  np.linalg.norm(translation_diff) 
        if error <= threshold:
            break

        # Calculate the Jacobian matrix
        jacobian = calculate_jacobian_fin_diff(dh_parameters)

        # update the joint angles using the pseudo-inverse of the Jacobian
        # delta_theta = np.linalg.pinv(jacobian[:3,:]) @ (translation_diff)
        j = jacobian[:3,:]
        delta_theta = j.T @ np.linalg.inv(j @ j.T + damping_constant**2 * np.eye(3)) @ translation_diff

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


