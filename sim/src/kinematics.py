import numpy as np
import copy

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

def calculate_jacobian_fin_diff(dh_parameters, h = 1e-6):
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



