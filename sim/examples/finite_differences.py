import numpy as np
import sys
import os
import copy

DIR_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(DIR_PATH, '..')))

from src.kinematics import *

if __name__ == "__main__":


    dh_parameters = [
        [0, np.pi/2, 0.5, 0],   # Joint 1: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 2: a, alpha, d, theta
        [1, 0, 0, 0],           # Joint 3: a, alpha, d, theta
        [0, np.pi/2, 0, 0],     # Joint 4: a, alpha, d, theta
        [0, -np.pi/2, 0.5, 0]     # Joint 5: a, alpha, d, theta
    ]

    h = 1e-6    # perturbation value (i.e., step-size)
    N = len(dh_parameters)  # num joints

    jacobian = np.zeros((3, N)) # just position pds
    
    # find pd with respect to each joint
    #   ex: 2d partial derivative
    #       δf/δx = [f(x + h,y) - f(x,y)] / h
    #       δf/δy = [f(x,y + h) - f(x,y)] / h
    for i in range(N):

        _dh = copy.deepcopy(dh_parameters)                          # dh_parameters.copy()
        ee_position = forward_kinematics(_dh)[-1][:3,3]             # find initial position
        _dh[i][3] += h                                              # perturb parameter
        ee_perturbed_position = forward_kinematics(_dh)[-1][:3,3]   # find final position
        pd = np.subtract(ee_perturbed_position, ee_position) / h    # calculate partial derivative

        print('initial position: ', ee_position)
        print(f'#{i}', dh_parameters[i][3], '-> ', _dh[i][3])
        print('final position: ', ee_perturbed_position)
        print('pd: ', pd)
        print('\n')

        jacobian[:3,i] = pd 

    other_jacobian = calculate_jacobian(forward_kinematics(dh_parameters))
    print('jacobian (finite-differences):\n', jacobian)
    print('\n')

