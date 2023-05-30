
### Algorithms

The following are algorithms used to compute inverse kinematics. In the context of inverse kinematics, we have a desired end effector position (or error) represented by the error vector, and we want to find the corresponding joint angle adjustments (delta theta) that will move the end effector towards the desired position.

#### The Jacobian Inverse Technique

The arm end-effector is represented by a 3D position function, dependent on each of the `N` joint angles in the system. The jacobian is a `3xN` matrix that contains all first-order partial derivatives of the multivariate position function. The jacobian inverse technique utilizes the jacobian iteratively to converge on a set of joint angles that minimize the error between the position and desired position. 

The Jacobian relates the changes in joint angles to the resulting changes in the end effector's position. When we multiply the pseudo-inverse of the Jacobian, **J**<sup>+</sup> by the error vector **e**, we are effectively solving the equation system **JΔθ** = **e** for **Δθ**, where **J** is the Jacobian matrix and **Δθ** is the vector of joint angle adjustments. The pseudo-inverse is used instead of the true inverse to handle cases where the Jacobian is not square or not full rank.

##### IKJI1 - No orientation control

1. initialize joint angles <i>θ</i> using DH parameters or random guess.
2. define hyperparameters: convergence threshold and max # of iterations.
3. iterate until error diminishes past threshold
    1. calculate current end-effector transformation matrix (position) with FK
    2. calculate error between desired and current positions (and break if threshold passed)
    3. calculate the jacobian using fin-diff
    4. calculate <i>Δθ</i> by multiplying the jacobian inverse by the error vector
    5. update <i>θ</i> and DH parameters
    6. repeat steps i - v until convergence criteria is met
4. return final <i>θ</i> 







### TODO

1. Damped Least Squares: Instead of using the pseudo-inverse directly, you can incorporate damping into the calculation. This helps to reduce the amplification of small errors and can improve convergence. The damped least squares method involves adding a damping term to the diagonal elements of the Jacobian matrix before calculating the pseudo-inverse.

2. Levenberg-Marquardt Method: The Levenberg-Marquardt algorithm is an optimization method commonly used for nonlinear least squares problems. It combines aspects of both the Gauss-Newton method and the gradient descent method. This method can provide faster convergence and better stability in some cases.

3. Null Space Control: If you have multiple objectives or constraints in your inverse kinematics problem, such as avoiding joint limits or optimizing secondary objectives, you can use null space control. It involves projecting the desired task-space motion into the null space of the primary task to achieve secondary objectives while maintaining the primary task. This allows for more flexible and robust control.


```js
function inverseKinematics(dh_parameters, end_effector, threshold):
    num_joints = length(dh_parameters)
    joint_angles = initializeJointAngles(num_joints)  // Initialize joint angles to an initial guess

    do:
        forward_kinematics = calculateForwardKinematics(dh_parameters, joint_angles)
        error_position = calculatePositionError(forward_kinematics, end_effector)
        error_orientation = calculateOrientationError(forward_kinematics, end_effector)
        error = concatenateErrors(error_position, error_orientation)

        if isWithinThreshold(error, threshold):
            break  // End if error is within the threshold

        jacobian = calculateJacobian(dh_parameters, joint_angles)
        delta_theta = calculateDeltaTheta(jacobian, error)

        joint_angles = updateJointAngles(joint_angles, delta_theta)
    while true  // Repeat until the error is within the threshold

    return joint_angles

function calculateForwardKinematics(dh_parameters, joint_angles):
    // Perform forward kinematics to calculate the end effector's position and orientation
    // based on the DH parameters and joint angles
    // Return the resulting transformation matrix representing the end effector's pose

function calculatePositionError(forward_kinematics, end_effector):
    // Calculate the position error between the expected and actual end effector position
    // Return the position error as a 3D vector

function calculateOrientationError(forward_kinematics, end_effector):
    // Calculate the orientation error between the expected and actual end effector orientation
    // Return the orientation error as a 3D vector representing the differences in pitch, roll, and yaw

function concatenateErrors(error_position, error_orientation):
    // Concatenate the position and orientation errors into a single vector
    // Return the concatenated error vector

function isWithinThreshold(error, threshold):
    // Check if the error is within the specified threshold
    // Return true if the error is within the threshold, false otherwise

function calculateJacobian(dh_parameters, joint_angles):
    // Calculate the Jacobian matrix based on the DH parameters and joint angles
    // Return the Jacobian matrix

function calculateDeltaTheta(jacobian, error):
    // Calculate the change in joint angles (delta_theta) using the Jacobian and error
    // Return the delta_theta as a vector of joint angle changes

function updateJointAngles(joint_angles, delta_theta):
    // Update the joint angles by adding the delta_theta to the current joint angles
    // Return the updated joint angles


```
