
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





### Issues

1. **Gimbal lock / signularities** - With end-effector orientation being represented by euler angles (roll, pitch & yaw) there are *ambiguities* that occur when the end effector is pointing parallel to the +z or -z axis; both resulting in an end-effector pitch angle of 0°. This can be observed in the 2D xz-plane plot by sweeping the first joint angle 360° and monitoring the end effector pitch during the simulation. When pointing in +x and -x the pitch is 90° and -90° respectively, but the singularity always occurs at expected 0° and 180° pitch attitudes. This issue causes the IK algorithm to be unable to converge on the correct orientation.

    To fix, represent angles using quaternions. When using quaternions, the pitch ambiguity problem can be resolved, as quaternions can represent any orientation without gimbal lock or singularities. The advantage lies in their ability to represent orientations in a continuous and non-redundant manner, providing a complete and unique representation of rotations in three-dimensional space. The Jacobian matrix, when extended to include quaternion components, allows for the calculation of the rotational error between the desired and current orientations using quaternion algebra. The error is then used to update the joint angles based on the inverse kinematics algorithm.








### TODO



- [x] Damped Least Squares: Instead of using the pseudo-inverse directly, you can incorporate damping into the calculation. This helps to reduce the amplification of small errors and can improve convergence. The damped least squares method involves adding a damping term to the diagonal elements of the Jacobian matrix before calculating the pseudo-inverse.
- [ ] Levenberg-Marquardt Method: The Levenberg-Marquardt algorithm is an optimization method commonly used for nonlinear least squares problems. It combines aspects of both the Gauss-Newton method and the gradient descent method. This method can provide faster convergence and better stability in some cases.
- [ ] If there are multiple objectives or constraints in the inverse kinematics problem, such as avoiding joint limits or optimizing secondary objectives, you can use null space control. It involves projecting the desired task-space motion into the null space of the primary task to achieve secondary objectives while maintaining the primary task. This allows for more flexible and robust control.
- [ ] In IK, add control to minimize torque on each joint for the final position

