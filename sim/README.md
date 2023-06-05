
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="https://pixlok.com/wp-content/uploads/2021/12/Robot-Icon-09iujcdf.png" alt="Logo" width="100" height="100">
  </a>

  <h3 align="center">Mechanica Kinematics Simulator</h3>

  <p align="center">
    A kinematics simulator for rigid multi-body systems
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Report Bug</a>
    ·
    <a href="https://github.com/othneildrew/Best-README-Template/issues">Request Feature</a>
  </p>
</div>


### Algorithms

The following are algorithms used to compute inverse kinematics; in which we're given a desired end-effector pose, ***p(θ)***, and we want to find the corresponding joint angles, ***θ***, such that the end effector is positioned (and/or oriented) in the desired configuration.

#### The Jacobian Inverse Technique

For a serial manipulator, the final end-effector pose function is dependent on each of the *N* joint angles in the system. The jacobian is a 6x*N* matrix that contains all first-order partial derivatives of the multivariate pose function. General techniques involving the jacobian utilize an open-form approach, iteratively updating joint angles based on numerical approximations to converge on a set of joint angles that minimize the error between the current and target pose. 

The Jacobian matrix ***J*** relates the changes in joint angles to the resulting changes in the end effector's position. When we multiply the pseudo-inverse of the Jacobian, ***J***<sup>+</sup>, by the error vector ***Δp***, we are effectively solving the equation system ***J Δθ*** = ***Δp*** for the vector of joint angle adjustments ***Δθ*** such that the end-effector is displaced by ***Δp***. The pseudo-inverse is used instead of the true inverse to handle cases where the Jacobian is not square or not full rank.

##### IKJI1 - No orientation control

1. initialize joint angles *θ* using DH parameters or random guess.
2. define hyperparameters: convergence threshold and max # of iterations.
3. iterate until error diminishes past threshold
    1. calculate current end-effector transformation matrix (position) with FK
    2. calculate error between desired and current positions (and break if threshold passed)
    3. calculate the jacobian using fin-diff
    4. calculate *Δθ* by multiplying the jacobian inverse by the error vector
    5. update *θ* and DH parameters
    6. repeat steps i - v until convergence criteria is met
4. return final *θ* 





<!-- ### Issues

1. **Gimbal lock / signularities** - With end-effector orientation being represented by euler angles (roll, pitch & yaw) there are *ambiguities* that occur when the end effector is pointing parallel to the +z or -z axis; both resulting in an end-effector pitch angle of 0°. This can be observed in the 2D xz-plane plot by sweeping the first joint angle 360° and monitoring the end effector pitch during the simulation. When pointing in +x and -x the pitch is 90° and -90° respectively, but the singularity always occurs at expected 0° and 180° pitch attitudes. This issue causes the IK algorithm to be unable to converge on the correct orientation.

    To fix, represent angles using quaternions. When using quaternions, the pitch ambiguity problem can be resolved, as quaternions can represent any orientation without gimbal lock or singularities. The advantage lies in their ability to represent orientations in a continuous and non-redundant manner, providing a complete and unique representation of rotations in three-dimensional space. The Jacobian matrix, when extended to include quaternion components, allows for the calculation of the rotational error between the desired and current orientations using quaternion algebra. The error is then used to update the joint angles based on the inverse kinematics algorithm. -->




### Requirements / To-Do

- [x] Damped Least Squares (DLS): Instead of using the pseudo-inverse directly, you can incorporate damping into the calculation. This helps to reduce the amplification of small errors and can improve convergence. The damped least squares method involves adding a damping term to the diagonal elements of the Jacobian matrix before calculating the pseudo-inverse.
- [ ] Levenberg-Marquardt (LM) Method: The Levenberg-Marquardt algorithm is an optimization method commonly used for nonlinear least squares problems. It combines aspects of both the Gauss-Newton method and the gradient descent method. This method can provide faster convergence and better stability in some cases.
- [ ] If there are multiple objectives or constraints in the inverse kinematics problem, such as avoiding joint limits or optimizing secondary objectives, you can use null space control. It involves projecting the desired task-space motion into the null space of the primary task to achieve secondary objectives while maintaining the primary task. This allows for more flexible and robust control.
- [ ] In IK, add general constraints like joint angle limits, objectives to minimize torque on each joint, etc.
- [ ] In IK, integrate several methods for updating joint angles (e.g., pinv, DLS, LM); conditionally choosing which to use based off linearity, jacobian conditioning, damping sensitivity, pose error magnitude (convergence time), available resources, etc. 
- [ ] Instead of calculating the exact Jacobian matrix, use approximation techniques such as finite differences or numerical differentiation to estimate the Jacobian. This can reduce the computational cost, especially for high-dimensional systems. Additionally, if the Jacobian has a sparse structure, memory requirements and computation time can be reduced by using sparse matrix algorithms and data structures to efficiently handle the sparse Jacobian.
- [ ] Look into parallel computing and GPU acceleration for matrix operations
- [ ] Store and reuse past results to avoid redundant computations for end-effector poses that have already been computed.
- [ ] Test line search (i.e., backtracking line search, golden section search, etc.) for tuning hyper-parameters like damping factor, step-size and/or lambda factor based on an error metric.


### Resources

1. Bus, S. R. (2009, October 7). Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods. Carnegie Mellon University, School of Computer Science. https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf 
1. Fang, G., Tian, Y., Yang, Z.-X., Geraedts, J. M. P., &amp; Wang, C. C. L. (2022, June 5). Efficient Jacobian-Based Inverse Kinematics with Sim-to-Real Transfer of Soft Robots by Learning. arXiv.org. https://arxiv.org/abs/2012.13965 
1. Keninck, S. D. (2018, April 16). Geometric Algebra 4 Lines Inverse Kinematics Solver. Observable. https://observablehq.com/@enkimute/geometric-algebra-4-lines-inverse-kinematics-solver 
1. Meredith, M., &amp; Maddock, S. (2004, January). University of Sheffield, Department of Computer Science, Real-Time Inverse Kinematics: The Return of the Jacobian. ResearchGate. https://www.researchgate.net/publication/228980657_Real-time_inverse_kinematics_The_return_of_the_Jacobian 
1. Nilsson, R. (2009). 2009:142 CIV Master’s Thesis, Inverse Kinematics. Diva. https://www.diva-portal.org/smash/get/diva2:1018821/FULLTEXT01.pdf 
1. Radavelli, L. A., Simoni, R., De Pieri, E. R., &amp; Martins, D. (2012, January). A Comparative Study of the Kinematics of Robots Manipulators by Denavit-Hartenberg and Dual Quaternion. ResearchGate. https://www.researchgate.net/publication/260354285_A_Comparative_Study_of_the_Kinematics_of_Robots_Manipulators_by_Denavit-Hartenberg_and_Dual_Quaternion 
1. Schaal, S., &amp; Planck, M. (n.d.). USC Robotics, Jacobian Methods for Inverse Kinematics and Planning. University of Washington Computer Science &amp; Engineering. https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf 
1. Schinstock, D. E., Faddis, T. N., &amp; Greenway, R. B. (n.d.). University of Kansas, Mechanical Engineering - Robust Inverse Kinematics Using Damped Least Squares with Dynamic Weighting. NTRS - NASA Technical Reports Server. https://ntrs.nasa.gov/api/citations/19950005142/downloads/19950005142.pdf 
1. Vidakovic, J. Z., Lazarevic, M. P., Kvrgic, V. M., Dancuo, Z. Z., &amp; Ferenc, G. Z. (2014, January). Advanced Quaternion Forward Kinematics Algorithm Including Overview of Different Methods for Robot Kinematics . University near Beograd, Faculty of Machinery. https://www.mas.bg.ac.rs/_media/istrazivanje/fme/vol42/3/3_jvidakovic.pdf 
1. Zaplana, I., Hadfield, H., &amp; Lasenby, J. (2022, March 14). Closed-form Solutions for the Inverse Kinematics of Serial Robots using Conformal Geometric Algebra. arXiv.org. https://arxiv.org/abs/2109.12411 

