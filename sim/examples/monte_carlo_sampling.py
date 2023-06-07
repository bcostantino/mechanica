import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

DIR_PATH = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(DIR_PATH, '..')))

from src.kinematics import forward_kinematics


dh_parameters = np.array([
#    a,     alpha,      d,      theta
    [0,     np.pi/2,    0.5,    0],
    [1,     0,          0,      0],
    [1,     0,          0,      0],
    [0,     np.pi/2,    0,      0],
    [0,    -np.pi/2,    0.5,    0]
])

joint_limits = [
    (-180, 180),
    (0, 180),
    (-90, 90),
    (-90, 90),
    (-180, 180)
]


# generate random joint angle samples
num_samples = 1000  # Adjust the number of samples as desired

random_joint_angles = []
for _ in range(num_samples):
    sample = [np.random.uniform(min_angle, max_angle) for min_angle, max_angle in joint_limits]
    random_joint_angles.append(sample)

print('sampling end effector positions')

# compute sampling of end-effector positions
end_effector_positions = []
for joint_angles in random_joint_angles:
    dh_parameters[:,3] = joint_angles
    transformations = forward_kinematics(dh_parameters)
    end_effector_positions.append(transformations[-1][:3,3])
end_effector_positions = np.array(end_effector_positions)


# determine the bounding volume
hull = ConvexHull(end_effector_positions)


# plot the convex hull
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2], color='blue')

for s in hull.simplices:
    s = np.append(s, s[0])  # Close the loop
    ax.plot(end_effector_positions[s, 0], end_effector_positions[s, 1], end_effector_positions[s, 2], 'r-')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Manipulator Workspace - Convex Hull')

# Show the plot
plt.show()


