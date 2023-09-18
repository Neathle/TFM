import numpy as np
from scipy.optimize import minimize, basinhopping
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R


def fit_rectangle(corners):
    def distance_to_rectangle(params):
        x, y, z, roll, pitch, yaw, width, height = params
        rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        center = np.array([x, y, z])

        # Define the rectangle corners based on width and height
        rectangle_corners = np.array([
            [-width/2, -height/2, 0],
            [width/2, -height/2, 0],
            [width/2, height/2, 0],
            [-width/2, height/2, 0]
        ])
        
        transformed_corners = [np.dot(rotation_matrix, corner) + center for corner in rectangle_corners]
        distances = [np.linalg.norm(rc - cc) for rc, cc in zip(transformed_corners, corners)]
        return sum(distances)
    
    com = np.mean(corners, axis=0)
    initial_guess = [com[0], com[1], com[2], 0, 0, 0, 1, 1]
    result = minimize(distance_to_rectangle, initial_guess, method='SLSQP', options={'maxiter': 500000})
    return result.x

def generate_yaml(marker_positions):
    data = {"marker_positions": [{"x": float(x), "y": float(y), "z": float(z), "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw), "width": float(width), "height": float(height), "ID": i} for i, (x, y, z, roll, pitch, yaw, width, height) in enumerate(marker_positions)]}
    with open("marker_positions.yml", "w") as f:
        yaml.dump(data, f)



corners = [
    [-0.810, 0.893, 1.802],
    [-1.274, 0.893, 1.796],
    [-0.808, 0.531, 1.796],
    [-1.267, 0.525, 1.792]
]  # Replace with your list of corners
marker_positions = [fit_rectangle(corners)]
generate_yaml(marker_positions)

#Plot first
rectangle = marker_positions[0]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot corners
xs, ys, zs = zip(*corners)
ax.scatter(xs, ys, zs, color='b')

# Plot rectangle center
ax.scatter(*rectangle[:3], color='r')

# Define the rectangle corners based on width and height
rectangle_corners = np.array([
    [-rectangle[6]/2, -rectangle[7]/2, 0],
    [rectangle[6]/2, -rectangle[7]/2, 0],
    [rectangle[6]/2, rectangle[7]/2, 0],
    [-rectangle[6]/2, rectangle[7]/2, 0]
])

# Apply rotation and translation to rectangle corners
rotation_matrix = R.from_euler('xyz', rectangle[3:6]).as_matrix()
transformed_corners = [np.dot(rotation_matrix, corner) + rectangle[:3] for corner in rectangle_corners]

# Plot rectangle edges
xs, ys, zs = zip(*transformed_corners)
ax.plot(xs + xs[:1], ys + ys[:1], zs + zs[:1], color='r')

# # Set axes limits
# ax.set_xlim([-2, 2])
# ax.set_ylim([ 0, 2])
# ax.set_zlim([-2, 2])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()