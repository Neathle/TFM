import numpy as np
from scipy.linalg import svd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation as R
import rospy
import tf.transformations as tf
import yaml

def calculate_rectangle(points):
    # Step 1: Calculate the center of mass (COM)
    COM = np.mean(points, axis=0)


    # Step 2: Fit a plane to the points and find its normal vector
    _, _, Vt = svd(points - COM)
    normal = -Vt[2, :] if np.sum(Vt[2, :]) > 0 else Vt[2, :]


    # Step 3: Project the points and the COM onto the plane
    points_proj = points - np.dot(points - COM, normal)[:, None] * normal
    COM_proj = COM - np.dot(COM - COM, normal) * normal


    # Step 4: Calculate the mean direction and center of each side
    centers = np.zeros((4, 3))
    directions = np.zeros((4, 3))
    for i in range(4):
        centers[i] = (points_proj[i] + points_proj[(i+1)%4]) / 2
        directions[i] = points_proj[(i+1)%4] - points_proj[i]
    directions /= np.linalg.norm(directions, axis=1)[:, None]


    # Find the direction that has the maximum dot product with the first direction
    dot_products = np.abs(np.dot(directions, directions.T))
    second_direction_index = np.argmax(dot_products[0][1:]) + 1

    for i in range(4):
        for j in range(i+1, 4):
            if not (np.isclose(dot_products[i, j], 1, atol=0.15) or np.isclose(dot_products[i, j], 0, atol=0.15)):
                new_points = np.zeros((4, 3))
                # If not, reorder the last two points and start over
                new_points[0] = points[0]
                new_points[1] = points[1]
                new_points[2] = points[3]
                new_points[3] = points[2]
                return calculate_rectangle(new_points)

    # The first pair of directions is then the first and second_direction_index directions
    first_pair_indices = [0, second_direction_index]

    # The second pair is the remaining two directions
    second_pair_indices = [1, 2, 3]
    second_pair_indices.remove(second_direction_index)

    # Calculate the mean direction of each pair
    mean_directions = np.zeros((4, 3))
    first_mean_direction = np.mean(directions[first_pair_indices], axis=0)
    second_mean_direction = np.mean(directions[second_pair_indices], axis=0)
    mean_directions[first_pair_indices] = first_mean_direction
    mean_directions[second_pair_indices] = second_mean_direction


    # Create a new figure and add a 3D subplot
    fig = plt.figure()
    ax = fig.add_subplot(121, projection='3d')

    # Plot the original points
    ax.scatter(points_proj[:, 0], points_proj[:, 1], points_proj[:, 2], color='b')

    # Plot the lines
    lines = np.zeros((4, 2, 3))
    for i in range(4):
        line = centers[i] + np.array([-1, 1])[:, None] * mean_directions[(i+1)%4]
        lines[i] = line
        ax.plot(line[:, 0], line[:, 1], line[:, 2])
        ax.scatter(centers[i, 0], centers[i, 1], centers[i, 2], color='r')

    # Set the labels of the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Display the plot
    # plt.show()

        
    # Find the intersections of these lines to form a new rectangle
    rectangle = np.zeros((0, 3))
    for i in first_pair_indices:
        for j in second_pair_indices:
            w = centers[i] - centers[j]
            line1 = lines[i][1] - lines[i][0]
            line2 = lines[j][1] - lines[j][0]
            cross1 = np.cross(w, line2)
            cross2 = np.cross(line1, line2)

            sI = np.dot(cross1, cross2) / np.linalg.norm(cross2)**2
            intersection = np.reshape(centers[i] + sI * line1, (1, -1))
            rectangle = np.append(rectangle,intersection, axis=0)

    # Reorder points to match original order
    # Compute pairwise distance matrix
    dist_matrix = cdist(points, rectangle)

    # Get the indices of the minimum element in each row (along axis 1)
    min_indices = np.argmin(dist_matrix, axis=1)

    # Use these indices to reorder rectangle corners
    rectangle = rectangle[min_indices]    


    # Step 6: Define the x and y axes of the rectangle
    x_axis_index = np.argmin(centers[:, 2])
    x_axis = rectangle[(x_axis_index+1)%4] - rectangle[x_axis_index]

    # Ensure the y-axis has a positive direction
    if np.cross(x_axis, normal)[2] < 0:
        x_axis = -x_axis

    y_axis = np.cross(normal, x_axis)
    z_axis = normal


    # Step 8: Return the projected COM position, rpy angles, width, and height
    width = np.linalg.norm(rectangle[(x_axis_index+1)%4] - rectangle[x_axis_index])
    height = np.linalg.norm(rectangle[(x_axis_index+2)%4] - rectangle[(x_axis_index+1)%4])

    # Create a new figure and add a 3D subplot
    ax = fig.add_subplot(122, projection='3d')

    # Plot the original points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='b')

    # Plot the rectangle
    rectangle = np.concatenate([rectangle, rectangle[:1]])  # Close the rectangle
    ax.plot(rectangle[:, 0], rectangle[:, 1], rectangle[:, 2], color='r')

    # Plot COM_proj in a different color
    ax.scatter(COM_proj[0], COM_proj[1], COM_proj[2], color='g')

    # Add an orthonormal frame with size width/5 given the rpy angles
    x_axis = x_axis / np.linalg.norm(x_axis) * width / 5
    y_axis = y_axis / np.linalg.norm(y_axis) * width / 5
    z_axis = z_axis / np.linalg.norm(z_axis) * width / 5
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], x_axis[0], x_axis[1], x_axis[2], color='r')
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], y_axis[0], y_axis[1], y_axis[2], color='g')
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], z_axis[0], z_axis[1], z_axis[2], color='b')


    # Step 7: Calculate the roll, pitch, and yaw angles
    # rot_matrix = np.array([[1,0,0], [0,1,0], [0,0,1]]).T
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    rot_matrix = np.array([z_axis,x_axis,y_axis])
    print(rot_matrix)
    rpy_angles = tf.euler_from_matrix(rot_matrix, axes="rzyx")
    rot_matrix = tf.euler_matrix(rpy_angles[0], rpy_angles[1], rpy_angles[2], "rzyx")
    print("Quat: ",tf.quaternion_from_matrix(rot_matrix))
    print(rot_matrix)

    x_axis = rot_matrix[0,:-1] * width / 5
    y_axis = rot_matrix[1,:-1] * width / 5
    z_axis = rot_matrix[2,:-1] * width / 5
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], x_axis[0], x_axis[1], x_axis[2], color='r')
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], y_axis[0], y_axis[1], y_axis[2], color='g')
    ax.quiver(COM_proj[0], COM_proj[1], COM_proj[2], z_axis[0], z_axis[1], z_axis[2], color='b')

    # Set the limits of the plot
    ax.set_xlim([COM_proj[0] - width, COM_proj[0] + width])
    ax.set_ylim([COM_proj[1] - width, COM_proj[1] + width])
    ax.set_zlim([COM_proj[2] - width, COM_proj[2] + width])

    # Set the labels of the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Display the plot
    # plt.show()

    return COM_proj, rpy_angles, width, height, rectangle

    


RECTANGLES = np.array([
    [ # Cardboard
        [-0.810, 0.893, 1.802],
        [-1.274, 0.893, 1.796],
        [-0.808, 0.531, 1.796],
        [-1.267, 0.525, 1.792],
    ],
    [ # Window
        [ 2.312, 2.075, 1.859],
        [ 1.137, 2.062, 1.834],
        [ 2.246, 0.988, 1.866],
        [ 1.150, 0.974, 1.842],
    ],
    [ # Wash Hands Sign
        [-1.687, 2.017, 1.806],
        [-1.880, 2.017, 1.805],
        [-1.682, 1.761, 1.806],
        [-1.864, 1.764, 1.805],
    ],
    [ # Rotated Banner
        [-0.218, 1.901, 1.817],
        [-0.405, 1.713, 1.813],
        [ 0.097, 1.586, 1.818],
        [-0.080, 1.412, 1.811],
    ],
    [ # Keep Distance Sign
        [ 3.077, 1.697, 1.868],
        [ 2.899, 1.708, 1.869],
        [ 3.068, 1.458, 1.869],
        [ 2.907, 1.445, 1.869],
    ],
    [ # Mask Sign
        [ 3.324, 2.113, 1.663],
        [ 3.313, 2.114, 1.892],
        [ 3.329, 1.872, 1.650],
        [ 3.317, 1.866, 1.884],
    ],
])

PERMUTATION = [2, 0, 1] # X Y Z mocap -> Z X Y map
RECTANGLES = RECTANGLES[:, :, PERMUTATION]

OFFSETS = ["+x", "+x", "+x", "+x", "+x", "+y"] # Offset the points according to the marker leg length
OFFSET_AMOUNT = 0.025 #marker leg length

data = {"marker_positions": []}

for rectangle_id, points in enumerate(RECTANGLES):
    for point in points:
        sense = 1 if OFFSETS[rectangle_id][0] == "+" else -1
        direction = 0 if OFFSETS[rectangle_id][1] == "x" else 1 if OFFSETS[rectangle_id][1] == "y" else 2 if OFFSETS[rectangle_id][1] == "z" else 999
        point[direction] = point[direction] + sense * OFFSET_AMOUNT


    COM_proj, rpy_angles, width, height, rectangle = calculate_rectangle(points)
    print(f"Projected COM position: {COM_proj}")
    print(f"Roll-Pitch-Yaw angles: {rpy_angles}")
    print(f"Width: {width}")
    print(f"Height: {height}")
    print(f"Rectangle: {rectangle}")

    data['marker_positions'].append({"x": float(COM_proj[0]), "y": float(COM_proj[1]), "z": float(COM_proj[2]), 
                                  "roll": float(rpy_angles[2]), "pitch": float(rpy_angles[1]), "yaw": float(rpy_angles[0]), 
                                  "width": float(width), "height": float(height), "ID": rectangle_id, # "corners": rectangle.tolist()[:-1],
                                  "sector": 0, "map": 0})

with open("src/amcl_hybrid/maps/marker_positions.yml", "w") as f:
        yaml.dump(data, f)




