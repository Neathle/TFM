import numpy as np
from scipy.linalg import svd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.distance import cdist

def calculate_rectangle(points):
    # Step 1: Calculate the center of mass (COM)
    COM = np.mean(points, axis=0)


    # Step 2: Fit a plane to the points and find its normal vector
    _, _, Vt = svd(points - COM)
    normal = Vt[2, :]


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
    second_direction_index = np.argmax(dot_products[1:]) + 1

    for i in range(4):
        for j in range(i+1, 4):
            if not (np.isclose(dot_products[i, j], 1, atol=0.1) or np.isclose(dot_products[i, j], 0, atol=0.1)):
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
    mean_directions[first_pair_indices[0]] = np.mean(directions[first_pair_indices[0]], axis=0)
    mean_directions[first_pair_indices[1]] = np.mean(directions[first_pair_indices[0]], axis=0)
    mean_directions[second_pair_indices[0]] = np.mean(directions[second_pair_indices[0]], axis=0)
    mean_directions[second_pair_indices[1]] = np.mean(directions[second_pair_indices[0]], axis=0)


    # Create a new figure and add a 3D subplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the original points
    ax.scatter(points_proj[:, 0], points_proj[:, 1], points_proj[:, 2], color='b')

    # Plot the lines
    lines = np.zeros((4, 2, 3))
    for i in range(4):
        line = centers[i] + np.array([-1, 1])[:, None] * directions[i]
        lines[i] = line
        ax.plot(line[:, 0], line[:, 1], line[:, 2])
        ax.scatter(centers[i, 0], centers[i, 1], centers[i, 2], color='r')

    # Set the labels of the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Display the plot
    plt.show()

        
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
    side_centers = (rectangle + np.roll(rectangle, -1, axis=0)) / 2
    x_axis_index = np.argmin(side_centers[:, 1])
    x_axis = rectangle[(x_axis_index+1)%4] - rectangle[x_axis_index]

    # Ensure the y-axis has a positive direction
    if np.cross(x_axis, normal)[1] < 0:
        x_axis = -x_axis

    y_axis = np.cross(normal, x_axis)
    z_axis = normal

    # Step 7: Calculate the roll, pitch, and yaw angles
    yaw = np.arctan2(y_axis[0], x_axis[0])
    pitch = np.arctan2(-z_axis[0], np.sqrt(z_axis[1]**2 + z_axis[2]**2))
    roll = np.arctan2(z_axis[1], z_axis[2])

    rpy_angles = np.array([roll, pitch, yaw])


    # Step 8: Return the projected COM position, rpy angles, width, and height
    width = np.linalg.norm(rectangle[(x_axis_index+1)%4] - rectangle[x_axis_index])
    height = np.linalg.norm(rectangle[(x_axis_index+2)%4] - rectangle[(x_axis_index+1)%4])

    return COM_proj, rpy_angles, width, height, rectangle



# Example usage:
points = np.array([
    [-0.810, 0.893, 1.802],
    [-1.274, 0.893, 1.796],
    [-0.808, 0.531, 1.796],
    [-1.267, 0.525, 1.792],
])
COM_proj, rpy_angles, width, height, rectangle = calculate_rectangle(points)
print(f"Projected COM position: {COM_proj}")
print(f"Roll-Pitch-Yaw angles: {rpy_angles}")
print(f"Width: {width}")
print(f"Height: {height}")




# Create a new figure and add a 3D subplot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the original points
ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='b')

# Plot the rectangle
rectangle = np.concatenate([rectangle, rectangle[:1]])  # Close the rectangle
ax.plot(rectangle[:, 0], rectangle[:, 1], rectangle[:, 2], color='r')

# Set the labels of the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Display the plot
plt.show()