import numpy as np

from scipy.spatial.transform import Rotation as R

def get_rotation_from_p2_to_p1(p1, p2):
    # Calculate the direction vectors
    direction1 = np.array(p2) - np.array(p1)
    direction1 /= np.linalg.norm(direction1)  # Normalize

    # Assuming the upward vector is (0, 0, 1) for a typical 3D environment
    upward = np.array([0, 0, 1])
    
    # Calculate the rotation axis (cross product)
    rotation_axis = np.cross(upward, direction1)
    rotation_angle = np.arccos(np.clip(np.dot(upward, direction1), -1.0, 1.0))

    # Create the rotation
    rotation = R.from_rotvec(rotation_axis * rotation_angle)
    return rotation.as_quat()  # Return quaternion representation


def get_point_between(p1, p2, distance):
    # Convert points to numpy arrays for easier calculations
    point1 = np.array(p1)  # (x1, y1, z1)
    point2 = np.array(p2)  # (x2, y2, z2)

    # Calculate the direction vector from point 1 to point 2
    direction = point2 - point1

    # Normalize the direction vector
    norm = np.linalg.norm(direction)
    if norm == 0:
        raise ValueError("Points p1 and p2 cannot be the same.")

    normalized_direction = direction / norm

    # Scale the normalized direction by the desired distance
    scaled_vector = normalized_direction * distance

    # Calculate the new point
    new_point = point2 + scaled_vector
    return new_point

# Example usage:
p1 = (1.0, 2.0, 3.0)  # Point 1 (x1, y1, z1)
p2 = (4.0, 5.0, 6.0)  # Point 2 (x2, y2, z2)
distance_from_p2 = 0.5

new_position = get_point_between(p1, p2, distance_from_p2)
print("New position:", new_position)

# Calculate the rotation from p2 to p1
rotation_quaternion = get_rotation_from_p2_to_p1(p1, p2)
print("Rotation quaternion:", rotation_quaternion)
