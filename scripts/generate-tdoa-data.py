import numpy as np
from itertools import combinations

# Predefined xyz locations of 8 points (id: 0 to 7)
room_size = 10.0  # Size of the room in meters
predefined_points = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([room_size, 0.0, 0.0]),
    2: np.array([0.0, room_size, 0.0]),
    3: np.array([0.0, 0.0, room_size]),
    4: np.array([room_size, room_size, 0.0]),
    5: np.array([room_size, 0.0, room_size]),
    6: np.array([0.0, room_size, room_size]),
    7: np.array([room_size, room_size, room_size])
}

print("Predefined points:")
for id_, point in predefined_points.items():
    print(f"{point}")

def compute_distance_differences(location):
    """
    Given a location (x, y, z), compute the distance differences between
    every pair of predefined points relative to the location.
    
    Returns a list of (id_1, id_2, distance_difference)
    """
    location = np.array(location)
    results = []

    for id1, id2 in combinations(predefined_points.keys(), 2):
        d1 = np.linalg.norm(predefined_points[id1] - location)
        d2 = np.linalg.norm(predefined_points[id2] - location)
        diff = d1 - d2
        results.append((id1, id2, diff))
    
    return results

loc = [3, 3, 3]
output = compute_distance_differences(loc)
for entry in output:
    print(f'{{ idA: {entry[0]}, idB: {entry[1]}, dist_diff: {entry[2]:.3f}}},')