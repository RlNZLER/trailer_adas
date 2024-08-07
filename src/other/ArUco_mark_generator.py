import cv2
import cv2.aruco as aruco

'''
Generated marker for m1 with ID 235
Generated marker for m2 with ID 236
Generated marker for m3 with ID 237
Generated marker for m4 with ID 238
Generated marker for m5 with ID 239
'''

# Define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Define tags and their corresponding ASCII-derived IDs
tags = {
    "m1": 27985,
    "m2": 27986,
    "m3": 27987,
    "m4": 27988,
    "m5": 27989
}

# Function to generate and save ArUco markers
def generate_marker(tag, marker_id, marker_size=200):
    marker_id_mod = marker_id % 250  # Using modulo to fit within DICT_6X6_250 range
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id_mod, marker_size)
    cv2.imwrite(f'ArUco_markers/{tag}_marker.png', marker_image)
    print(f"Generated marker for {tag} with ID {marker_id_mod}")

# Generate markers for each tag
for tag, marker_id in tags.items():
    generate_marker(tag, marker_id)
