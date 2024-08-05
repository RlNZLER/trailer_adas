import cv2
import cv2.aruco as aruco

# Define the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Generate the marker for "LEFT"
left_marker_id = 1280403796 % 250  # Using modulo to fit within DICT_6X6_250 range
left_marker_size = 200
left_marker_image = aruco.drawMarker(aruco_dict, left_marker_id, left_marker_size)
cv2.imwrite('ArUco_markers/left_marker.png', left_marker_image)

# Generate the marker for "RIGHT"
right_marker_id = 562861421956 % 250  # Using modulo to fit within DICT_6X6_250 range
right_marker_size = 200
right_marker_image = aruco.drawMarker(aruco_dict, right_marker_id, right_marker_size)
cv2.imwrite('ArUco_markers/right_marker.png', right_marker_image)
