import cv2
import cv2.aruco as aruco

# Choose the dictionary (same as in your detector)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Marker ID to generate (0 to 49 in this dictionary)
marker_id = 43

# Size in pixels
marker_size = 300

# Generate the marker
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Save to a file
cv2.imwrite(f"aruco_marker_{marker_id}.png", marker_image)

# Optional: display it
cv2.imshow("Aruco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()