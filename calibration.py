import numpy as np
import cv2
import glob

# Define the size of the checkerboard (number of inner corners per chessboard row and column)
CHECKERBOARD = (9, 6)

# Termination criteria for corner sub-pixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in real world space)
# Like (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load images
images = glob.glob('calibration_images/*.jpg')  # Path to your calibration images

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # If found, refine corners and add them to the list
    if ret:
        objpoints.append(objp)

        # Refine the corner locations to sub-pixel accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration to find the camera matrix, distortion coefficients, etc.
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results for later use
np.save('camera_matrix.npy', camera_matrix)
np.save('dist_coeffs.npy', dist_coeffs)

# Print results
print("Camera matrix:")
print(camera_matrix)

print("Distortion coefficients:")
print(dist_coeffs)

# Now let's undistort an image
img = cv2.imread('calibration_images/example_image.jpg')  # Load one of the images for testing
h, w = img.shape[:2]
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

# Undistort the image
dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

# Crop the image to the valid ROI
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

# Show the original and undistorted image
cv2.imshow('Original Image', img)
cv2.imshow('Undistorted Image', dst)
cv2.waitKey(0)
cv2.destroyAllWindows()
