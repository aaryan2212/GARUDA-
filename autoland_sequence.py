# auto_land_sequence.py

import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil
import time
import math

# Parameters
MARKER_ID = 0  # ID of the ArUco marker to detect
MARKER_SIZE = 2.4  # Marker size in meters (8 ft ≈ 2.4 meters)
TARGET_ALTITUDE = 0.5  # Altitude in meters to initiate landing
LANDING_SPEED = 0.2  # Descent speed in m/s
SEARCH_ALTITUDE = 5.0  # Altitude to search for the marker
CAMERA_PORT = 0  # Camera port index (adjust if necessary)
BAUD_RATE = 57600  # Baud rate for MAVLink communication
SERIAL_PORT = '/dev/ttyAMA0'  # Serial port for MAVLink communication
MAX_RADIUS = 100  # Maximum search radius in meters

# Load camera calibration data
camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')

# Initialize MAVLink connection
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

# Wait for the heartbeat message to find the system ID
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Define states
STATE_SEARCHING = 'SEARCHING'
STATE_TRACKING = 'TRACKING'
STATE_LANDING = 'LANDING'

# Initialize variables
state = STATE_SEARCHING
home_position = None  # To store starting GPS position
current_position = None

# Function to arm the vehicle
def arm_vehicle():
    print("Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed.")

# Function to disarm the vehicle
def disarm_vehicle():
    print("Disarming vehicle...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Vehicle disarmed.")

# Function to change the flight mode
def set_mode(mode):
    # Check if mode is available
    if mode not in master.mode_mapping():
        print(f'Unknown mode : {mode}')
        print(f'Try: {list(master.mode_mapping().keys())}')
        return False

    mode_id = master.mode_mapping()[mode]
    # Set new mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    return True

# Function to send waypoint to the vehicle
def goto_position_target_global_int(lat, lon, alt):
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # type_mask (only positions enabled)
        int(lat * 1e7),  # Latitude (WGS84), in 1e7 * degrees
        int(lon * 1e7),  # Longitude (WGS84), in 1e7 * degrees
        alt,  # Altitude in meters
        0, 0, 0,  # x, y, z velocity in m/s (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)

# Function to get the current GPS position
def get_current_position():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000  # Convert from millimeters to meters
        return (lat, lon, alt)
    return None

# Function to calculate distance between two GPS coordinates
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(delta_lambda / 2) ** 2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Output distance in meters

# Function to generate waypoints in lawnmower pattern
def generate_search_waypoints(center_lat, center_lon, altitude, radius, spacing):
    waypoints = []
    # Determine the number of lines based on the radius and spacing
    num_lines = int((2 * radius) / spacing)
    for i in range(num_lines):
        offset = -radius + i * spacing
        # Line heading east
        waypoints.append((center_lat + (offset / 111320), center_lon - (radius / (111320 * math.cos(math.radians(center_lat)))), altitude))
        waypoints.append((center_lat + (offset / 111320), center_lon + (radius / (111320 * math.cos(math.radians(center_lat)))), altitude))
        # Line heading west
        i += 1
        offset = -radius + i * spacing
        waypoints.append((center_lat + (offset / 111320), center_lon + (radius / (111320 * math.cos(math.radians(center_lat)))), altitude))
        waypoints.append((center_lat + (offset / 111320), center_lon - (radius / (111320 * math.cos(math.radians(center_lat)))), altitude))
    return waypoints

def main():
    global state, home_position, current_position

    # Arm the vehicle
    arm_vehicle()

    # Set to GUIDED mode
    if set_mode('GUIDED'):
        print("Mode changed to GUIDED.")
    else:
        print("Failed to change mode.")
        return

    # Get home position
    home_position = get_current_position()
    if home_position is None:
        print("Failed to get home position.")
        return

    print(f"Home position: {home_position}")

    # Take off to search altitude
    print(f"Taking off to altitude: {SEARCH_ALTITUDE} meters")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, SEARCH_ALTITUDE)
    time.sleep(10)  # Wait for takeoff

    # Initialize video capture
    cap = cv2.VideoCapture(CAMERA_PORT)
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    # Define ArUco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # Generate search waypoints
    search_waypoints = generate_search_waypoints(
        home_position[0], home_position[1], SEARCH_ALTITUDE, MAX_RADIUS, spacing=20)

    waypoint_index = 0

    try:
        while True:
            # Get current position
            position = get_current_position()
            if position:
                current_position = position
            else:
                print("Failed to get current position.")
                continue

            # Read frame from camera
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame.")
                continue

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None and MARKER_ID in ids:
                # Marker detected
                state = STATE_TRACKING
            else:
                state = STATE_SEARCHING

            if state == STATE_TRACKING:
                # Process marker detection
                idx = list(ids.flatten()).index(MARKER_ID)
                marker_corners = [corners[idx]]

                # Estimate pose
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, camera_matrix, dist_coeffs)

                # Draw marker and axis
                aruco.drawDetectedMarkers(frame, marker_corners)
                aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 1)

                # Get translation vector
                tvec = tvecs[0][0]

                # PID controllers can be added here for fine adjustments (similar to previous code)

                # Initiate landing sequence
                print("Marker detected. Initiating landing sequence.")
                set_mode('LAND')
                state = STATE_LANDING
                break  # Exit the loop to allow landing

            elif state == STATE_SEARCHING:
                # Follow search waypoints
                if waypoint_index < len(search_waypoints):
                    wp = search_waypoints[waypoint_index]
                    print(f"Going to waypoint {waypoint_index + 1}/{len(search_waypoints)}: {wp}")
                    goto_position_target_global_int(wp[0], wp[1], wp[2])
                    waypoint_index += 1
                else:
                    # Search completed
                    print("Search pattern completed. Landing at home position.")
                    goto_position_target_global_int(home_position[0], home_position[1], SEARCH_ALTITUDE)
                    time.sleep(5)
                    set_mode('LAND')
                    state = STATE_LANDING
                    break  # Exit the loop to allow landing

            # Display the frame for debugging
            cv2.imshow('Landing Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)  # Loop delay

    except KeyboardInterrupt:
        print("Operation interrupted by user.")

    finally:
        # Land and disarm
        set_mode('LAND')
        time.sleep(10)  # Wait for landing
        disarm_vehicle()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
