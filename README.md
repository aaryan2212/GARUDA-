# GARUDA-
Here’s a detailed README for your **VTOL Auto Landing on ArUco Marker** project. The README covers all the essential aspects of your project, including its purpose, features, setup instructions, and how it works.

---

# **VTOL Auto Landing on ArUco Marker**

This project implements an autonomous landing system for a tilt-rotor VTOL (Vertical Take-Off and Landing) aircraft using an ArUco marker placed on the ground. The aircraft can search for the marker within a defined area using a lawnmower search pattern and initiate landing once the marker is detected. The system uses a Raspberry Pi for computer vision processing and communicates with the flight controller via the MAVLink protocol.

## **Features**

- **Autonomous Search Pattern**: The VTOL performs a systematic search for the ArUco marker using a lawnmower (zig-zag) pattern within a 100-meter radius.
- **ArUco Marker Detection**: The system uses OpenCV and its ArUco module to detect and estimate the pose of the ArUco marker.
- **MAVLink Communication**: The Raspberry Pi communicates with the flight controller using MAVLink, sending commands to adjust the aircraft’s flight path and land.
- **State Management**: The system operates in different states such as "searching," "tracking," and "landing," depending on the detection status of the marker.

## **Project Structure**

```
VTOL_Auto_Landing/
├── auto_land_sequence.py        # Main Python script for the auto landing sequence
├── camera_calibration/          # Contains camera calibration data
│   ├── camera_matrix.npy        # Camera intrinsic parameters
│   └── dist_coeffs.npy          # Camera distortion coefficients
├── README.md                    # Detailed project documentation
├── LICENSE                      # Project license
└── .gitignore                   # Specifies which files Git should ignore
```

## **Hardware Requirements**

To set up the VTOL Auto Landing system, you will need:

- **VTOL Aircraft**: Tilt-rotor VTOL with a compatible flight controller running ArduPlane firmware.
- **Flight Controller**: Any flight controller that supports MAVLink communication (e.g., Pixhawk).
- **Raspberry Pi**: Acts as a companion computer for computer vision processing (Raspberry Pi 3/4 recommended).
- **Camera Module**: Downward-facing Raspberry Pi camera module or equivalent USB camera.
- **Telemetry Link**: UART connection between the Raspberry Pi and the flight controller for MAVLink communication.
- **Landing Target**: A printed ArUco marker that the VTOL can detect and land on.
- **Power Supply**: Ensure adequate power for both the Raspberry Pi and the camera module during operation.

## **Software Requirements**

- **Operating System**: Raspberry Pi OS (or any other compatible Linux distribution).
- **Python 3.x**: The script is written in Python 3.
- **OpenCV**: With the `opencv-contrib-python` package for ArUco marker detection.
- **pymavlink**: For MAVLink communication between the Raspberry Pi and the flight controller.
  
### Install Dependencies

1. **Update Raspberry Pi and Install Required Packages**:

   ```bash
   sudo apt-get update
   sudo apt-get upgrade
   sudo apt-get install python3-opencv python3-pip
   ```

2. **Install OpenCV with ArUco Module**:

   ```bash
   pip3 install opencv-contrib-python
   ```

3. **Install pymavlink**:

   ```bash
   pip3 install pymavlink
   ```

4. **Clone the Project Repository**:

   ```bash
   git clone https://github.com/yourusername/VTOL_Auto_Landing.git
   cd VTOL_Auto_Landing
   ```

## **Usage**

### 1. **Configure and Calibrate Your Camera**

You need to calibrate your camera to correct for lens distortion. The calibration files (`camera_matrix.npy` and `dist_coeffs.npy`) should be placed in the `camera_calibration/` directory.

If you haven't calibrated your camera yet, you can follow the OpenCV camera calibration tutorial: [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

### 2. **Connect the Raspberry Pi to the Flight Controller**

- **Wiring**: Connect the Raspberry Pi to the flight controller via a UART interface (using TX/RX pins).
- **Configure MAVLink**: Ensure that MAVLink is enabled on the corresponding serial port on the flight controller.

### 3. **Running the Auto Landing Script**

After all the hardware and software are set up, you can run the auto landing script.

1. **Navigate to the Project Directory**:

   ```bash
   cd VTOL_Auto_Landing
   ```

2. **Execute the Script**:

   ```bash
   sudo python3 auto_land_sequence.py
   ```

3. **Monitor the Output**:

   - The script outputs status messages, such as "Marker detected" or "Searching for marker."
   - A window displaying the camera feed will appear, showing the detected marker and the aircraft's position in relation to it.

4. **Ending the Script**:

   - Press 'q' in the camera display window to quit the program.
   - Alternatively, press `Ctrl+C` in the terminal to interrupt the script.

## **How It Works**

### **Search Pattern**
The aircraft initially takes off to a predefined altitude (e.g., 5 meters) and performs a systematic **lawnmower search pattern** within a 100-meter radius. It moves back and forth in a zig-zag pattern, attempting to detect the ArUco marker from its camera feed.

### **Marker Detection**
When the aircraft’s downward-facing camera detects the ArUco marker, OpenCV computes the marker's position and orientation relative to the camera. The flight controller receives this information via the Raspberry Pi and adjusts the aircraft’s position to align it above the marker.

### **Landing Sequence**
Once the marker is detected, the aircraft transitions into a **landing mode**. The Raspberry Pi sends commands via MAVLink to control the descent, and the aircraft lands on the marker.

### **Failsafe: Return to Home**
If the marker is not detected after completing the entire search pattern, the aircraft returns to the **home position** (the GPS coordinates from where it took off) and performs an automatic landing.

## **Important Parameters**

- **MARKER_ID**: The ID number of the ArUco marker to detect.
- **MARKER_SIZE**: The real-world size of the ArUco marker (in meters).
- **SEARCH_ALTITUDE**: Altitude at which the aircraft conducts the search pattern.
- **MAX_RADIUS**: The maximum radius (in meters) within which the aircraft searches for the marker.

These parameters can be adjusted within the `auto_land_sequence.py` script based on your specific requirements.

## **Safety Considerations**

- **Obstacle Avoidance**: Ensure the search area is free of obstacles (trees, buildings, etc.).
- **Manual Control**: Always be prepared to take manual control of the aircraft if needed. Test the system in a controlled environment first.
- **Regulatory Compliance**: Ensure that your operation complies with local drone laws and regulations, especially when flying autonomously.
- **Failsafe Mechanisms**: Implement a failsafe to handle communication loss or marker detection failure (i.e., return to home).

## **Contributing**

Feel free to submit issues or pull requests to improve the project. Contributions are welcome!

## **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## **Acknowledgments**

- **OpenCV**: Used for image processing and ArUco marker detection.
- **ArduPilot**: For providing robust flight control software.
- **pymavlink**: For enabling communication between the Raspberry Pi and the flight controller.

---

### **Frequently Asked Questions (FAQ)**

**1. How large should the ArUco marker be?**

The marker size depends on the altitude at which the aircraft will be searching for it. For this project, an 8x8 ft (2.4-meter) marker is used.

**2. Can I use a different type of marker (e.g., AprilTag)?**

Yes, but you will need to modify the marker detection logic. OpenCV also supports AprilTags, or you can use other libraries like `apriltag-python`.

**3. What if the marker is not detected?**

If the marker is not detected, the VTOL will complete the search pattern and return to its home position for an automatic landing.

---

By following this detailed documentation, you should be able to successfully run the autonomous landing system on your VTOL aircraft.

If you encounter any issues, please consult the code comments, read through the FAQ section, or submit an issue to the GitHub repository.

---

You can now copy this **README.md** content into your project folder and push it to GitHub following the steps I provided earlier.

Let me know if you need further assistance!