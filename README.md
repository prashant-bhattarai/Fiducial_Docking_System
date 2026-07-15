Fiducial Docking & 3D Spatial Perception with ArUco Markers

Hey! This is a C++ computer vision project I built to understand how Autonomous Mobile Robots (AMRs), like the drive units used at Amazon, calculate their exact 3D position relative to a target.

Instead of just detecting 2D pixels, this pipeline calculates the exact real-world Distance (X, Y, Z) and Rotation (Pitch, Yaw, Roll) of a target, and uses a physics-based filter to smooth the data for robotic control.

How it Works

I broke this project down into three main technical challenges:

The Math (Perspective-n-Point): I used OpenCV's solvePnP algorithm. By defining the physical size of a printed marker in meters, and using a pre-calibrated camera matrix (Intrinsic parameters), the math translates 2D pixel corners into a 4x4 Extrinsic Matrix, giving me the exact 6-DoF pose of the target.

Multi-Marker Robustness: A robot goes blind if a single marker is covered by dirt or a shadow. I upgraded the code to track an ArucoBoard (a grid of markers). By treating the grid as a single rigid body, the math uses dozens of corners simultaneously. I can physically cover half the board with my hand, and the 3D origin stays perfectly locked in the center.

The Kalman Filter: Raw camera pixels are noisy, which causes the 3D distance calculations to "jitter" slightly. If fed directly to robot motors, the robot would vibrate. I built a custom 6-state Kalman Filter that predicts the box's velocity using basic kinematics, and fuses that prediction with the noisy camera measurements. The result is perfectly smooth 3D tracking.

Getting Started

If you want to run this locally:

Build the project using CMake (requires OpenCV).

Run Calibrator.exe with a printed checkerboard to generate your specific webcam's camera_calibration.xml file.

Run GridTracker.exe and hold up a printed ArUco dictionary grid to see the filtered 3D axes drawn in real-time.
