# Navigation-Stack-Development-and-GPS-IMU-Dead-Reckoning-Analysis

## Introduction
In robotics and other fields requiring accurate position and orientation data, navigation using an IMU and a magnetometer is a widely adopted technique. IMUs use accelerometers and gyroscopes to measure linear acceleration and angular velocity, respectively, while magnetometers measure the Earth's magnetic field direction. Combining data from these sensors allows for the estimation of the device's position and orientation in three dimensions.

## Project Overview
### Calibration of Magnetometer
Calibrating a magnetometer involves compensating for both hard iron and soft iron distortions:

Hard Iron Calibration: Detects and corrects continuous offset biases caused by nearby magnetic materials.
Soft Iron Calibration: Rectifies distortions in the magnetic field caused by the device or adjacent magnetic materials, using transformation matrices.

### Sensor Data Processing and Integration
#### Noise Reduction
Low-pass Filters: Eliminate high-frequency interference from sensor data while maintaining essential signals.
Dead Reckoning: Estimates linear and angular motion using accelerometer and gyroscope data. Critical for position estimation without external references.
Sensor Fusion with GPS: Combines GPS data with IMU and magnetometer data to improve navigation accuracy, correcting drift and long-term errors.

### Yaw Estimation
Combines accelerometer and gyroscope data with a complementary filter to estimate the yaw angle.
Uses low-pass filtering for accelerometer data and high-pass filtering for gyroscope data to eliminate noise and drift.

### Forward Velocity Estimation
Analyzes linear acceleration in the x-direction to determine the vehicle's forward acceleration.
Corrects biases in forward speed estimation by filtering out stationary data and directly setting certain data points to zero.

### Dead Reckoning with IMU
Compares paths measured by IMU and GPS, noting minor discrepancies due to different measurement origins and sensor limitations.

### Key Figures
Magnetometer Data Before and After Correction: Shows data before and after correcting hard iron and soft iron distortions.
Yaw Estimation: Displays the gyro-integrated yaw and calibrated yaw, highlighting drift and noise.
Forward Velocity Comparison: Compares IMU and GPS velocities, emphasizing the need for GPS correction.
Displacement and Acceleration Comparisons: Illustrates the paths and acceleration measurements from IMU and GPS.

