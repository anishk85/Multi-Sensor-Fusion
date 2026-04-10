Raw Sensor Data Analysis & Fusion Readiness Report
Executive Summary

This report details the statistical and visual analysis of raw sensor data (GPS, IMU, and Wheel Odometry) collected from the mobile robot platform. The primary objective of this analysis is to extract accurate baseline noise profiles (covariance matrices) required to configure an Extended Kalman Filter (EKF) for robust sensor fusion.

The analysis successfully isolated the true Gaussian noise of the GPS and IMU sensors, calculated the necessary covariance matrices, and uncovered a critical physical mismatch between the software navigation stack and the hardware motor controllers that must be resolved prior to finalizing the fusion algorithm.
1. Global Positioning System (GPS) Analysis

Objective: To determine the baseline variance of the GPS module to populate the position measurement covariance matrix.

Findings:
The GPS data yielded excellent results, displaying perfectly symmetrical Gaussian (normal) distributions across all three axes. This is the ideal scenario for an EKF, which mathematically assumes standard Gaussian noise.

    Latitude Variance: 0.263

    Longitude Variance: 0.245

    Altitude Variance: 1.230 (Note: Expected behavior, as GPS altitude is typically significantly noisier than lateral position).

    [Insert Plot Here: GPS Noise Distribution (Gaussian Check) Histograms]

The "Random Walk" Phenomenon:
A spatial plot of the stationary GPS data revealed the classic "Random Walk" effect. Even when the robot was completely still, the GPS reported erratic movement within a roughly 1-meter radius due to atmospheric interference and signal multipath errors. This specific vulnerability completely justifies the need for an EKF; without fusing this data with stable wheel odometry, the robot's local planner would attempt to constantly correct for "ghost" movements.

    [Insert Plot Here: Odom vs GPS (Local XY, start-aligned) Spatial Plot]

2. Kinematic Analysis: Command vs. Odometry

Objective: To verify the accuracy of the robot's internal wheel odometry against the commanded velocities from the navigation stack.

Findings: Critical Hardware/Software Mismatch
Time-series analysis of the commanded velocity (cmd_vel) versus the measured odometry (odom) revealed a severe disconnect between what the software demands and what the hardware can physically execute.

    Linear Velocity Cap: The navigation stack commanded speeds up to 4.0 m/s, but the internal motor drivers aggressively capped the physical output at exactly 1.5 m/s.

    Angular Velocity Cap: The software requested aggressive turns exceeding 10.0 rad/s, but the physical chassis maxed out at approximately 2.0 rad/s.

    [Insert Plot Here: Linear Velocity and Angular Velocity: command vs odometry Time Series]

Impact on Sensor Fusion:
Because the EKF relies heavily on odometry to fill the gaps between GPS updates, this mismatch will cause massive Root Mean Square Errors (RMSE). If the "brain" expects the robot to have moved 40 meters based on its commands, but the wheels only moved 15 meters, the fusion algorithm will fail to converge.

Recommendation: The navigation controller's parameters (max_vel_x and max_vel_theta) must be hard-capped to match the physical 1.5 m/s limits of the hardware before tuning the EKF.
3. Inertial Measurement Unit (IMU) Analysis

Objective: To extract the baseline electrical noise (variance) of the gyroscopes and accelerometers while rejecting physical movement artifacts.

Phase 1: The Moving Dataset
Initial histograms of the IMU data showed widely scattered peaks rather than clean bell curves. Time-series analysis confirmed this data was captured while the robot was actively driving, meaning the actual physical bumps and turns were drowning out the baseline sensor noise. Furthermore, the IMU driver was found to be publishing empty (zeroed) covariance arrays, requiring manual calculation.

    [Insert Plot Here: IMU Angular Velocity & Linear Acceleration Time Series (Moving Dataset)]

Phase 2: The Stationary Ground Truth
A secondary dataset was captured with the robot perfectly stationary. This successfully isolated the thermal/electrical noise of the MEMS sensors, resulting in tight, highly accurate Gaussian distributions.

    Gravity Calibration: The Z-axis linear accelerometer accurately reported a mean of 9.85 m/s², confirming proper sensor orientation and gravity measurement.

    Non-Holonomic Considerations: Despite being a 2D ground robot, the IMU accurately reported 3D environmental noise (pitch, roll, and Z-axis vibrations). This will be handled by enabling two_d_mode in the EKF configuration to ignore irrelevant 3D axes.

    [Insert Plot Here: IMU Accel/Gyro Noise Distribution Histograms (Stationary Dataset)]

Calculated Covariance Matrices:
Based on the stationary data, the following matrices have been generated for direct insertion into the EKF configuration parameters:

Angular Velocity Covariance (Rgyro​):
​0.0002260.00.0​0.00.0014620.0​0.00.00.041601​​

Linear Acceleration Covariance (Raccel​):
​0.9696660.00.0​0.00.5653100.0​0.00.00.691300​​
Next Steps

Now that we have the report structured and the math solidified, the immediate next step is to actually write the configuration files for your robot.

Would you like me to draft the full ekf.yaml configuration file for robot_localization using these exact numbers, or would you prefer to start by fixing those max_vel limits in your navigation stack?