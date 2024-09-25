# Differential Swerve Drive Pathing Library

Welcome to the **Differential Swerve Drive Pathing Library**, designed specifically for 2-wheel differential swerve drive systems running on the REV Control Hub. This library provides pathing capabilities with built-in control systems and optional filtering for enhanced sensor fusion. It is ideal for teams and projects utilizing odometry and an IMU for precise movement and path execution.

---

## Features

- **Built-in PIDF Control**: Leverages the built-in PIDF (Proportional-Integral-Derivative with Feedforward) controller for accurate and smooth path following.
  
- **Optional Kalman Filter**: Includes optional support for Kalman filtering, which improves sensor data fusion and noise reduction. If no external position estimator is available, the Kalman filter functions like a low-pass filter.

- **Path Generation Options**:
  - **Splines**: Allows smooth, curving paths between waypoints for fluid movement.
  - **Straight Lines**: Supports simple, direct paths between two points.
  - **Turns**: Handles rotational movements, adjusting the robot's heading as needed.

- **3-Wheel Odometry Support**: Accurate positional tracking with 3-wheel odometry ensures reliable feedback during complex maneuvers.

- **IMU Integration**: Utilizes the REV Control Hub's built-in IMU for maintaining precise heading control throughout the robot's movement.

## System Requirements

### Hardware:
- 2-wheel differential swerve drive setup.
- REV Control Hub (Android-based device).
- 3-wheel odometry system.
- Inertial Measurement Unit (IMU) for heading control.

### Software:
- REV Robotics SDK.
- Android-based environment configured for REV Control Hub.

## Installation

1. Download or clone the library and integrate it into your project’s build environment (e.g., Android Studio).
2. Ensure the REV SDK is properly installed and configured for your REV Control Hub system.
3. Connect your 3-wheel odometry and IMU sensors to the Control Hub.

## Key Components

- **Differential Swerve Drive**: The core class that manages the swerve drive system, handling pathing, sensor input, and movement execution.
  
- **Pathing Support**: Build paths using splines, straight lines, and turns, and chain them together for more complex maneuvers. The library provides flexible path generation options.

- **Kalman Filter**: An optional filter that can be added to improve sensor fusion and data accuracy. If no external sensors are available, the Kalman filter serves as a low-pass filter for reducing noise from odometry and IMU inputs.

## Documentation

For further details on setting up, configuring, and using the library, please refer to the official [Coyote Beta Documentation](https://txkl.gitbook.io/coyote-beta). While the documentation is currently minimal, it contains essential information for getting started. Contributions to expand and improve the documentation are welcome.

## Contributing

Contributions to this project are welcome! Whether you have suggestions for new features, improvements to the library, or additional documentation, feel free to open an issue or submit a pull request.

## License

This project is licensed under the MIT License. Please review the `LICENSE` file for more details.
