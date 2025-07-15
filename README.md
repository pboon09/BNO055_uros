# BNO055 IMU Sensor Package

This package is part of the **CARVER Autonomous Robot** project - an autonomous Ackermann steering mobile robot. For more information about the complete robot system, visit [CARVER-NEXT-GEN on GitHub](https://github.com/CARVER-NEXT-GEN).

## Overview
This package provides a micro-ROS implementation for the [Bosch BNO055 9-DOF Absolute Orientation IMU](https://www.adafruit.com/product/2472). It publishes sensor data including quaternion orientation, linear acceleration, angular velocity, magnetometer readings, and Euler angles through a ROS2 Float64MultiArray message using an STM32 microcontroller with micro-ROS.

## Features
- Complete 9-DOF sensor fusion using BNO055's internal processor
- High-speed data acquisition via DMA-based I2C communication
- Configurable sensor operation modes (NDOF, IMU, COMPASS, etc.)
- Built-in calibration system with persistent offset storage
- Real-time publishing at 100Hz via micro-ROS
- Quaternion, Euler angles, and raw sensor data output
- Automatic axis remapping support
- Watchdog timer for system reliability

## Hardware Overview
The BNO055 is a System in Package (SiP) integrating:
- 3-axis 14-bit accelerometer
- 3-axis 16-bit gyroscope (±2000°/s)
- 3-axis magnetometer
- 32-bit ARM Cortex M0+ microcontroller
- Fusion algorithms providing absolute orientation

### Key Specifications
- Voltage: 3.3V (5V tolerant I2C)
- Current: 12.3mA @ 3.3V
- Communication: I2C (up to 400kHz)
- Update Rate: Up to 100Hz
- Operating Modes: Multiple fusion and non-fusion modes

## Dependencies
- STM32CubeIDE
- micro-ROS for STM32
- ROS2 (tested on Humble)
- micro-ros-agent
- FreeRTOS

## Hardware Setup
### Wiring Connections
Connect the BNO055 to your STM32 board:
| BNO055 Pin | STM32 Pin | Description |
|------------|-----------|-------------|
| VIN | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | PB9 | I2C Data |
| SCL | PB8 | I2C Clock |
| RST | PA0 | Reset |

### I2C Address Configuration
- Default address: 0x28 (ADR pin LOW or floating)
- Alternative: 0x29 (ADR pin HIGH)

## Software Architecture
The implementation consists of several key components:

### Core Files
- `BNO055.h/c`: Main driver implementation with I2C communication
- `BNO055_config.h/c`: Calibration offsets and configuration
- `main.c`: micro-ROS publisher and FreeRTOS task management

### Data Structure
The published Float64MultiArray contains 19 elements:
```
[0-3]   : Quaternion (x, y, z, w)
[4-6]   : Acceleration (x, y, z) in m/s²
[7-9]   : Linear Acceleration (x, y, z) in m/s²
[10-12] : Angular Velocity (x, y, z) in rad/s
[13-15] : Magnetometer (x, y, z) in μT
[16-18] : Euler Angles (roll, pitch, yaw) in radians
```

## Installation
### Prerequisites
- STM32CubeIDE (version 1.13.0 or later)
- micro-ROS setup for STM32
- ST-Link debugger or compatible programmer

### Building the STM32 Firmware
1. Clone this repository into your STM32CubeIDE workspace
```bash
cd ~/STM32CubeIDE/workspace
git clone https://github.com/CARVER-NEXT-GEN/bno055_imu_sensor.git
```
2. Open STM32CubeIDE and import the project:
   - File → Import → Existing Projects into Workspace
   - Select the cloned repository folder
   - Click Finish

3. Clean and build the project:
   - Right-click on the project → Clean Project
   - Right-click on the project → Build Project

4. Configure the debugger:
   - Click on the project in Project Explorer
   - Run → Debug Configurations
   - Double-click on STM32 C/C++ Application
   - Configure your ST-Link settings
   - Apply and Close

5. Flash the firmware:
   - Connect your STM32 board via ST-Link
   - Run → Debug (or press F11)
   - The firmware will be flashed automatically

## Usage
### Starting the micro-ROS Agent
To establish communication between the STM32 and ROS2:
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0
```

### Verifying Data Publication
Check that the IMU data is being published:
```bash
# List active topics
ros2 topic list

# Echo the IMU data
ros2 topic echo /bno055_data

# Check publishing rate
ros2 topic hz /bno055_data
```

### Parameters
The firmware supports several compile-time configurations in `BNO055_config.h`:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| BNO_CALIB_OFF/ON | Macro | OFF | Enable/disable calibration mode |
| BNO_ACC_OFF_X/Y/Z | Integer | 8, 17, -17 | Accelerometer offsets |
| BNO_MAG_OFF_X/Y/Z | Integer | -396, 179, -221 | Magnetometer offsets |
| BNO_GYRO_OFF_X/Y/Z | Integer | -2, 1, 0 | Gyroscope offsets |
| BNO_ACC_RAD | Integer | 1000 | Accelerometer radius |
| BNO_MAG_RAD | Integer | 1207 | Magnetometer radius |

## Calibration
The BNO055 features an automatic calibration system. The calibration status for each sensor can be monitored:
- 0: Not calibrated
- 1: Low calibration
- 2: Medium calibration
- 3: Fully calibrated

### Calibration Procedure
1. Enable calibration mode by defining `BNO_CALIB_ON` in `BNO055_config.h`
2. Move the sensor through various orientations:
   - Gyroscope: Keep stationary for 2-3 seconds
   - Accelerometer: Rotate slowly through 6 positions (±X, ±Y, ±Z)
   - Magnetometer: Draw figure-8 patterns in the air
3. Once calibrated, the offsets are automatically saved and can be hardcoded into `BNO055_config.h`

## Operation Modes
The sensor supports multiple operation modes configured in the initialization:
- **NDOF** (default): 9-DOF sensor fusion
- **IMU**: Accelerometer + Gyroscope fusion
- **COMPASS**: Accelerometer + Magnetometer fusion
- **M4G**: Magnetometer for orientation
- **ACCONLY**: Accelerometer only
- **MAGONLY**: Magnetometer only
- **GYROONLY**: Gyroscope only

## Code Implementation
### Initialization Example
In your `main.c` file, the BNO055 is initialized as follows:

```c
// In USER CODE BEGIN Includes
#include "BNO055.h"
#include "BNO055_config.h"

// In USER CODE BEGIN PV
BNO055_t BNO055;

// In StartDefaultTask
BNO055_Init(&BNO055, &hi2c1, 0, NDOF);
#ifdef BNO_CALIB_ON
BNO055_Calibrated(&BNO055, &BNO055_stat, &BNO055_off);
#endif
BNO055_SetOffsets(&BNO055, &BNO055_off);
BNO055_SetAxis(&BNO055, P0_Config, P1_Sign);
```

### Reading Sensor Data
The sensor data is read using DMA in the timer interrupt callback:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        if (BNO055.flag == HAL_OK) {
            BNO055_Read_DMA(&BNO055, 0);  // 0 = read all data
            BNO055.flag = HAL_BUSY;
        }
    }
}
```

## Troubleshooting
### Common Issues
1. **No data published**
   - Check I2C connections and pull-up resistors (typically 4.7kΩ)
   - Verify the correct I2C address (0x28 or 0x29)
   - Ensure micro-ROS agent is running with correct baudrate

2. **Erratic sensor readings**
   - Perform sensor calibration
   - Keep sensor away from magnetic interference
   - Check power supply stability

3. **Low update rate**
   - Verify DMA is properly configured
   - Check I2C clock speed (up to 400kHz supported)
   - Ensure timer interrupt frequency matches desired rate

### LED Indicators
- LED toggling indicates active data publishing
- Steady LED may indicate a communication issue

## Advanced Configuration
### Axis Remapping
The sensor axes can be remapped using predefined configurations:
```c
BNO055_SetAxis(&BNO055, P0_Config, P1_Sign);
```

### Power Modes
Configure power consumption vs. performance:
- Normal Mode: Full performance
- Low Power Mode: Reduced power, lower update rate
- Suspend Mode: Minimal power consumption

## Sensor Coordinate System
The BNO055 uses a right-handed coordinate system with the following orientation (when in default axis configuration):
- **X-axis**: Points to the right when viewing the chip from above
- **Y-axis**: Points forward 
- **Z-axis**: Points upward

### Euler Angle Conventions
The BNO055 outputs orientation data in Euler angles with the following conventions (Table 3-13 from datasheet):

**Android Format (default):**
- **Pitch**: +180° to -180° (turning clockwise decreases values)
- **Roll**: -90° to +90° (increasing with increasing inclination)
- **Heading/Yaw**: 0° to 360° (turning clockwise increases values)

**Windows Format:**
- **Pitch**: -180° to +180° (turning clockwise increases values)
- **Roll**: -90° to +90° (increasing with increasing inclination)
- **Heading/Yaw**: 0° to 360° (turning clockwise increases values)

The format can be selected using the UNIT_SEL register (bit 7). In your implementation, the default format is Android (ORI_Android_Windows = 1).

## References
- [BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- [micro-ROS Documentation](https://micro.ros.org/)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-packages.html)

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.
