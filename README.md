# BNO055 IMU Sensor Package

This package is part of the **CARVER Autonomous Robot** project - an autonomous Ackermann steering mobile robot. For more information about the complete robot system, visit [CARVER-NEXT-GEN on GitHub](https://github.com/CARVER-NEXT-GEN).

## Table of Contents
- [Overview](#overview)
- [Critical Configuration](#️-critical-absolute-orientation-configuration)
- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Dependencies](#dependencies)
- [Hardware Setup](#hardware-setup)
- [Software Architecture](#software-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Calibration](#calibration)
- [Operation Modes](#operation-modes)
- [Code Examples](#code-examples)
- [Troubleshooting](#troubleshooting)
- [Reference](#references)
- [Feedback](#feedback)

## Overview

This package provides two implementations for the [Bosch BNO055 9-DOF Absolute Orientation IMU](https://www.adafruit.com/product/2472):

1. **Standalone Calibration Version** - For testing and calibration procedures
2. **micro-ROS Publisher Version** - For real-time ROS2 integration

Both versions utilize STM32 microcontrollers with DMA-based I2C communication for high-speed data acquisition at 100Hz.

## ⚠️ CRITICAL: Absolute Orientation Configuration

**The sensor will NOT provide absolute orientation without proper calibration loading!**

Without magnetometer calibration, the sensor only provides relative orientation from power-on position. To get absolute orientation (where 0° = magnetic north):

```c
// Initialize sensor
BNO055_Init(&bno, &hi2c1, 0);

// CRITICAL: Load calibration for absolute orientation
BNO055_LoadCalibration(&bno, &saved_calib);  // ← MUST BE CALLED

// Optional: Set axis remapping for your mounting
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, AXIS_REMAP_SIGN_P1);
```

⚠️ **Ensure magnetometer calibration status ≥ 2 for valid absolute heading.**

## Features

- ✅ Complete 9-DOF sensor fusion using BNO055's internal processor
- ✅ High-speed data acquisition via DMA-based I2C communication
- ✅ Absolute orientation with proper calibration loading
- ✅ Built-in calibration system with persistent offset storage
- ✅ Real-time publishing at 100Hz via micro-ROS (ROS2 version)
- ✅ Quaternion, Euler angles, and raw sensor data output
- ✅ Configurable axis remapping for any mounting orientation
- ✅ Watchdog timer for system reliability (ROS2 version)
- ✅ Interactive calibration mode with LED feedback (Standalone version)
- ✅ Automatic error recovery and soft reset capability

## Hardware Overview

The BNO055 is a System in Package (SiP) integrating:

- **3-axis 14-bit accelerometer** - ±2g to ±16g range
- **3-axis 16-bit gyroscope** - ±125°/s to ±2000°/s range
- **3-axis magnetometer** - ±1300μT (x/y), ±2500μT (z)
- **32-bit ARM Cortex M0+** microcontroller
- **Fusion algorithms** providing absolute orientation

### Key Specifications

| Parameter | Value |
|-----------|--------|
| **Voltage** | 3.3V (5V tolerant I2C) |
| **Current** | 12.3mA @ 3.3V |
| **Communication** | I2C (up to 400kHz) |
| **Update Rate** | Up to 100Hz |
| **Temperature Range** | -40°C to +85°C |
| **Heading Accuracy** | ±2.5° |
| **Operating Modes** | Multiple fusion and non-fusion modes |

## Dependencies

### Common Dependencies
- STM32CubeIDE (version 1.13.0 or later)
- STM32 HAL Library
- FreeRTOS (for ROS2 version)
- ST-Link debugger or compatible programmer

### For micro-ROS Version
- micro-ROS for STM32
- ROS2 (tested on Humble)
- micro-ros-agent
- std_msgs package

## Hardware Setup

### Wiring Connections

Connect the BNO055 to your STM32 board:

| BNO055 Pin | STM32 Pin | Description |
|------------|-----------|-------------|
| VIN        | 3.3V      | Power supply |
| GND        | GND       | Ground |
| SDA        | PB9       | I2C Data (with 4.7kΩ pull-up) |
| SCL        | PB8       | I2C Clock (with 4.7kΩ pull-up) |
| RST        | Optional  | Reset (connect to GPIO for software reset) |
| ADR        | GND/3.3V  | I2C Address selection |

### I2C Address Configuration

- **Default address:** 0x28 (ADR pin LOW or floating)
- **Alternative:** 0x29 (ADR pin HIGH)

### Pull-up Resistors

⚠️ **Important:** Ensure proper I2C pull-up resistors (4.7kΩ) are installed on SDA and SCL lines for reliable communication.

## Software Architecture

### Project Structure

```
bno055_imu_sensor/
├── Core/
│   ├── Inc/
│   │   ├── BNO055.h          # Driver header file
│   │   └── main.h
│   └── Src/
│       ├── BNO055.c          # Driver implementation
│       └── main.c            # Application code
├── Middlewares/              # (ROS2 version only)
│   └── microros/
├── .ioc                      # STM32CubeMX configuration
└── README.md
```

### Core Components

1. **BNO055.h/c** - Simplified driver with DMA support and proper unit configuration
2. **main.c** - Application layer (calibration or micro-ROS publisher)
3. **DMA Configuration** - Optimized for continuous 100Hz operation

### Data Structure

The published Float64MultiArray (ROS2 version) contains 16 elements:

| Index | Data | Unit |
|-------|------|------|
| [0-3] | Quaternion (x, y, z, w) | normalized |
| [4-6] | Acceleration (x, y, z) | m/s² |
| [7-9] | Angular Velocity (x, y, z) | rad/s |
| [10-12] | Magnetometer (x, y, z) | µT |
| [13-15] | Euler Angles (roll, pitch, yaw) | radians |

### Data Units Configuration

The sensor is configured with UNIT_SEL register (0x3B) = 0x04:

- **Euler angles:** radians (1 rad = 900 LSB)
- **Gyroscope:** rad/s (1 rad/s = 900 LSB)
- **Accelerometer:** m/s² (1 m/s² = 100 LSB)
- **Magnetometer:** µT (1 µT = 16 LSB)
- **Temperature:** °C

## Installation

### Prerequisites

1. Install STM32CubeIDE from [ST website](https://www.st.com/en/development-tools/stm32cubeide.html)
2. For ROS2 version, set up micro-ROS following [official guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

### Building the Firmware

1. **Clone the repository:**
```bash
cd ~/STM32CubeIDE/workspace
git clone https://github.com/CARVER-NEXT-GEN/bno055_imu_sensor.git
```

2. **Import project in STM32CubeIDE:**
   - File → Import → Existing Projects into Workspace
   - Select the cloned repository folder
   - Click Finish

3. **Select implementation version:**
   - For calibration: Use `main_calibration.c`
   - For ROS2: Use `main_microros.c`
   - Rename the appropriate file to `main.c`

4. **Configure the build:**
   - Right-click on project → Properties
   - C/C++ Build → Settings → Tool Settings
   - Verify MCU settings match your STM32 model

5. **Build the project:**
```bash
# Clean build
Right-click on project → Clean Project
# Build
Right-click on project → Build Project
```

6. **Flash the firmware:**
   - Connect STM32 board via ST-Link
   - Run → Debug (F11)
   - The firmware will be flashed automatically

## Usage

### Standalone Calibration Version

1. **Power on the device**
2. **Press the USER button (PC13)** to start calibration
3. **Follow LED patterns:**
   - 1 blink: Place flat, Z-up
   - 2 blinks: Place flat, Z-down
   - 3 blinks: Place on left side
   - 4 blinks: Place on right side
   - 5 blinks: Place front up
   - 6 blinks: Place back down
   - Fast blinking: Draw figure-8 patterns
   - Solid ON: Calibration complete

4. **Read calibration values** from the serial output or debugger

### micro-ROS Publisher Version

1. **Start the micro-ROS agent:**
```bash
# Serial connection (USB)
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0

# For network connection
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

2. **Verify connection:**
```bash
# List topics
ros2 topic list

# You should see:
# /bno055_data
```

3. **Monitor the data:**
```bash
# View raw data
ros2 topic echo /bno055_data

# Check publishing rate (should be ~100Hz)
ros2 topic hz /bno055_data

# View data with formatting
ros2 topic echo /bno055_data --no-arr
```

## Calibration

### Understanding Calibration Levels

| Level | Status | Description |
|-------|--------|-------------|
| 0 | Not calibrated | Sensor not calibrated |
| 1 | Low | Minimal calibration |
| 2 | Medium | **Minimum for magnetometer absolute heading** |
| 3 | Fully calibrated | Optimal performance |

### Interactive Calibration Procedure

1. **Gyroscope Calibration:**
   - Keep the sensor completely stationary for 2-3 seconds
   - The gyroscope will auto-calibrate

2. **Accelerometer Calibration:**
   - Hold the sensor in 6 positions for 3 seconds each:
     - +X up, -X up, +Y up, -Y up, +Z up, -Z up
   - Move smoothly between positions

3. **Magnetometer Calibration:**
   - Draw large figure-8 patterns in all three dimensions
   - Continue until magnetometer status reaches level 3
   - This is critical for absolute heading

### Saving Calibration Data

After successful calibration, save the offsets:

```c
// Calibration data structure
const calibration_data_t saved_calib = {
    .accel_offset_x = 8,      // Your values here
    .accel_offset_y = 17,
    .accel_offset_z = -17,
    .mag_offset_x = -396,     // Critical for absolute heading
    .mag_offset_y = 179,      // Critical for absolute heading
    .mag_offset_z = -221,     // Critical for absolute heading
    .gyro_offset_x = -2,
    .gyro_offset_y = 1,
    .gyro_offset_z = 0,
    .accel_radius = 1000,
    .mag_radius = 1207
};
```

### Loading Calibration on Startup

```c
// In main() after BNO055_Init()
BNO055_LoadCalibration(&bno, &saved_calib);
```

## Operation Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **NDOF** | 9-DOF fusion (recommended) | Absolute orientation |
| **IMU** | Accel + Gyro fusion | Relative orientation only |
| **COMPASS** | Accel + Mag fusion | Compass heading |
| **M4G** | Mag for orientation | Magnetic field detection |
| **ACCONLY** | Accelerometer only | Acceleration/tilt |
| **MAGONLY** | Magnetometer only | Magnetic field |
| **GYROONLY** | Gyroscope only | Angular velocity |

## Code Examples

### Basic Initialization

```c
// Declare sensor structure
BNO055_t bno;

// Initialize with saved calibration
const calibration_data_t saved_calib = {
    // Your calibration values
};

// Initialize sensor
if (BNO055_Init(&bno, &hi2c1, 0) != HAL_OK) {
    Error_Handler();
}

// Load calibration (CRITICAL for absolute orientation!)
BNO055_LoadCalibration(&bno, &saved_calib);

// Set axis remapping if needed
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, AXIS_REMAP_SIGN_P1);
```

### Reading Sensor Data (Polling)

```c
// Update all sensor data
BNO055_Update(&bno);

// Access the data
float heading = bno.euler.yaw;
float pitch = bno.euler.pitch;
float roll = bno.euler.roll;

// Quaternion for advanced users
float qw = bno.quat.w;
float qx = bno.quat.x;
float qy = bno.quat.y;
float qz = bno.quat.z;
```

### DMA-Based Reading (High Performance)

```c
// In timer interrupt (100Hz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        if (bno.dma_ready) {
            BNO055_UpdateDMA(&bno);  // Start DMA transfer
        }
    }
}

// In I2C DMA complete callback
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        BNO055_ProcessDMA(&bno);  // Process received data
    }
}
```

### Checking Calibration Status

```c
// Get current calibration status
BNO055_GetCalibrationStatus(&bno);

// Check individual sensors
if (bno.calib_status.system >= 2 && 
    bno.calib_status.mag >= 2) {
    // Sensor is ready for absolute orientation
}

// Quick check
if (BNO055_IsCalibrated(&bno)) {
    // All sensors calibrated
}
```

## Troubleshooting

### Common Issues and Solutions

#### 1. No Absolute Orientation (Heading Drifts)

**Symptoms:** Yaw/heading value drifts over time

**Solutions:**
- Ensure `BNO055_LoadCalibration()` is called after initialization
- Verify magnetometer calibration status is ≥ 2
- Check for magnetic interference from motors/electronics
- Recalibrate in the operating environment

#### 2. All Data Reads Zero

**Symptoms:** All sensor values return 0.0

**Solutions:**
- Check I2C connections and pull-up resistors
- Verify correct I2C address (0x28 or 0x29)
- Ensure proper power supply (3.3V)
- Check variable naming (use lowercase `bno` not `BNO055`)

#### 3. Incorrect Axis Orientation

**Symptoms:** Axes don't match expected coordinate system

**Solutions:**
```c
// For standard RHR (X=forward, Y=left, Z=up)
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, 0x05);

// For custom mounting, experiment with configurations
// See axis remapping table in documentation
```

#### 4. Low Update Rate

**Symptoms:** Data updates slower than expected

**Solutions:**
- Enable DMA for I2C transfers
- Increase I2C clock to 400kHz
- Check timer configuration (should trigger at 100Hz)
- Reduce other processing in main loop

#### 5. I2C Communication Errors

**Symptoms:** HAL_ERROR or HAL_TIMEOUT returns

**Solutions:**
- Add 4.7kΩ pull-up resistors on SDA and SCL
- Reduce I2C speed if cable is long
- Check for proper grounding
- Enable automatic soft reset on errors

### LED Status Indicators

| Pattern | Meaning |
|---------|---------|
| Toggle every 500ms | Normal operation |
| Rapid blinking | Calibration mode active |
| Solid ON | Calibration complete or error |
| OFF | No power or initialization failed |

## API Reference

### Initialization Functions

```c
// Initialize sensor with I2C interface
HAL_StatusTypeDef BNO055_Init(BNO055_t *bno, I2C_HandleTypeDef *i2c, uint8_t addr);

// Set operation mode
HAL_StatusTypeDef BNO055_SetMode(BNO055_t *bno, uint8_t mode);

// Configure axis remapping
HAL_StatusTypeDef BNO055_SetAxisRemap(BNO055_t *bno, axis_remap_config_t config, axis_remap_sign_t sign);
```

### Calibration Functions

```c
// Load calibration offsets
HAL_StatusTypeDef BNO055_LoadCalibration(BNO055_t *bno, const calibration_data_t *calib);

// Get current calibration offsets
HAL_StatusTypeDef BNO055_GetCalibration(BNO055_t *bno, calibration_data_t *calib);

// Get calibration status
HAL_StatusTypeDef BNO055_GetCalibrationStatus(BNO055_t *bno);

// Check if fully calibrated
bool BNO055_IsCalibrated(BNO055_t *bno);
```

### Data Acquisition Functions

```c
// Update all sensor data (blocking)
HAL_StatusTypeDef BNO055_Update(BNO055_t *bno);

// Start DMA transfer (non-blocking)
HAL_StatusTypeDef BNO055_UpdateDMA(BNO055_t *bno);

// Process DMA buffer
void BNO055_ProcessDMA(BNO055_t *bno);
```

### Utility Functions

```c
// Check if sensor is responding
bool BNO055_IsResponding(BNO055_t *bno);

// Perform soft reset
HAL_StatusTypeDef BNO055_SoftReset(BNO055_t *bno);
```

## Data Conversion Reference

| Data Type | Register Format | Conversion Formula |
|-----------|-----------------|-------------------|
| Euler angles | 16-bit signed | raw / 900.0 = radians |
| Gyroscope | 16-bit signed | raw / 900.0 = rad/s |
| Accelerometer | 16-bit signed | raw / 100.0 = m/s² |
| Magnetometer | 16-bit signed | raw / 16.0 = µT |
| Quaternion | 16-bit signed | raw / 16384.0 = normalized |
| Temperature | 8-bit signed | raw = °C |

## Performance Metrics

| Metric | Value |
|--------|-------|
| Update Rate | 100Hz |
| I2C Speed | 400kHz |
| DMA Transfer Time | <1ms |
| Power Consumption | ~50mW |
| Startup Time | ~650ms |
| Calibration Time | 2-5 minutes |

## References

- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [micro-ROS Documentation](https://micro.ros.org/)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.
