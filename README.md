# BNO055 IMU Sensor Package

This package is part of the **CARVER Autonomous Robot** project - an autonomous Ackermann steering mobile robot. For more information about the complete robot system, visit [CARVER-NEXT-GEN on GitHub](https://github.com/CARVER-NEXT-GEN).

## Overview
This package provides a micro-ROS implementation for the [Bosch BNO055 9-DOF Absolute Orientation IMU](https://www.adafruit.com/product/2472). It publishes sensor data including quaternion orientation, acceleration, angular velocity, magnetometer readings, and Euler angles through a ROS2 Float64MultiArray message using an STM32 microcontroller with micro-ROS.

## ⚠️ CRITICAL: Absolute Orientation Configuration
**The sensor will NOT provide absolute orientation without proper calibration loading!**

Without magnetometer calibration, the sensor only provides relative orientation from power-on position. To get absolute orientation (where 0° = magnetic north):

```c
// Initialize sensor
BNO055_Init(&bno, &hi2c1, 0);

// CRITICAL: Load calibration for absolute orientation
BNO055_LoadCalibration(&bno, &saved_calib);  // ← MUST BE CALLED

// Optional: Set axis remapping for your mounting
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, 0x06);
```

Ensure magnetometer calibration status ≥ 2 for valid absolute heading.

## Features

- Complete 9-DOF sensor fusion using BNO055's internal processor
- High-speed data acquisition via DMA-based I2C communication
- Absolute orientation with proper calibration loading
- Built-in calibration system with persistent offset storage
- Real-time publishing at 100Hz via micro-ROS
- Quaternion, Euler angles, and raw sensor data output
- Configurable axis remapping for any mounting orientation
- Watchdog timer for system reliability

## Hardware Overview
The BNO055 is a System in Package (SiP) integrating:

- 3-axis 14-bit accelerometer
- 3-axis 16-bit gyroscope (±2000°/s)
- 3-axis magnetometer
- 32-bit ARM Cortex M0+ microcontroller
- Fusion algorithms providing absolute orientation

### Key Specifications

- **Voltage:** 3.3V (5V tolerant I2C)
- **Current:** 12.3mA @ 3.3V
- **Communication:** I2C (up to 400kHz)
- **Update Rate:** Up to 100Hz
- **Operating Modes:** Multiple fusion and non-fusion modes

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
| VIN        | 3.3V      | Power supply |
| GND        | GND       | Ground |
| SDA        | PB9       | I2C Data |
| SCL        | PB8       | I2C Clock |

### I2C Address Configuration

- **Default address:** 0x28 (ADR pin LOW or floating)
- **Alternative:** 0x29 (ADR pin HIGH)

## Software Architecture
The implementation consists of several key components:

### Core Files

- **BNO055.h/c:** Simplified driver with DMA support and proper unit configuration
- **main.c:** micro-ROS publisher and FreeRTOS task management

### Data Structure
The published Float64MultiArray contains 16 elements:

```
[0-3]   : Quaternion (x, y, z, w) - normalized
[4-6]   : Acceleration (x, y, z) in m/s²
[7-9]   : Angular Velocity (x, y, z) in rad/s
[10-12] : Magnetometer (x, y, z) in µT
[13-15] : Euler Angles (roll, pitch, yaw) in radians
```

### Data Units
The sensor is configured with UNIT_SEL register (0x3B) = 0x04:

- **Euler angles:** radians (1 rad = 900 LSB)
- **Gyroscope:** rad/s (1 rad/s = 900 LSB)
- **Accelerometer:** m/s² (1 m/s² = 100 LSB)
- **Magnetometer:** µT (1 µT = 16 LSB)

## Installation

### Prerequisites

- STM32CubeIDE (version 1.13.0 or later)
- micro-ROS setup for STM32
- ST-Link debugger or compatible programmer

### Building the STM32 Firmware

1. **Clone this repository into your STM32CubeIDE workspace**

```bash
cd ~/STM32CubeIDE/workspace
git clone https://github.com/CARVER-NEXT-GEN/bno055_imu_sensor.git
```

2. **Open STM32CubeIDE and import the project:**
   - File → Import → Existing Projects into Workspace
   - Select the cloned repository folder
   - Click Finish

3. **Clean and build the project:**
   - Right-click on the project → Clean Project
   - Right-click on the project → Build Project

4. **Configure the debugger:**
   - Click on the project in Project Explorer
   - Run → Debug Configurations
   - Double-click on STM32 C/C++ Application
   - Configure your ST-Link settings
   - Apply and Close

5. **Flash the firmware:**
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

# Check publishing rate (should be ~100Hz)
ros2 topic hz /bno055_data
```

## Calibration
The BNO055 features an automatic calibration system. The calibration status for each sensor can be monitored:

- **0:** Not calibrated
- **1:** Low calibration
- **2:** Medium calibration (minimum for magnetometer absolute heading)
- **3:** Fully calibrated

### ⚠️ Important: Calibration Persistence
Calibration offsets must be:

1. Obtained when sensor is fully calibrated
2. Saved to non-volatile memory
3. Loaded immediately after initialization
4. Applied before switching to NDOF mode

### Calibration Procedure

1. **Run the sensor without loading calibration**
2. **Move the sensor through various orientations:**
   - **Gyroscope:** Keep stationary for 2-3 seconds
   - **Accelerometer:** Rotate slowly through 6 positions (±X, ±Y, ±Z facing up)
   - **Magnetometer:** Draw figure-8 patterns in all three dimensions

3. **When all calibration levels reach 3, read the offsets using `BNO055_GetCalibration()`**
4. **Save these offsets in your code as `saved_calib` structure**

### Calibration Offsets Structure

```c
const calibration_data_t saved_calib = {
    .accel_offset_x = 8,
    .accel_offset_y = 17,
    .accel_offset_z = -17,
    .mag_offset_x = -396,    // Critical for absolute heading
    .mag_offset_y = 179,     // Critical for absolute heading
    .mag_offset_z = -221,    // Critical for absolute heading
    .gyro_offset_x = -2,
    .gyro_offset_y = 1,
    .gyro_offset_z = 0,
    .accel_radius = 1000,
    .mag_radius = 1207
};
```

## Operation Modes
The sensor is configured in NDOF mode (Nine Degrees of Freedom) for absolute orientation. Other available modes:

- **IMU:** Accelerometer + Gyroscope fusion (relative orientation only)
- **COMPASS:** Accelerometer + Magnetometer fusion
- **M4G:** Magnetometer for orientation
- **ACCONLY:** Accelerometer only
- **MAGONLY:** Magnetometer only
- **GYROONLY:** Gyroscope only

## Code Implementation

### Initialization Example (Correct)

```c
// In USER CODE BEGIN PV
BNO055_t bno;  // Note: lowercase variable name

const calibration_data_t saved_calib = {
    // Your calibration values here
};

// In main() after peripheral initialization
if (BNO055_Init(&bno, &hi2c1, 0) != HAL_OK) {
    Error_Handler();
}

// CRITICAL: Load calibration for absolute orientation
BNO055_LoadCalibration(&bno, &saved_calib);

// Optional: Set axis remapping for your mounting
// P1 config with P2 sign for specific mounting orientation
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, AXIS_REMAP_SIGN_P2);

// Start timer for DMA readings
HAL_TIM_Base_Start_IT(&htim2);
```

### Reading Sensor Data
The sensor data is read using DMA in the timer interrupt callback:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        if (bno.dma_ready) {
            BNO055_UpdateDMA(&bno);  // Start DMA transfer
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        BNO055_ProcessDMA(&bno);  // Process received data
    }
}
```

## Sensor Coordinate System

### Default P1 Configuration (with dot at bottom-left, chip facing up):

- **X-axis:** Points backward
- **Y-axis:** Points left
- **Z-axis:** Points down

### For Standard RHR (X=forward, Y=left, Z=up):

```c
BNO055_SetAxisRemap(&bno, AXIS_REMAP_P1, 0x05);  // Flips X and Z axes
```

### Axis Remapping Options

| Config | Mapping | Use Case |
|--------|---------|----------|
| P0 | X→Y, Y→X, Z→Z | 90° rotation |
| P1 | X→X, Y→Y, Z→Z | Default |
| P2 | X→X, Y→Y, Z→Z | Same as P1, different signs |
| P3-P7 | Various | Other mounting orientations |

## Troubleshooting

### Common Issues

1. **No absolute orientation (heading drifts)**
   - **Calibration not loaded:** Call `BNO055_LoadCalibration()` after init
   - **Magnetometer not calibrated:** Check `calib_status.mag >= 2`
   - **Wrong mode:** Ensure NDOF mode is set

2. **All data reads zero**
   - **Variable name mismatch:** Use lowercase `bno` not uppercase `BNO055`
   - **I2C communication failure:** Check connections and pull-up resistors
   - **Sensor not initialized properly**

3. **Incorrect axis orientation**
   - Adjust axis remapping based on physical mounting
   - Use `BNO055_SetAxisRemap()` with appropriate config and sign

4. **Low update rate**
   - DMA not configured properly
   - Timer frequency too low
   - I2C clock speed too slow (increase to 400kHz)

### LED Indicators

- **LED toggling every 500ms:** Normal operation
- **LED steady on/off:** Communication issue or error state

## Data Conversion Formulas

| Data Type | UNIT_SEL = 0x04 (Current) | Conversion |
|-----------|---------------------------|------------|
| Euler | radians | raw / 900.0 |
| Gyro | rad/s | raw / 900.0 |
| Accel | m/s² | raw / 100.0 |
| Mag | µT | raw / 16.0 |
| Quaternion | normalized | raw / 16384.0 |

## References

- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [micro-ROS Documentation](https://micro.ros.org/)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.
