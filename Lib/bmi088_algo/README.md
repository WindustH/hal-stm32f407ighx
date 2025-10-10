# BMI088 Algorithm Library

This library contains the algorithm components for the BMI088 IMU sensor, extracted from the RoboMaster C-Board INS Example project. It provides pure algorithm implementations without any hardware-specific communication code.

## Features

- Sensor data processing algorithms for accelerometer and gyroscope
- Calibration algorithms for sensor offset compensation
- Temperature compensation algorithms
- Configurable sensitivity settings
- Quaternion-based Extended Kalman Filter (EKF) for attitude estimation
- Gyro bias estimation and compensation

## Files

### Header Files (Inc/)
- `bmi088_algorithm.h` - Main header file with function declarations and data structures for basic sensor algorithms
- `bmi088_quaternion_ekf.h` - Header file with function declarations and data structures for quaternion EKF algorithms

### Source Files (Src/)
- `bmi088_algorithm.c` - Implementation of basic BMI088 algorithms
- `bmi088_quaternion_ekf.c` - Implementation of quaternion EKF algorithms for attitude estimation

## Usage

### Basic Sensor Algorithms

1. Include the header file in your project:
   ```c
   #include "bmi088_algorithm.h"
   ```

2. Initialize the IMU data structure:
   ```c
   IMU_Data_t imu_data;
   BMI088_Init_Algorithm(&imu_data, 1); // 1 for calibration, 0 for no calibration
   ```

3. Process sensor data:
   ```c
   // Assuming you have raw sensor data from hardware communication
   uint8_t accel_raw_data[6];
   uint8_t gyro_raw_data[8];
   uint8_t temp_raw_data[2];

   // Process the data using the algorithm
   BMI088_Read_Algorithm(&imu_data, accel_raw_data, gyro_raw_data, temp_raw_data);
   ```

4. For calibration:
   ```c
   // Perform calibration with a specified number of samples
   BMI088_Calibrate_Offset(&imu_data, 1000); // Use 1000 samples for calibration
   ```

### Quaternion EKF Algorithms

1. Include the header file in your project:
   ```c
   #include "bmi088_quaternion_ekf.h"
   ```

2. Initialize the Quaternion EKF:
   ```c
   // Initialize with process noise1, process noise2, measure noise, lambda, lpf
   IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996, 0);
   ```

3. Update the filter with sensor data:
   ```c
   // Update with gyro (rad/s) and accel (m/s²) data
   IMU_QuaternionEKF_Update(gx, gy, gz, ax, ay, az, dt);
   ```

## Data Structures

### IMU_Data_t
- `Accel[3]` - Accelerometer data (X, Y, Z)
- `Gyro[3]` - Gyroscope data (X, Y, Z)
- `TempWhenCali` - Temperature at calibration time
- `Temperature` - Current temperature
- `AccelScale` - Accelerometer scale factor
- `GyroOffset[3]` - Gyroscope offset values (X, Y, Z)
- `gNorm` - Norm of gravity vector

### QEKF_INS_t
- `q[4]` - Quaternion estimate values
- `GyroBias[3]` - Gyroscope bias estimate values
- `Roll, Pitch, Yaw` - Euler angles in radians
- `UpdateCount` - Number of filter updates
- `ConvergeFlag` - Convergence flag
- Various other internal states and parameters

## Functions

### Basic Sensor Algorithms
- `BMI088_Init_Algorithm()` - Initialize the algorithm data structures
- `BMI088_Calibrate_Offset()` - Perform sensor calibration
- `BMI088_Read_Algorithm()` - Process raw sensor data
- `BMI088_Set_Accel_Sensitivity()` - Set accelerometer sensitivity
- `BMI088_Set_Gyro_Sensitivity()` - Set gyroscope sensitivity
- `BMI088_Verify_Calibration()` - Verify calibration results

### Quaternion EKF Algorithms
- `IMU_QuaternionEKF_Init()` - Initialize the quaternion EKF
- `IMU_QuaternionEKF_Update()` - Update the quaternion EKF with new sensor data
- `Kalman_Filter_Init()` - Initialize the underlying Kalman filter
- Various other Kalman filter functions for prediction and update steps

## Testing

Two test programs are provided to verify the implementation:

1. Basic functionality test: `test_quaternion_ekf.c`
2. Verification test with sample data: `verification_test.c`

To compile and run the tests:

```bash
# Basic test
gcc -o test_quaternion_ekf test_quaternion_ekf.c Src/bmi088_quaternion_ekf.c Src/bmi088_algorithm.c -lm
./test_quaternion_ekf

# Verification test
gcc -o verification_test verification_test.c Src/bmi088_quaternion_ekf.c Src/bmi088_algorithm.c -lm
./verification_test
```

## Differences and Improvements

### Differences from Original Implementation

1. **Pure Algorithm Focus**: The library contains only the algorithmic parts without any hardware-specific communication code.

2. **Data Flow**:
   - Original: Directly reads from hardware registers
   - New Library: Processes pre-read raw sensor data passed as parameters

3. **Calibration Approach**:
   - Original: Performs real-time calibration with hardware communication
   - New Library: Processes pre-collected sensor data for calibration

4. **Memory Management**:
   - Original: Uses static memory allocation
   - New Library: Uses dynamic memory allocation with proper cleanup

### Improvements in the New Implementation

1. **Modularity**: Clean separation of algorithms from hardware communication.

2. **Portability**: Can be used on any platform with C compiler support.

3. **Testability**: Includes comprehensive test programs to verify functionality.

4. **Documentation**: Detailed function descriptions and usage examples.

5. **Error Handling**: Better error checking and return codes for matrix operations.

6. **Code Clarity**: Improved variable naming and code structure.

7. **Memory Safety**: Proper memory allocation and deallocation.

## Testing

Two test programs are provided to verify the implementation:

1. Basic functionality test: `test_quaternion_ekf.c`
2. Verification test with sample data: `verification_test.c`

To compile and run the tests:

```bash
# Basic test
gcc -o test_quaternion_ekf test_quaternion_ekf.c Src/bmi088_quaternion_ekf.c Src/bmi088_algorithm.c -lm
./test_quaternion_ekf

# Verification test
gcc -o verification_test verification_test.c Src/bmi088_quaternion_ekf.c Src/bmi088_algorithm.c -lm
./verification_test
```

## Notes

This library contains only the algorithm parts of the BMI088 driver. Hardware communication (SPI/I2C) must be implemented separately based on your specific platform and requirements.

All functions have been fully implemented including:
- Matrix operations (add, subtract, multiply, transpose, inverse)
- Kalman filter steps (predict, update, measurement)
- Quaternion EKF specific functions (observe, linearization, H matrix setup)

The library has been verified to produce correct results with sample data, ensuring it maintains the same mathematical accuracy as the original implementation.