#Coursework - Kalman filter for calculating tilt angle
Name: Ben Williams. College: Clare. CRSID: brw32.

##Code layout
The key relevant files for this project are 'warp-kl03-ksdk1.1-boot.c', 'devMPU6050.c' and 'devMPU6050.h'. These can each be found in the folder 'src/boot/ksdk1.1.0/'

'devMPU6050.c' is the device driver for the MPU-6050 inertial measurement unit. It allows the content of device drivers to be read via I2C, ensuring that the power management register is set correctly to ensure the device is not in sleep mode.

'devMPU6050.h' is the corresponding header file used for function and variable definitions.

'warp-kl03-ksdk1.1-boot.c' then implements I2C communications to read from the accelerometer and gyroscope registers on the MPU-6050. Each iteration, it uses the Kalman filter algorithm to estimate the tilt angle (part of the device *state*) based on previous estimates and the current sensor values. The state estimate is then printed.

'CMakeLists.txt' and 'build/ksdk1.1/build.sh' have also been edited accordingly for the new device driver.

##Hardware
The required hardware is:
* 1 x FRDM KL03
* 1 x MPU-6050 IMU
* 1 x Breadboard
* 5 x Male to male jumper wires
* 1 x USB mini to USB A cable for laptop connection (if printed output)
