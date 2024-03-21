<!-- link list, last updated 15.01.2023 -->
[0]: https://www.sparkfun.com/products/retired/13284
[1]: https://compsci290-s2016.github.io/CoursePage/Materials/EulerAnglesViz/

# IMU

An IMU (Inertial Measurement Unit) sensor is a compact electronic device that measures and reports motion-related data. It typically consists of gyroscopes, accelerometers, and sometimes magnetometers. Gyroscopes measure angular velocity, accelerometers measure linear acceleration, and magnetometers detect magnetic fields. IMU sensors are commonly used in robotics, drones, wearable devices, and motion-capture systems to track orientation, velocity, and gravitational forces. They provide essential data for navigation, stabilization, and motion analysis in various applications.

<p align="center">
    <img src="../images/imu.png" alt="IMU sensor" width="340"/> </br>
    <i>IMU breakout board</i>
</p>

## Technical Specifications

| | LSM9DS1 IMU sensor|
|-|-|
|Channels           | 3 acceleration, 3 angular rate, 3 magnetic field|
|Data output        | 16 bit                                          |
|Interface          | SPI/I2C                                         |
|Supply voltage     | 1.9 V to 3.6V                                   |
|Linear acceleration| ±2/±4/±8/±16 g                                  |
|Gauss magnetic     | ±4/±8/±12/±16                                   | 
|Dps angular rate   | ±245/±500/±2000                                 |

## Links

[SparkFun 9DoF IMU Breakout - LSM9DS1][0] <br>

## Datasheets 

[LSM9DS1](../datasheets/lsm9ds1.pdf)

## Principle of Operation
IMU with 9 degrees of freedom is a device that electronically measures and provides information about a body's specific force, how fast it's rotating, and sometimes its orientation. It does this by using a mix of accelerometers, gyroscopes, and magnetometers.
- Accelerometer - it detects acceleration or changes in velocity along the three axes of 3D space. Sensor operates by employing a small mass connected to fixed capacitors arranged using MEMS technology. When the sensor experiences acceleration or velocity changes in a specific direction, the mass inside the accelerometer moves relative to the fixed capacitors. This alters the distance between the mass and capacitors, thereby changing the capacitance of the system. An electronic circuit detects this capacitance change and converts it into a digital signal. </br>
- Gyrometer - is a sensor designed to gauge the rate of rotation or angular velocity of an object across the three axes of 3D space.Functioning through a minute vibrating component, often a MEMS resonator, it oscillates within a specific frequency range. As the gyro undergoes angular motion, the Coriolis force impacts the resonator, a process detected by sensors positioned around it. These sensors ascertain the rate and direction of angular motion, which is then translated into a digital signal by an electronic circuit. </br>
- Magnetometer - is a sensor crafted to gauge the intensity of the magnetic field surrounding it across the three axes of 3D space. Its mechanism involves a MEMS (Micro-Electro-Mechanical Systems) device housing a minuscule silicon structure designed to respond to magnetic fields.When the sensor encounters a magnetic field, the silicon structure undergoes movement, detected by capacitive or resistive elements within the MEMS device. This alteration in capacitance/resistance is then detected by an electronic circuit, which transforms it into a digital signal. </br>

## Practical Tips


## IMU Driver
The ``IMU`` class is a tool used to process the results through the fusion of three sensors included in the IMU and calculations used to obtain information about the position of the board in space, as well as about the movement of the board.
The following data can be obtained directly from the driver:
- gyroscope - values in three XYZ axes;
- accelerometer - values in three XYZ axes;
- magnetometer - values in three XYZ axes;
- board orientation expressed in quaterions obtained by means of the fusion of sesors with the use of a Mahony filter;
- board orientation expressed in Euler angles in the Roll Pitch Yaw convention [visualization][1];
- tilt angle;

To start working with the IMU, it is necessary to create an object in the ***main.cpp*** file and assign the correct pins. </br>
There is no need to plug anything as IMU is an integral part of the PES Board. Pins that are used to communicate with the IMU:
```
// IMU
#define PB_IMU_SDA PC_9
#define PB_IMU_SCL PA_8
```

### Create IMU Object

Add the IMU driver ``IMU.h`` to the top of the ***main.cpp*** file:

```
#include "pm2_drivers/IMU.h"
```

To be able to start to use the ``IMU`` driver, the initial step is to create the IMU object and specify the pins to which the hardware is connected in the ``main()`` scope.

As mentioned the IMU is using two pins to communicate. Next step is to create an object of the class that will serve as a store of data, sorted in an appropriate way and second object with the associated pins passed as an argument to receive measurments from IMU:

```
ImuData imu_data;
IMU imu(PB_IMU_SDA, PB_IMU_SCL); 
```
