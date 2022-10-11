# SudoOS

SUDO is our STM32-based main flight control computer in the Model Satellite System we designed as the Yildiz Rocket Team. Here are some topics covered by the SudoOS project.
 
- FreeRTOS usage with CMSIS v2 interface for STM32
 
- Use of C struct, enumeration etc. for a clean architecture
 
- The BNO055 IMU tuning and data reading (poll method) library we have prepared is used. In this project, the library is used for operations such as remapping axis and setting the desired mode of the sensor. BNO055 is a smart sensor and has fusion mode which supports maximum 4G acceleration in this mode. With this mode, healthy Euler angle data can be received internally from the sensor. In this project, this mode was used and data was taken with the interrupt method in order to get the data as fast as possible. This way, data is captured hundreds of times per second and processed as soon as data is received, without blocking the main code.
 
- Data reading was also made from the BME280 pressure sensor with the interrupt mode. Pressure and temperature data are received hundreds of times per second with the interrupt method, and operations such as altitude, movement direction, max altitude, and vertical velocity calculation are performed each time in this function.
 
- Location detection is done by using the Quectel L86 GPS module, with the interrupt method, by capturing the NMEA sentences sent over the UART and parsing with the help of the sscanf function. Thus, data such as lat, long, utc_time, GPS speed, and the number of satellites are read in a non-blocking way.
 
- For efficient communication with the ground station, the data is packaged by using C union and transferred to the ground station with the SPI RFM98W-433S2(LoRa) module. Receive commands by listening to data coming from the same module (bi-directional usage).
 
- Controlling the servo that provides separation in the model satellite system and the quad brushless motor with ESC connection, which provides controlled descent, with Timer PWM with high sampling. Besides using PWM, motor control has a PID control algorithm architecture. The used algorithm is used to stabilize the satellite according to the angle data. The gain values of the PID algorithm can be changed remotely.

- Data logging using SD card in STM32. For more detailed usage of SD card like file transfer, you can refer to our file transfer project.

![SUDO](https://i.hizliresim.com/nlzlmui.jpg)
