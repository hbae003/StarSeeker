# StarSeeker

This is the final project for CS122A at University of Califonia, Riverside.

## Introduction 

The Star Seeker is a laser mount inspired by the equatorial Goto mount used by telescopes to automatically point at an astronomical object. Like the Goto mount, the Star Seeker is able to track the night sky by driving the right ascension axis. A gyroscope is placed at the base gear to keep track of the right ascension of the laser. The motor connected to the base gear turns the gears until the gyroscope data matches the desired degree. The second motor controls the declination of laser. The atmega holds the current degree of declination and turns the motor to the desired degree by calculating the error between current degree and desired degree. The real time clock is used to calculate the Local Sidereal Time in order for the laser to track the stars. The controller allows for the user to scroll through a list of different constellations and select one to find. It also allows the user to enter manual mode and control the laser with the D-pad. 

## User Guide
1. Turn the Star Seeker until the MPU6050 is on the north side of the base gear.
2. Make sure the laser is pointing perpendicular to the floor at a 90 degree angle.
3. Using the tripod, tilt the entire mount so that the laser is pointing to the North Celestial Pole (North Star - Polaris).
4. Turn on the arduino, connect USART with atmega and allow 15 seconds for the gyroscope to calibrate. 
5. After 15 seconds passes, the base moves until the the gyroscope finds the degree where RA = 0 hours.
6. After the gyroscope calibrates, choose any constellation from the LCD screen.

**Note: At any point, pressing START puts the system into manual mode. To exit, realign the laser until the declination is 90 degrees and press START again.**

## Components Used

* Atmega1284 - Controls the two motors and receives data from the Arduino via USART
* Arduino Uno - Receives data from the MPU6050, RTC, and NES controller. Also outputs data to the LCD screen. 
* NES Controller - Allows the Arduino to receive user input.
* RTC - Keeps track of the real time that is used to calculate Local Sidereal Time.
* MPU6050 - Provides information to the arduino in yaw, pitch, and roll degrees. 
* LCD Screen - Allows the user to select an object to find. 

## Connection

![star](/Connection.png)

## Demo

A video demo of the project can be found [![here.](http://img.youtube.com/vi/mrKWNr8jOBU&t/0.jpg)](http://www.youtube.com/watch?v=mrKWNr8jOBU&t)

## References 

[Star Track](http://www.instructables.com/id/Star-Track-Arduino-Powered-Star-Pointer-and-Tracke/?ALLSTEPS) - Most of the information needed for this project was found here. The code for Arduino is very similar but a different MPU6050 library was used. The RTC library contains example code that was used in the main arduino code. The model for the 3D print was also taken from this site. 

[NES Controller Code](http://forum.arduino.cc/index.php?topic=8481.0) - The controller_read loop was turned into a state machine. 

[MPU6050 Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) - Provided by Jeff Rowberg.

[LST](http://www.stargazing.net/kepler/altaz.html) - The calculations needed to find Local Sidereal Time can be found here
