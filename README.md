# IOT-SUM24 - Feild Training - 02-24-03205 

Final Project for Feild Training 1

## Project Description
This is a simulation of an autonomous factory, where the roaming parts are collected and given to a robotic arm to sort them into their designated bins.


## Main Parts

### Robotic Arm
The arm uses continous rotation servo motors to move freely in 360 degrees which allows for full flexibility. It uses a Raspberry Pi-Cam to send data over to a server and the server returns a code, this code is then used to perform the correct action for the arm.

### UGV 
This small but fast robot uses an ESP-Cam to send data over to the server and translate them into fuzzy values to move the motors, then the objects are collected and sent into the sorting zone, if the UGV gets hazzy, it can go to the calibration zone and get its sensors working again.

### Server
Aided with a YOLOv5 model, it detects the collected objects for classification and distance measurements, then sends the coordinates data over for fuzzification in the UGV.
