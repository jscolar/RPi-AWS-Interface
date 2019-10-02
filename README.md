# RPi-AWS-Interface

Integration of the code for a monitoring device on a bus, tracking location, occupancy and acceleration of one vehicle using a Raspberry Pi, Bluetooth as an internal communication, NB-IoT for real-time data and AWS as an IoT device manager and data processing platform.

The architecture of the solution considers two different nodes, connected via Bluetooth. The sensors, distributed between both nodes, consider an accelerometer, a GPS, a videocamera and a PIR sensor.

Location is defined on a first stage by the GPS, with an accuracy of less than 2 mts. A dead-reckoning algorithm will be adapted in future work to improve the performance of the tracking device.

Occupancy is measured by both the videocamera using a computer vision algorithm and a PIR sensor. Accuracy is less than 70% using the camera, but it is being improved.

Acceleration is monitored by an accelerometer and a gyroscope, and it is used within a mulivariate anomaly detection algorithm to identify the state of the road.

The fist section of the Master Code identifies the multiple initializations of the electronics and the variables required for the algorithms to function.

The security keys (which are not included) are declared, as well as the AWS certificates and the MAC addresses of both nodes.

The process takes place in three threads, one for tracking location and communicating with the slave node, one for the computer vision algorithm and one for the communication with the AWS platform.

The slave node follows the same principle, first initializing the variables, building the monitoring methods and then, using two threads to track the acceleration and communicate with the master node.
