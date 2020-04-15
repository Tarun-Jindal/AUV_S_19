# Beaver Country Day School
Code [link](http://github.com/BeaverAUV)

## Overview

Prospero II's software is what allows it to operate autonomously. It is structured around Robot Operating System (ROS). A state machine controls each individual task which the robot has to complete. All movement is controlled by a series of proportional, integral, derivitive(PID) controllers, which use data from all of the sensors to determine how fast each thruster needs to move in order to accomplish the goal of each individual task.
![](http://beaverauv.org/assets/software-block.jpg)

Prospero’s software systems, like its mechanical systems,
were designed to be as modular as possible. The software
systems were designed to be object-oriented. Each program
has several layers of abstraction to increase modularity and
usability.

## ROS
ROS (Robot Operating System) is the main architecture
that all of Prospero’s programs are built around. It provides
a simple yet robust framework that allows different nodes
(individual executable programs) to communicate and share
information with each other. This allows for simple passing
of data between programs. ROS has a large open source development community that allows it to provide support
for many different devices and libraries. 

## State Machine
Prospero uses a state machine to control task
management. The ROS package SMACH is used to manage
the state machine. Each task which the AUV must complete
is programmed to a different state; as each state is activated,
the robot executes different programs corresponding to
different tasks.
An ‘emergency kill’ state is used when the kill switch is
activated to ensure that the thrusters do not continue to run
when the robot is powered back on. This allows the kill
switch to shut off power to the thrusters without shutting off
power to the computer, while still ensuring that thrusters
will not continue to run upon returning power to the
thrusters. 

## Motor Control
Prospero’s motor control allows accurate movement in
six axes (surge, sway, heave, roll, pitch, and yaw). The
motor controller inputs the vehicle’s desired movement in
these six axes, and calculates how fast to drive each thruster
to achieve the desired movement. Each axis is controlled by
a PID (Proportional Integral Derivative) controller to ensure
accurate movement. Prospero’s PID loops were tuned to
ensure consistent performance. First, the maximum speed of
the AUV was determined so the PID loops could compare
an input of speed from the IMU to the percentage of speed
provided by the motor controllers. Then, the proportional,
derivative, and integral terms of each PID controller were
adjusted until each controller performed consistently.
After the desired thruster values have been determined,
the motor control calls several services (functions in ROS
which can be used across nodes) to communicate with the
Arduinos and control the ESCs. Data is passed to Arduinos
over a serial interface. The Arduinos monitor the PWM
values and update them as needed.

## Hydrophones Signal Processing
Prospero’s hydrophone system uses **digital signal processing to complete all the filtering and localization of the signal received from the acoustic pinger**. The software first works to identify when the pinger is emitting a ping.
Then,** the signals from the four hydrophones are recorded and stored as .WAV files in two-second increments to ensure that each recording captures an entire ping. An IIR(Infinite Impulse Response) bandpass filter is used to crudely filter the signals. Then, the filtered signals from two hydrophones are compared using cross-correlation to determine the TDOA (Time Difference Of Arrival) of the two signals. The TDOA represents the time difference between when the two hydrophones received the signal; this is converted to a difference of distance by multiplying by the speed of sound in water, about 1482m/s. Using this difference in distance, a hyperboloid is constructed which represents every possible location of the pinger. This is repeated for two more combinations of two hydrophones.After three hyperboloids have been constructed, they are compared to find an intersection which represents the location of the pinger. If an intersection cannot be found, the position of the pinger is approximated by looking for areas where the three hyperboloids come close to intersecting.**

## Vision Tracking
BeaverAUV implemented a vision tracking system that
allows for accurate underwater movement using various
objects as way-points. Vision tracking is handled by Open
Computer Vision (OpenCV), an open source image
processing library. Three main algorithms were developed
for vision tracking: image thresholding, color calibration
and movement calculations.
 * **Image Thresholding**: After the image is received from the
camera, the image is processed in the RGB (Red Green
Blue) colorspace. Using a series of preset RGB ranges, all
color is removed from the image—aside from those which
fall into the predetermined RGB ranges—and then stored as
a threshold image. A threshold image is a binary image,
where white pixels represent pixels where the original color
of the pixel is within the predetermined RGB ranges; black
pixels occur when the original color of the pixel is not
within the predetermined RGB ranges.

 * **Color Calibration**: In water, the perceived color of any
given object can change frequently and dramatically due to
changes in the environment, especially changes in sunlight.
This causes a need for accurate color calibration. In order to calibrate, the image tracking software sets the
aforementioned RGB ranges to a set of initial RGB range
values which have been chosen through testing. A small
card—meant for photographers to calibrate both color and
white balance—is used as a reference point to accurately
calibrate color. The card has extremely accurate colors
printed on it, including the shades of red, green and orange
used on RoboSub obstacles. For each color, once the
calibration card has moved in front of the camera, the
program steps through several variations of the RGB range
values. The best settings are determined by measuring the
area of the detected portion of the card, and comparing it to
the known area of the detected portion of the physical card. 

 * **Movement Calculations**: The tracked object’s location
relative to the AUV is found by calculating its centroid, and
finding the position of the centroid on a XY coordinate
plane which is defined by the individual pixels of the image
which is being processed. Based on the coordinates of the
object’s centroid, the movement algorithms decide how to
move depending on what task it is currently set to. To do
this, the image is split into several sections. These sections
change depending on the task. By default, the image is split
horizontally into three columns; the center column is the
smallest and the left and right columns are equal and larger.
If the centroid of the tracked object is in the left or right
columns, a signal is sent to the motor control software
which causes the AUV to translate or rotate left or right.
The center column is then split vertically into three rows.
Based on the location of the centroid of the tracked object
relative to the three columns, the AUV is signaled to
increase or decrease its depth so that the object becomes
centered in the coordinate plane. Once the object is within
the center section, the AUV is signaled to move forwards,
and to automatically compensate if the object moves outside
the center.
