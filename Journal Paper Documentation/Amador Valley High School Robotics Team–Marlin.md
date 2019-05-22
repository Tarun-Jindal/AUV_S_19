# Amador Valley High School Robotics Team–Marlin

Marlin’s main computer is built on a Jetway
NF9J-Q87 Mini-ITX motherboard. It contains a 2.7
GHz quad core Intel i7-4790T Haswell processor, 8GB
of RAM, and a 120GB mSATA SSD. The main computer handles high-level functions such as image processing. The NF9J-Q87 was chosen for its large number of
USB ports and its onboard RS-232 ports. The i7-4790T
was chosen for its combination of high processing power and low power consumption. At full power, the main
computer uses just 60W. To handle the heat generated
by the main computer, a heat pipe CPU cooler is used.

## Software Sub-Division Overview
 This subdivision designs and improves algorithms to manage various aspects of the AUV’s operation, including mission planning, navigation, localization, machine vision, and signal processing.
 
 To manage the complex software system with multiple collaborators, the team uses Git version control. Git allows the software team to work on the same files independently while keeping backups of previous versions of the code. **Marlin’s software stack runs on the Debian OS (GNU/Linux)**. Dubbed **Aquastorm**, it consists of multiple processes that communicate over a shared **mutex** and **named pipes**. Each process works independently of the others, while the mutex ensures thread safety. **Most software is written in C/C++ and Python**.
 
 mutex - In computer programming, a mutex (mutual exclusion object) is a program object that is created so that multiple program thread can take turns sharing the same resource, such as access to a file.They key to understanding the need for a mutex lies in the reordering of memory access and that lack atomicity in large data structures. The typical mutex solves both of these problems. It ensures that only one thread is executing a key piece of code at a time, which in turns limits access to a data structure.It ensures that the both threads have a full and proper view of that memory irrespective of any CPU reordering.

## Interface
Interface is the master program that starts each process on the software stack. Each process is run in a separate thread to increase the efficiency of our program. The threads run independently from each other, and read or write to a mutex when they need or receive data (e.g. images, the current state, the prediction model).Each program can issue queries or commands to Interface through the designated pipes. Typically these involve requests for various objects (e.g.,
images, the current state, and the prediction model) or
messages to pass to other processes. Each connection
is processed in a separate thread, so each component is
plug-and-play; if a component is disconnected, Interface
will just stop using that information; if it is reconnected,
Interface will resume using that information.

## Control
Our low-level controller runs on the **ATMega2560 microcontroller**. It connects to **Aquastorm, the AHRS, and the thruster controllers via serial UART pins**. It uses six PID filters, one for each bearing of the submarine. The resulting PID vector is multiplied by the thruster matrix, mapping the effect of each thruster on each element of the state, to produce the desired thruster power. The state consists of the (X, Y) position, computed by integrating the thrust vectors, **the depth, determined from the analog pressure sensor**, and the roll, pitch, and yaw from AHRS. Control sends the current state to the control process in Aquastorm, and receives the desired setpoint. It also activates the relays on command. **Because of the simple serial protocol, Marlin can be controlled manually through a terminal for testing and debugging use**. Furthermore, all configuration variables (e.g., PID gains and the thruster matrix) can be adjusted over serial communication to empirically tune the optimal values on the fly. **Most of Control, aside from a limited set of AVR specific sections such as input / output, is designed to be platform independent, so it can be tested and debugged offline on any laptop**.

## Modeling
In order to robustly navigate obstacles and account for imprecise positioning and observations, Modeling localizes the sub to its environment. Our modeling algorithm uses Monte Carlo Localization, which initially constructs a uniform particle filter. Each particle represents a possible state of the sub. As the particle filter is first set to be mostly uniform, the sub has little knowledge about its surroundings. However, as the submarine moves around its environment, it is able to resample its particles with motor and sensor updates. Eventually, the particles converge upon the actual location of the submarine, leading to successful localization.


In order to robustly navigate between objects
in the pool and account for imprecise positioning and
possibly unreliable observations, Modeling constructs
a probabilistic map of all objects in the pool. Our custom designed modeling algorithm incorporates recursive Bayesian estimation to update its predictions as
it receives new signals from the perception processes.
The probability density map is computed as the sum of a
set of hypotheses (Gaussian distributions) with varying
means and variances. For each observation (given in the
form of another Gaussian distribution), each hypothesis
is split into two hypotheses, one accepting the observation and the other rejecting it, based on a given certainty. Gaussian distributions are combined by multiplying
them, forming a new Gaussian distribution as described
in [1]. In order to prevent exponential computational
complexity as more observations are added, Modeling
trims out unlikely hypotheses, where likelihood is proportional to the integral of the probability density function (PDF) across all dimensions.
This solution was arrived at after much research
into alternative systems. It was found that a particle filter
[3] would have to be either too slow or too inaccurate,
and Kalman filter variants (e.g., Unscented Kalman Filter [4] and Extended Kalman Filter [6]) would not be
able to account for bad observations without hard-coding an error rejection filter. The implementation of a
multi-hypothesis model whereby each hypothesis is given by arbitrary PDFs was considered, but because of the
inaccuracies involved in combining hypotheses and the
high power requirements for computing the integral, instead Modeling enforces that all hypotheses are Gaussian, resulting in the solution described above, at the cost
of slight approximation error

## Vision
Marlin has one front facing USB camera and one down facing ethernet camera. Each competition element is detected with a separate vision module for efficiency
and parallelization. Each competition task is associated with **a vision function within Aquastorm, that returns an observation based on what objects it detects. Most vision processes initially run preprocessing to generate more contrast in the image**. Afterwards, they use a variation of **machine learning, color mapping, edge, contour, and blob detection** with the aid of the OpenCV library. Our**machine learning algorithms utilize the Tensorflow Object Detection API to detect objects within an image**.

## Mission Control
Mission Control generates a list of goals (e.g., ram
the red buoy), each containing a list of primitive actions
(e.g., move to location). Each goal is associated with a
point value and time to completion (TTC). At the start of
a run, and after each goal is completed, Mission Control
selects a new goal on the fly based on the point value,
TTC, and distance (computed from the most likely hypothesis). This allows for varying task sequences in order to adapt to unforeseen circumstances.


Mission generates a list of goals to complete throughout the competition run. Each goal contains a set of instructions, location, and point value. Based on the location of both the sub and the goal, we can compute an estimated time to completion for each goal, and visit the goals in an optimal fashion, maximizing our point value. Once we select a goal to complete, mission tells the corresponding vision function to start returning observations.

## Otolith
Otolith, written using LabVIEW, runs on the FPGA and reads from the four hydrophones located outside the main hull. The analog-to-digital converter (ADC) receives amplified signals from the hydrophones at 500 thousand samples per second. Using each analog signal, Otolith applies a Butterworth low pass filter to remove extraneous noise, then looks for a ping of the designated frequency. The time at which the ping is detected is recorded for each hydrophone, then the time difference of arrival (TDOA) is computed and sent to the main computer for localization. On the main computer, a small C program receives the TDOAs and computes the location of the pinger using trigonometric ratios between the TDOAs; this method, while introducing a slight built-in error (which decreases as the sub points towards the pinger), is **less susceptible to slight errors in physical measurements and ping detection when compared to multilateration (solving the system of hyperboloids), making it much more robust**. It sends to Interface the corresponding **Gaussian distribution**.
![](http://www.avbotz.com/images/otolith.png)

## Logging
Because essentially all important information
is sent over the pipes between processes, Marlin uses
the standard “tee” command to copy messages to a file.
Therefore, Marlin records comprehensive logs including
the current state, the estimated model, images, and the
results of vision processing.

## Renderer
The renderer is designed in the free and opensource 3D-modeling software Blender. When it is given a state, it generates a front and down image from that
state and sends them over the designated pipes. Combined with a simple physics engine, this serves as a
simulator for the entire competition course. Because the
simulator can be run on each development computer, this allows different team members to work on different
tasks concurrently. In addition, the simulator increases
development efficiency by detecting time-consuming
bugs during dry testing.

## Testing
Marlin’s software stack has been
extensively tested offline to ensure correctness, safety,
and robustness. Each process was run through valgrind,
which reports zero definitely lost and very few possibly
lost blocks, which appear to come from either valgrind
itself, libc, or included libraries. Control was tested with
a custom physics simulator to verify working PID. Each
vision module has been tested on hundreds (or in some
cases, thousands) of images to ensure accuracy; each
module tends to get over 90% accuracy (e.g., buoys gets
around 98%). Furthermore, the software system as a
whole has been tested with the renderer to simulate an
entire competition run.
