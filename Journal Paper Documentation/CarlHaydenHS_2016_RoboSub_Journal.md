# CarlHaydenHS_2016_RoboSub_Journal

## Vision
Vision system: This is comprised of several
components.
1. The vision processor (the “main
computer”),
2. Two HD USB board camera. (one
forward and one down)
3. And the viewing window. (Two inch
dome)
The viewing window is mentioned because
is has a significant effect on the field of
view of the camera. We elected to use a
dome with a depth of 2” and a diameter of
6” to minimize the reduction of FOV
underwater that results from using a flat
window. This shallow dome has less optical
power (less FOV) than a 6” diameter
hemisphere.

## Software Archietecture

There are four separate programs that run
our robot. The Teensy microprocessor
handles all of the inertial control loops,
maintaining our robot’s attitude and position in the pool by directly reading from the
robot’s inertial sensors and feeding the
motor escs the control loop output. The
teensy process communicates telemetry
information and receives set points from the
main program.
The main program is written in Java
and runs on the main computer. It is what
handles mission planning, managing the
various settings, logging data, and acts as
the communications hub. The mission
planner reads from a json file that describes
the tasks and the task order for the mission.
The settings system saves in a similar
fashion, reading and writing to a json file.
When the program boots, all the settings are
read out of the json file into a map that is
stored by the program. The settings are
communicated to the other programs in a
system similar to that of TCP, where the
settings are indexed and the teensy
occasionally informs the main program of
the highest index where all settings below
that index are up to date. The main program,
upon receiving that information, will send
the next 10 settings from the index received.
This way the teensy won’t be flooded with
all the settings immediately after boot.
The vision processing is done by an
entirely separate program, running on the
same computer, written in C++. It receives
filter commands and values from the main
computer, sending back coordinates and
such information. Our vision system can be
configured by using different “blocks”. Each
block represents one process, for example it
could be a read from camera block, a filter
block, or a display image block. Each block
is it’s own separate thread, and it passes
frames along by reference, using a pointer
buffer on both the input and the output to
organize it’s todo list.
The Operator Interface, written in
java, is run on the surface by a laptop.
Multiple instances of the OI can be run at
once from multiple laptops, allowing for easier viewing. The OI is also designed so
that the AUV can be run without it, so that
our runs can qualify for points. The OI can
manage a variety of tasks, each task is
designed into an “App”. The OI is divided
into panels, and each app can occupy any
given panel. The apps can be dragged
between different panels, two apps can even
occupy the same panel, by using a tabbed
system. Each app can have settings,
configurable by the operator by double
clicking on the app’s tab. Currently, the list
of apps that the operator has access to are as
follows:
 OIConsoleApp, for displaying
general debug messages from the
robot
 CommSettingsApp, allowing the
operator access to some of the
communications settings.
 OIVideoApp, which displays the
video feeds from the robot cameras.
 GamepadApp, for reading from and
sending gamepad values to the robot.
 TelemetryGrapherApp, displays
selected telemetry values to a graph
on the OI.
 RawTelemetryApp, displays all
telemetry values to a text window on
the OI

## Processors

The main computer this year is an
Intel i7 @ 4GHz on an asus micro ITX
motherboard, with 8 GB RAM. It uses a 240
GB SSD for the important systems and a
1TB HDD for general storage. Key tasks for
this processor are data logging, vision
processing and task sequencing.
I/O processor: We use two Teensy
3.1 microprocessors as our GPIO devices,
connected to the Main PC over USB. They
are both mounted on a custom PCB to
simplify cable management. The PCB has connectors for all of our devices, like the
DVL, the motor controllers, the Gyro, etc.
the bidirectional interfaces are mostly
asynchronous serial data (RS232 and
TTL). Also, there will be standard servo
type PWM outputs for the 10 thrusters.
The INS (Inertial Navigation
System) processor: Will be one of the two
Teensy 3.1 devices. The INS processor
reads the raw information from the FOG,
DVL and other sensors and forms a
navigation solution of the X-Y position,
attitude and depth of the AUV in the pool.
This navigation solution is used by the Task
sequencing processor in order to trigger
AUV actions to perform the tasks.
