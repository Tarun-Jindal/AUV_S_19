# CalTech_2016_RoboSub_Journal

## Overview
Dory runs **Ubuntu**, the most popular Linux
distribution, on an **Intel NUC™ with a Core i3 processor and a 120 GB solid-state hard drive**. This
setup provides high performance in a small, rugged,
low-power form-factor. For convenience, external
connections can be made either via WiFi or Ethernet.


Dory’s autonomy relies on its robust all-**C++**
software architecture, consisting of five processes. This architecture is modular
in order to aid development, bug isolation, testing,
and logging. Each of these five processes run
independently while communicating with each other
using a custom, **interprocess communication (IPC)**
library based on a publisher-subscriber model.
Processes can publish data to topics and subscribe to receive all data published to relevant topics. 
**The team chose to write its own IPC library instead of using an existing framework such as ROS in order to minimize the use of external black-boxes and reducing the learning curve for new members**.

## Hardware Process
The hardware process interfaces with the lowlevel serial devices including the core board **(depth sensor, kill switch, and solenoids), Doppler velocity logger (DVL), attitude heading reference system(AHRS), VideoRay M5 thrusters, and Dynamixel servos**.
**The output is orientation and position.**
Orientation is provided by our **AHRS, which internally fuses accelerometer, gyro, and magnetometer measurements**. The magnetometer is
calibrated to account for the distortion of the fields
caused by the vehicle using the onboard live HIS
calibration. Position is a little more problematic
because the **DVL only pings once per second, and if the water column is not tall enough, no data is returned**. With these limitations we can neither run
tight translational control loops nor do tasks below
us. To deal with both of these issues, we
implemented a **custom 12-dimensional Extended Kalman Filter**. While many teams use an Unscented
Kalman Filter, because this is not analytic method, it
is plagued by the curse of dimensionality and
sampling issues. Instead, an Extended Kalman Filter
linearizes the process around the current state, and
these linearizations of quaternion and matrix
expressions are all computed from analytic formulae.
Using quaternions instead of Euler angles minimizes
the non-linearities. **The key insight of the Kalman
Filter is that it incorporates the thrust outputs in order
to estimate position when the DVL is unable to take a
measurement or in between pings**. The output of the
Kalman filter is a unified state estimate including 3D
position, quaternion-based orientation, and
translational velocities, and angular rates, which is
periodically published to the robot state topic.

## Mobility Process

**The mobility process is responsible for setting
the speeds of all seven thrusters based on the current
state and target state, received from the hardware and
commander processes respectively**. 


**Last year we used
a custom 6 DOF dynamic-inversion-based PID
control loop. However, because the center of mass
and center of buoyancy were not lined up perfectly
our vehicle was not stable, which was a major
performance bottleneck**.
An **engineer at NASA JPL suggested using a
linear quadratic regulator (LQR)**, which is an optimal
control scheme for systems that can be modeled as
x = Ax+Bu,
where x is the state of the system, u is a vector of
thrust forces, and A and Bare matrices. An **LQR
controller, unlike a PID controller, computes the
optimal path to minimize a quadratic cost function
depending on x(t)**. We chose the state, x, as an 18-
dimensional vector including translational and
angular errors, integrals of errors, and velocities. This
way our cost function could penalize absolute error,
steady state error, and high speeds.
However, **rotations in 3D make the vehicle
dynamics nonlinear**. So we had to first linearize the
system around the target state. This was done using
an analytic formula which returned the linearized
versions of A and B for the current state using
complicated matrix and quaternion expressions. The
result was given to MIT’s Drake Library in order to
solve for the gain matrix and optimal thrusts on the
fly. **All the constants used in the dynamic modeling
are generated from the CAD including the center of
mass, center of buoyancy, inertia tensor, thruster
positions, and thruster directions**. To get accurate
thrust control, we profiled our thrusters using a
strain-gauge with high-speed software-controlled
measurement.
Like our last controller, LQR can function at all
pitch and roll angles. But because LQR accounts for
vehicle dynamics and plans an optimal path, it can
uniquely deal with non-diagonal inertia tensors and
misaligned centers of mass and buoyancy. For this
reason,** its stability is leagues ahead of any PID-style
control system**. In addition, it requires no tuning: we
transitioned vehicles, including changing thruster
type and configuration without touching a single
tuning parameter, whereas re-tuning a PID controller
would have taken a month.

## Vision Process

The vision process converts images into
actionable information. It asynchronously reads new
frames from an **Allied Vision Guppy Pro camera
using libdc1394**. Useful features of this camera are
the high quality optics and CCD, exposure control,
white balance control, and on-demand shutter, which
**help us prevent reflective objects from saturating and
know exactly where the vehicle and gimbals were
when each frame was taken**.
After the image is pulled and corrected for
distortions, they are fed through a** custom detection
algorithm**. One of these is a rectangle detector
(including the torpedo, path, navigation channel,
marker bins, and the orange covers). Our **novel
algorithm is based on Gauss-Newton minimization
and determines the 3D orientation and position of any
rectangle given the pixel positions of its corners, even
in the depth dimension and even while looking from
very skew angles**. The validation gate use **Haar-like
features to detect the three segments of the gate**.
Finally, the remaining **detectors (buoys, handle, and
doubloon) use a mixture of edge detection, color
thresholding, and contour analysis**.
Lastly, the vision process controls the gimbals. **If
nothing was detected, the gimbals begin a search
phase. Once an object is detected, the gimbals lock
onto it so it remains centered in the field of view as the vehicle moves**. This eliminated many problem
last year related to finding field elements.

## Commander Process

The Commander process is responsible for
knowing which task we are on, how to accomplish it,
and how to move to the next task and directs the
other three processes. It has two threads. The first
runs the task while the second resets the first if the
kill switch is pulled. **The tasks are programed in a
state-machine-like architecture**.

## Logger Process

**In the rare case things go wrong, the logging
process helps us debug this by recording every
message on every topic, in both a binary Google
ProtoBuf and JSON formats (used for playback and
manual inspection respectively). One advantage of
this type of logging is that it allows us to visualize
any logged run via a Mathematica GUI.** 
