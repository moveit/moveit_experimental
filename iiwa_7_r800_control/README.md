# iiwa_7_r800_control

Base communication with KUKA IIWA 7 R800 using the Fast Robot Interface over UDP with ros_control

## Build

This package will automatically build the original Sunrise.OS 1.7 SDK from Kuka that comes with the IIWA 7 R800. It does this by calling its default Makefile. Note: if you make any changes within the SDK, they will not be rebuild automatically by catkin. Rather, you should:

    cd fri_client_sdk/fri_client_sdk/build/GNUMake/
    make

## Run

See README in parent directory for instructions

## Configurtion

### Control Frequency

The control frequency of the IIWA-ros_control hardware interface is specified in the Sunrise OS Java code that runs on the robot. However, it is also specified in ``config/iiwa_7_r800_controllers.yaml`` so that a mismatch in control frequency times can be checked and alerted to user.

The control frequency has been chosen based on guessing several values and finding one that caused the least amount of "squeaking" from the actuators. It must be between 1 and 100 ms (1000hz and 10hz)

 - 200hz was the default being used by the example applications, but we found it noisy
 - 500hz seemed to make it louder
 - 300hz seems good enough (only occational noise)
