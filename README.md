Q Gripper
=========

A simple 1 Dof 2-finger gripper. 
One finger is fixed, the second finger is actuated by a Feetech STS3020 Servo.
The fixed finger is equipped with a load cell measuring the closing force.
An Arduino Nano 33IoT is used as controller.
Most parts are 3D-printed.

Gripper Message Reference
-------------------------

The gripper communicates with the PC in text form via a serial interface in both directions. 
The type of the message is indicated by the first letter in it followed by data.
Status messages are sent by the gripper to the PC and command messages are sent to the gripper.


### Status Messages

**A**: 

### Command Messages




Hardware
--------


Stl files are located in the *URDF* directory. 
While some are modeling the off-the-shelf parts used to build the gripper, many are 3D-printed. 


**gripper_hull.stl**: The hull of the gripper, including the TAMS flange mount.




### Camera Holder
