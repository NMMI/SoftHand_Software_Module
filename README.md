# SoftHand node
ROS node to communicate with SoftHand Device

## Installation
### Requirements
If you have never set it up, you probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. To do so, just open a terminal and execute the following command:
```
sudo gpasswd -a <user_name> dialout
```
where you need to replace the `<user_name>` with your current linux username.

_Note: don't forget to logout or reboot._

### Sources
>Since you are interested in the ROS interfaces for our devices, it is assumed that you are familiar at least with the very basics of the ROS environment. If not, it might be useful to spend some of your time with [ROS](http://wiki.ros.org/ROS/Tutorials) and [catkin](http://wiki.ros.org/catkin/Tutorials) tutorials. After that, don't forget to come back here and start having fun with our Nodes.

Install the package for a ROS user is straightforward. Nonetheless the following are the detailed steps which should be easy to understand even for ROS beginners:

1. Clone the package to your Catkin Workspace, e.g. `~/catkin_ws`:

1. Compile the packages using `catkin`:
   ```
   cd `~/catkin_ws`
   catkin_make
   ```

### Device Setup
Connect the device to your system through the USB connection and power up the system with the proper voltage [12V or 24V for SoftHand].

## Configuration
Main parameters can be configurable by a YAML file, which is request during the call to softhand node with "roslaunch" method.
If you do not specify your yaml own custom file, by variable yamlFile:="PathToYourConfFile", a default yaml fill will be set.

example

**communication port**

port: '/dev/ttyUSB0'

**ID of the device**

ID: 1

**Node frequency [Hz]** 

run_freq: 100

**Reference commands topic**

ref_hand_topic: "ref"

**Actual position reading topic**

meas_hand_topic : "meas"

## Communication
With default configuration, communication is allowed through two topics.

  - /SoftHand/ref
  - /SoftHand/meas
  
**Message Type**

[/SoftHand/ref]

Message is of type std_msgs::Float64. Value is for closure reference in the range [0.0 - 1.0].

[/SoftHand/meas]

Message is of type std_msgs::Float64MultiArray. First value is the actual position of the hand.
Expected values are in the range [0.0 - 1.0].

## Basic Usage

Launch the node

```
$ roslaunch softhand softhand.launch
```

Send the commands through ROS topic (e.g. for half hand closure):

```
$ rostopic pub /SoftHand/ref std_msgs/Float64 "data: 0.5" 
```

