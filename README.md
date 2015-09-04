# ROS Driver for Motion Capture Systems
This package contains ROS drivers for two different motion capture systems,**Vicon** And **Qualisys**.

## License
Apache 2.0 wherever not specified

## Compiling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace.

```
roscd motion_capture_system
catkin_make --pkg motion_capture_system --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This will complie the drivers for both **Vicon** and **Qualisys**

## Example Usage

**Common Parameters**

`server` (`string`)

Address of the server of the motion capture system to be connected.

`frame_rate` (`int`, `default: 100`)

The frame rate of the motion capture system

`max_accel` (`double`, `default: 10.0`)

The max possible acceleration which serves to construct the noise parameters.

`publish_tf` (`bool`, `default: false`)

If set to true, tf msgs for the subjects are published.

`fixed_frame_id` (`string`, `mocap`)

The fixed frame ID of the tf msgs for each subject. Note that the child frame id is automatically set to the name of the subject.

`model_list` (`vector<string>`, `default: []`)

A vector of subjects of interest. Leave the vector empty if all subjects are to be tracked.

**Published Topics**

`\{mocap}\{subject_name}\odom` (`\nav_msgs\Odometry`)

Odometry msg for each specified subject in `model_list`.

**Node**

`roslaunch motion_capture_system {mocap}.launch`

## FAQ

1. Will the msgs be delayed if the driver is handling several subjects? The driver is multi-threaded. It uses different threads to process the msg from different subjects. So, emmmmm, don't worry =).

## Bug Report

Perfer to open an issue. You can also send an E-mail to sunke.polyu@gmail.com.
