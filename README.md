# ROS2 Triton camera driver

A simple camera driver for the LUCID's Triton cameras using ROS2

## Description

This program takes images from one or more cameras on an external trigger then send those images to topics in ROS2

## Getting Started

### Dependencies

- ROS2 Humble
- LUCID Arena SDK
- libserial-dev

### Installing

Clone the repository  
```bash
git clone https://github.com/The-Last-Resort-FR/ROS2_Triton_camera_driver.git
cd ROS2_Triton_camera_driver
```
Build  
```bash
colcon build --symlink-install
```
Source the environement  
```bash
source install/setup.bash
```
The trigger source should be connected to line0 on the cameras (green wire on official M8 cable)

### Executing program

Run the node  
```bash
ros2 run lucid_cam_driver lucid_cam_node
```
Make sure the trigger source is actually triggering the cameras  
You can visualize the images on the topics depending on the configuration of the camera master
```bash
ros2 run image_view image_view image:=/camera/image _image_transport:=theora # SINGLE mode
ros2 run image_view image_view image:=/camera/imageL _image_transport:=theora &  ros2 run image_view image_view image:=/camera/imageR _image_transport:=theora # DUAL mode

```

## TODO

- Better error handling
- Restarts
- More ROS2 compliance

## Tested Hardware

- Host: Jetson AGX Orin
- Cameras : TRI054S-CC, TRI050S1-QC

## Version History

- V0.1.0 Inital version
- V0.1.1 Adding communication with a MCU via USB Serial for hardware triggerring

## License

This project is licensed under the "The Unlicense" License - see the LICENSE.md file for details

## Acknowledgments

-  LUCID Arena SDK