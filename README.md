# ROS2 Astra Camera

This README needs help...

## Usage
1. make a ros2 workspace, then clone the repo in the src subdir
then build with colcon build

note: astra driver provide two work method, normal and
 filter. with filter driver, get better quality depth 
data but need high-performance platform , like pc.
 if you work in the ARM, suggest to use normal method. 
 you can use -DFILTER=ON / OFF to change the method.
Does this still apply on eloquent???

2. create astra udev rule
$ roscd astra_camera && ./scripts/create_udev_rules
Does this still apply on eloqunt???

3. run astra_camera
source local_setup.bash or setup.bash in the workspace install dir
ros2 run astra_camera astra_camera_node

4. Best way to view camera stream with ROS2? 
ros2 run image_tools showimage ????
