# ROS2 RealSense cameras get point cloud x,y,z coordinates
These are packages for using Intel RealSense cameras with ROS2 and get the point cloud x,y,z coordinates.<br>
For more detail about Intenl RealSense cameras with ROS2. Please see [realsense-ros](https://github.com/intelrealsense/realsense-ros/tree/ros2)

## Enivorment/Equipment
- Ubuntu 20.04
- ROS2-foxy
- Intel® RealSense™ DEPTH CAMERA D435i

## Installation
   (If your already install the enviroment and realsense2-camera. Please jump to the [Fourth step](https://github.com/WZS666/ros2_get_point_cloud_xyz#fourth0).)
   ### First0
   You should download ROS2. For me I'm using [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) on [Ubuntu 20.04](https://releases.ubuntu.com/20.04/).<br>
   ### Second
   - #### Install realsense2_camera
   ```
   sudo apt-get install ros-foxy-realsense2-camera
   ```
   if you are using another ROS2 distro, just change "foxy" to which distro you are using.

   ### Third
   - #### Create a workspace
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
   - #### Clone the ROS2 Intel® RealSense™ Intel from [IntelRealSense,realsense-ros](https://github.com/IntelRealSense/realsense-ros)
   ```
   git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
   ```

   ### Fourth
   - #### Clone the [ROS2_get_point_cloud](https://github.com/) to ```~/ros2_ws/src```
   ```
   cd ~/ros2_ws/src
   git clone https://github.com/WZS666/ros2_get_point_cloud_xyz
   cd ~/ros2_ws
   ```
   ### Fifth
   - #### Install dependencies:
   ```
   sudo apt-get install python3-rosdep -y
   sudo rosdep init
   rosdep update
   rosdep install -i --from-path src --rosdistro foxy -y
   sudo apt purge ros-foxy-librealsense2 -y
   sudo apt-get install ros-foxy-sensor-msgs-py
   ```
   ### Sixth
   - #### Build
   ```
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
   (If you got the error about Intel RealSense SDK 2.0 is missing, please check your SDK from installation [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))
   ### Seventh
   - #### Source
   you should do it at every new terminal
   ```
   source install/local_setup.bash
   ```
   if it pop out the ros2 can't found error please enter the ```source /opt/ros/foxy/setup.bash```

## Usage introduction
- ### First terminal start the camera node in ROS2:
```
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true device_type:=d435
```
- ### Second terminal start to get_point_cloud's x,y,z coordinates.
```
ros2 run ros2_get_point_cloud_xyz get_point_cloud_xyz
```

## Some demo
#### - ROS2 get point cloud xyz coordinates
![ROS2 get point cloud xyz coordinates](https://s8.gifyu.com/images/ezgif-1-ae83e0621d52.gif)

#### - ROS2 get point cloud xyz coordinates with Rviz
![ROS2 get point cloud xyz coordinates with Rviz](https://s8.gifyu.com/images/ezgif-1-ac33fd2f7bf1.gif)

## Reference
https://github.com/intelrealsense/realsense-ros/tree/ros2<br>
https://github.com/ros2/common_interfaces/blob/master/sensor_msgs_py/sensor_msgs_py/point_cloud2.py<br>
https://answers.ros.org/question/379029/how-to-get-xyz-coordinates-of-pointcloud-using-intel-real-sense-d415/<br>
https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html<br>