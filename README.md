# Sensor Box ITV

[![N|Solid](http://www.itv.org/wp-content/themes/html5blank/img/logo-vale.svg)](www.itv.org)

Sensor Box ITV is a 'wooden box' to perform data acquisition. This box is provided with:
  - Velodyne 16
  - Axis IP RGB Camera
  - ToF camera o3d3xx

The camera/velodyne calibration was made based on the following paper:
  - Velas, Spanel, Materna, Herout: Calibration of RGB Camera with Velodyne LiDAR (http://www.fit.vutbr.cz/~spanel/pubs.php?file=%2Fpub%2F10578%2FCalibration_of_RGB_Camera_With_Velodyne_LiDAR.pdf&id=10578)
  - Implementation described in http://wiki.ros.org/but_calibration_camera_velodyne and https://github.com/robofit/but_velodyne


### Installation

This tutorial was ment to be executed on a **Ubuntu 16.04 machine with ROS Kinetic**. 

- Clone this repository 

    `git clone https://github.com/h3ct0r/sensor_box_itv`

- Copy this repository to your catkin workspace

- Install OpenCV 3 from apt

    `sudo apt install ros-kinetic-opencv3`

- Install Velodyne libs:

    ```sh
    $ cd /tmp
    $ git clone https://github.com/robofit/but_velodyne_lib
    $ sudo apt install libboost-math-dev
    $ cd but_velodyne_lib
    $ mkdir bin; cd bin
    $ cmake ..
    $ make
    $ sudo make install
    ```

- Install ToF libs:
    ```sh
    $ cd /tmp
    $ git clone https://github.com/lovepark/libo3d3xx
    $  sudo apt install libxmlrpc-c++8-dev libgtest-dev libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev libopencv-contrib-dev libgtkglext1 libgtkglext1-dev
    $ cd libo3d3xx
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
    ```

- Run `catkin_make` from your catkin workspace

- **Check for compilation errors**

### Hardware


Before running the nodes, you need to check first if everything is well connected and powered.

- Connect the blue ethernet cable from the *switch* to the computer (cable label says **ubuntu**)

- Set a ip address at the same network than the other devices:

    `sudo ifconfig eth0 192.168.1.2`
    
- Check if the devices are reachable:

    `ping 192.168.1.200` (Axis)
    `ping 192.168.1.201` (Velodyne)
    `ping 192.168.1.69` (ToF camera)

- If a device does not respond to ping, then check cables, power, etc.

### Running nodes

- After all hardware configurations and packages are installed, just need to edit the launch file to define where to save the ROS Bags and activate/deactivate the respetive bags defining "true/false" on the arguments ["record_velodyne", "record_rgb_camera", "record_tof_camera"]:

    ```sh
    $ roscd sensor_box_itv
    $ vim launch/init_sensors.launch
    ```

- Then finally run:
    `roslaunch sensor_box_itv init_sensors.launch`

If the RViz interface does not open run:
`rosrun rviz rviz`

*Remember to set the correct transformation on RViz to see the pointcloud correctly aligned*

### Todos

 - Write MOAR Tests
 - Add support to color maps

License
----

GPLv3


**Free Software, Hell Yeah!**
