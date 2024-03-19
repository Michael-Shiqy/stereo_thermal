## Pre-request:
<ol>
    <li>Ubuntu 22.04 LTS</li>
    <li>ZED SDK v4.0 or later</li>
    <li>CUDA dependency</li>
    <li>ROS 2 Humble</li>
    <li>Rviz2</li>
</ol>


## 1. Follow the ZED * ROS2 tutorial 
https://www.stereolabs.com/docs/ros2

Just the part "Installation"

## 2. Build the color rendering node:
1. Open a new terminal and navigate to the folder test_ws 
2. Check missing dependencies using command:
    
        rosdep install -i --from-path src --rosdistro humble -y

3. Build the new package

        colcon build --packages-select pc_sub 


## 3. Run the color rendering node:
1. Open a new terminal, navigate to test_ws, and source the setup files:

        . install/setup.bash

2. Run the new node (package name: pc_sub, executable name: pc_sub)

        ros2 run pc_sub pc_sub


## 4. Run the ZED pointcloud publisher node
https://www.stereolabs.com/docs/ros2

Just the part "Starting the ZED node" (In a new terminal of course)

## 5. Visualization of the data:
1. Open a new terminal and use command:

        rviz2

2. In the graphic interface of rviz, click "add" on the left bottom corner and then "by topic", select the topic with type "PointCloud2" and press OK.

## 6. Modify the color rendering node

Edit the code file *test_ws/src/pc_sub/src/pu_sub.cpp*, the part to be modified is commented in the code. 

### Note: 
1. Type sensor_msgs::msg::PointCloud2 is the the raw message transfered between nodes
2. Type pcl::PointCloud\<pcl::PointXYZRGB\> is more user-friendly, which is exactly what to be modified. 


