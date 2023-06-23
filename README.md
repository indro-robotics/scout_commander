# scout_commander
Repository for NAV2 integration of a scout mini with a Commander SLIM, IMU, and Zed2 Camera.
# Installation
To install the package, first navigate to your colcon workspace src folder, and clone the repository.
```
cd <colcon_ws>/src
git clone https://github.com/indro-robotics/scout_commander.git
```
Next, we are going to need to install all the dependencies needed to run the package. Navigate to the root of your colcon workspace, and run the rosdep commands. 

**Note: You must ensure that you have the rosdep package installed. If you receive an error saying it isn't installed, run `sudo apt install python3-rosdep2`**
```
cd <colcon_ws>/
rosdep init #Only to be done the first time using dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```
If all the dependencies were succesfully installed, you should receive the following message:
```
#All required rosdeps installed successfully
```

Next, in order for the ZED2 camera to be visualized (and used), you must have the ROS2 ZED wrapper package installed into your workspace. Instructions to do this can be found [here](https://www.stereolabs.com/docs/ros2/). Make sure to install all the prerequisites before building your package.  

Once finished, you can build the package using the `colcon build` command:
```
colcon build --packages-select scout_mini_description scout_mini_control
```
## Launching the robot
To launch the robot in this package, there are two main launch file methods.

### **Launching the robot in a simulated Gazebo Environment**

When launching the robot in a simulated environment, used for tuning and experimentation with the navigation system without a physical robot, you are going to use the launch files and files in the `scout_mini_gazebo` package. 

To launch the robot, first launch the robot description, with included sensor simulations, with `scout_mini_gazebo` launch file:
```
ros2 launch scout_mini_gazebo scout_viz.launch.py
```
This launches the simulated robot model, the navigation stack is launched in a separate control directory in `scout_mini_control`.


### **Launching the robot in a physical environment**

Due to ROS version limitations with Jetson NX computers, the ZED2 camera node is launched in a native `FOXY` installation, and the navigation and control scripts are launched within a `HUMBLE` docker container. 
#### **NATIVE FOXY** 
From the native `FOXY` installation, we are going to launch the `zed_wrapper` node that publishes all the needed ZED2 camera topics. The SDK and wrapper are already installed on the Jetson. Launch the ZED2 camera node using the following `ros2 launch` command:
```
ros2 launch zed_wrapper zed2.launch.py base_frame:=base_footprint publish_tf:=true cam_pose:=[0.277812,0.0,0.176212,0.0,0.0,0.0] camera_name:=scout_mini
```
This should bringup all the required topics. 
#### **HUMBLE CONTAINER**
From within the humble container we are going to launch the `microstrain_imu` , the `EKF_node`, and the static `TF`.

 The difference between the Gazebo `TF` and the static `TF` being that no Gazebo sensors or wheel odometry are launched in conjunction. 

Use the the launch file in the `scout_mini_control` package.
```
ros2 launch scout_mini_control scout_bringup.launch.py
```

This node launches the static `TF` publisher, the `EKF` node, and the `IMU` launch node. The appropriate topics are already set in the configurations and these nodes will publish all the `ROS2 Humble` nodes needed for navigation.

To launch the Navigation 2 stack to begin SLAM, after the above two nodes have been started, we are going to launch our `nav2.launch.py` file:
```
ros2 launch scout_mini_control nav2.launch.py
```

## Visualizing ROBOT over CYCLONEDDS
The physical robot is installed with CYCLONEDDS. Assuming appropriate configurations are set, all robot topics should be visible in your local computers ROS environment. To visualize these topics, use the `rviz2.launch.py` file in the `scout_mini_control` package:
```
ros2 launch scout_mini_control rviz2.launch.py
```

This launches RVIZ using the appropriate configuration to visualize all the required nodes. 
