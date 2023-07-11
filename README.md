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
colcon build --packages-select scout_mini_description scout_mini_control scout_mini_gazebo
```

**Note: if building on the physical robot, delete the `scout_mini_gazebo` package, as there will be unmet dependencies**:
```
cd ~/humble_ws/src/scout_commander/
rm -rf scout_mini_gazebo
```



# **Launching the robot in a simulated Gazebo Environment**

When launching the robot in a simulated environment, used for tuning and experimentation with the navigation system without a physical robot, you are going to use the launch files and files in the `scout_mini_gazebo` package. 

To launch the robot, first launch the robot description, with included sensor simulations, with `scout_mini_gazebo` launch file:
```
ros2 launch scout_mini_gazebo scout_viz.launch.py
```
This launches the simulated robot model, the navigation stack is launched in a separate control directory in `scout_mini_control`.


# **Launching the robot in a physical environment**

Due to ROS version limitations with Jetson NX computers, the ZED2 camera node is launched in a native `FOXY` installation, and the navigation and control scripts are launched within a `HUMBLE` docker container. 
#### **NATIVE FOXY** 
From the native `FOXY` installation, we are going to launch the `zed_wrapper` node that publishes all the needed ZED2 camera topics. The SDK and wrapper are already installed on the Jetson. Launch the ZED2 camera node using the following `ros2 launch` command:
```
ros2 launch zed_wrapper zed2.launch.py base_frame:=base_footprint publish_tf:=false publish_map_tf:=false cam_pose:=[0.277812,0.0,0.176212,0.0,0.0,0.0] camera_name:=scout_mini
```
This should bringup all the required topics. 
#### **HUMBLE CONTAINER**
From within the humble container we are going to launch the `microstrain_imu` , the static `TF`, and the `rtabmap` node that will handle all of the visual odometry. 

***Note: visual odometry relies on overlapping pointcloud data, large empty spaces will have less reliable odometry than populated environments***

 The difference between the Gazebo `TF` and the static `TF` being that no Gazebo sensors, environments, or wheel odometry simulations are launched in conjunction. Launching this `scout_bringup` for the desired robot mode is explained below.



## **Launching the robot in MAPPING mode**

To launch the robot in a `mapping` mode in order to create a map of the environment, you will use the `scout_bringup` launch file in the `scout_mini_control` directory. There are several paramaters that can be passed through depending on your mapping requirements.
- `database_file`: this paramater will set the location and name of the map you are going to generate. The map is saved in the form <MAP_NAME>.db. This is a larger file than a typical `.yaml` file because it includes all the point cloud information needed for visual odometry. 
- `rtabmap_args`: This passes through specific arguments relevant to rtabmap. The argument most commonly used in this application is the `--delete_db_on_start` argument. This dictates whether to use the existing map at the `database_file` path or to delete that map and begin anew. 

Launching the robot into mapping mode can be done using the command below:

```
ros2 launch scout_mini_control scout_bringup.launch.py rtabmap_args:=--delete_db_on_start database_file:=/home/indro/colcon_ws/src/scout_commander/scout_mini_control/maps/map.db
```

This will launch the robot in a mapping mode. You can now manually drive the robot around it's environment to create a map. 

***Note: launching the `navigation` stack while the robot is in mapping mode is NOT recommended, as localization and path planning simultaneously require a lot of processing power, and your control nodes are much more likely to crash***

This node will automatically save your environment map into your `database_file`, no need to do any external saving. When the node is stopped, the map will be saved.


## **Launching robot in LOCALIZATION mode**

If you have already created the map above, or have another database map of your environment, you can launch the robot into `localization` mode and pass in that map file. That is done using the `scout_bringup` launch file and passing through the following arguments:
- `localization`: this boolean value (true/false) dictates the mode of the robot. The default value of this is `false` to indicate mapping mode.
- `database_file`: this parameter sets the location and name of the map to use for localization. This value should match the value set during `mapping` mode (If you are planning to use the map you have just created)

Launching the robot into localization mode can be done using the command below:
```
ros2 launch scout_mini_control scout_bringup.launch.py localization:=true database_file:=/home/indro/colcon_ws/src/scout_commander/scout_mini_control/maps/map.db
```
# Launching the NAV2 System
Once you have launched the robot's bringup file, it will be publishing to the `RobotModel`, `TF`, `odom`, and `map` frames, as well as some other `rtabmap` and sensor topics. To begin using the Navigation system, for path planning, object detection, and future automony, you will launch the navigation stack using the `navigation_bringup` launch file. No arguments need to be passed into this node.
```
ros2 launch scout_mini_control navigation_bringup.launch.py
```
## Following a set of waypoints
The navigation system has been set up such that the robot will path plan and then navigate to a series of waypoints given by a **CSV** file. The **CSV** file contains 7 values per row. These values are:

`x_position`, `y_position`, `z_position`, `x_orientation`, `y_orientation`, `z_orientation`, `w_orientation`

An example of this file is given by the `waypoints.txt` file in the `scout_mini_control/maps` directory. 

To command the robot to navigate through the waypoints in your waypoints **CSV**, use the `waypoint_follower` launch file in the `scout_mini_control` directory. 
```
ros2 launch scout_mini_control waypoint_follower.launch.py waypoints_file:=<path_to_waypoints_file>
```

***Note: this command can be sent via the visualization computer, assuming your robot and station are connected over `CYCLONEDDS`*** 

# Visualizing ROBOT over CYCLONEDDS
The physical robot is installed with CYCLONEDDS. Assuming appropriate configurations are set, all robot topics should be visible in your local computers ROS environment. To visualize these topics, you can use the RVIZ configuration specific launch file depending on your robot mode.

**To launch in mapping mode:**
```
ros2 launch scout_mini_control rviz2_mapping.launch.py
```
**To launch in localization mode:**
```
ros2 launch scout_mini_control rviz2_localization.launch.py
```
