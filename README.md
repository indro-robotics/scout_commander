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

Next, in order for the ZED2 camera to be visualized (and used), you must have the ROS2 ZED wrapper package installed into your workspace. Instructions to do this can be found [here](https://www.stereolabs.com/docs/ros2/). Make sure to install all the prerequisets before building your package.  

Once finished, you can build the package using the `colcon build` command:
```
colcon build --packages-select scout_mini_description scout_mini_control
```
## Launching the robot
To launch the robot URDF to be visible in RVIZ, use the `ros2 launch` command:
```
ros2 launch scout_mini_description scout_mini.launch.py
```
