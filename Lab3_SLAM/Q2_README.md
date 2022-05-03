# MTRX5700, Assignment 3

This folder contains the simulation framework and skeleton code for Assignment 3.


### Setting up the framework
This code was tested on Ubuntu 18.04 with ROS Melodic and Gazebo 9.0.0.
Please try to match this configuration if you are using your personal computers.

#### Pre-requisites (Same as Assignment 1/2. We have also provided a Virtual Box Image with these steps already done.)
*Note*: These dependencies will also help in future assignments.

1. Install [Ubuntu 18.04](https://ubuntu.com/download/desktop).
2. Install ROS Melodic following the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
3. Install the following dependencies

```bash
sudo apt-get install socat libeigen3-dev libyaml-cpp-dev libboost-dev python-lxml libsoqt4-dev libcoin80-dev libqt4-dev libblas-dev liblapack-dev libqhull-dev python-pip python-catkin-tools python-pymodbus
```

```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-soem ros-melodic-socketcan-interface ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-moveit-python ros-melodic-moveit-sim-controller ros-melodic-moveit-resources ros-melodic-actionlib ros-melodic-derived-object-msgs ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-eigen-conversions ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-control-msgs ros-melodic-controller-interface ros-melodic-controller-manager ros-melodic-dynamic-reconfigure ros-melodic-effort-controllers ros-melodic-force-torque-sensor-controller ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-geometry-msgs ros-melodic-hardware-interface ros-melodic-joint-state-controller ros-melodic-joint-state-publisher ros-melodic-joint-trajectory-controller ros-melodic-message-generation ros-melodic-message-runtime ros-melodic-moveit-core ros-melodic-moveit-fake-controller-manager ros-melodic-moveit-kinematics ros-melodic-moveit-planners-ompl ros-melodic-moveit-ros-manipulation ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-visualization ros-melodic-moveit-simple-controller-manager ros-melodic-pluginlib ros-melodic-realtime-tools ros-melodic-robot-state-publisher ros-melodic-roscpp ros-melodic-sensor-msgs ros-melodic-std-srvs ros-melodic-tf ros-melodic-tf-conversions ros-melodic-tf2-geometry-msgs ros-melodic-tf2-msgs ros-melodic-tf2-ros ros-melodic-trajectory-msgs ros-melodic-urdf ros-melodic-velocity-controllers ros-melodic-xacro
```

```bash
pip install argparse rosdep matplotlib mpmath numpy scikit-learn scipy`
pip install torch==1.5.0+cpu torchvision==0.5.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
```

4. Initialize and update rosdep
```bash
sudo rosdep init
rosdep update
```

#### Setting up catkin workspace
The computers in MXLab have the above configuration. All the depedencies for this code to run have been installed. Please type the following commands in a terminal one after the other.  
1. Source ROS commands
```bash
source /opt/ros/melodic/setup.bash
```
2. Create and initialize new catkin workspace. You may choose any name you like.
Here we chose **`ur5espace`** and it is located in the home directory.  
```bash
mkdir -p ur5espace/src && cd ur5espace
catkin init
```
3. Download this folder, unzip and copy all the subfolders to `ur5espace/src`
```bash
cp -r <location_of _downloaded _folder>/* src/
```

4. Install dependencies using rosdep. Run the following from the folder `ur5espace`
```bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```
6. Build the packages in the catkin workspace  
```bash
catkin build
```
7. Source the workspace
```bash
source devel/setup.bash
```


## Question 2

Running the unit tests:
1. Terminal 1 - run unit tests
```bash
python src/experimental_ass3/Lab3_SLAM/Unit_Tests.py
```

For running the simulated environment: 

Launch 5 terminals with the following.

1. Terminal 1 - roscore
```bash
roscore
```
2. Terminal 2 - Launch the simulated environment in gazebo
```bash
roslaunch turtlebot3_gazebo turtlebot3_stage_2.launch
```
3. Terminal 3 - Launch remote control
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
4. Terminal 4 - Launch the landmark publisher
```bash
python src/experimental_ass3/Lab3_SLAM/Landmark_publisher.py
```
 This will publish the landmarks from the enronment so that your SLAM algorithm can see them.

5. Terminal 5 - Launch the SLAM algorithm
```bash
python src/experimental_ass3/Lab3_SLAM/LandmarkSLAM_students.py
```

Makes sure you have sourced your setup.bash script and export the turtlebot model as burger.

For this question you will complete the code in the LandmarkSLAM_students.py file. Following the comments.
Start with the unit tests before using the simulator.

We will provide a rosbag to use as there is less ways for something to go wrong.
