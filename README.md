# rcm_poncage

rcm_poncage is a robotics project created by master degree engineering students at the University of Sherbrooke for [RCM Modulaire](https://www.rcmgroupe.com/). The goal of the project is to automaticaly sand plaster on construction walls.


## Architecture
For now, the project is divided in 4 main packages/folders:
1. pa_ctrl: ROS package that stand for Ponçage Automatisé control (Up comming)
2. pa_rgb: ROS package that stand for Ponçage Automatisé par caméra RGB
3. pa_tof: ROS package that stand for Ponçage Automatisé par caméra Time Of Flight (In progress)
4. pa_uv: ROS package that stand for Ponçage Automatisé par fluorescence UV capté par caméra RGB (In progress)

And 2 side packages to enable the main ones:

5. camera_control_msgs: ROS package used to to adapt [Helios Lucid 2 ROS driver](https://support.thinklucid.com/using-ros-for-linux/?gclid=EAIaIQobChMIvf7E3MHB9wIVsnRvBB3k_gAmEAAYASAAEgKesvD_BwE)
6. ur_adapted_launch: ROS package used to customize the UR5e's [URDF](https://gramaziokohler.github.io/compas_fab/0.21.0/examples/03_backends_ros/07_ros_create_urdf_ur5_with_measurement_tool.html)

## Installation

### Python 3 on Ubuntu 20.04
1. Make shure python 3 is installed on your pc (it should be by default)
```bash
python3 -V
```
2. If not, install it
```bash
sudo apt install -y python3-pip
```

### ROS Noetic on Ubuntu 20.04
1. Follow the instructions on the offical website ([ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu))
2. If you are not familiar with ROS, we strongly recommend that you do the tutorials ([ROS Tutorials](http://wiki.ros.org/ROS/Tutorials))


### ROS Libraries

The list of ROS libraries used in the project is as follows:
- [pcl_ros](http://wiki.ros.org/pcl_ros) (To interpret points_clouds through ROS)
- [velodyne](http://wiki.ros.org/velodyne) (To interface with the [Velodyne VLP16 lidar](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16))
- [librealsense2](http://wiki.ros.org/librealsense2) (To interface with the D435i camera)
- [realsense2_camera](http://wiki.ros.org/realsense2_camera) (To use Realsense camera through ROS)
- [vision_opencv](http://wiki.ros.org/vision_opencv) (To enable the perception algorithmes)
- [Robot_localization](http://wiki.ros.org/robot_localization) (To fuse sensors)
- [imu_filter_madgwick](http://wiki.ros.org/imu_filter_madgwick) (If the imu on the D435i camera is used to update it's tf)
- ~~[ur_robot_driver](http://wiki.ros.org/ur_robot_driver) (To inteface with the UR5e robot) -> Has to be install seperatly~~ Changed UR robot to FANUC
- [moveit](https://moveit.ros.org/documentation/concepts/) (To plan and execute movements)
- [jsk_rviz_plugins](https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/index.html) (To display more informations in rviz)
- [Git](https://github.com/git-guides/install-git) and [Git lfs](https://git-lfs.com/) (To download large files)
- open3d

1. ~~Switch for a [real time kernel](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md) since the ur_robot_driver requires it.~~ Changed UR to FANUC

2. ~~Follow instructions from [ur_robot_diver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/README.md#building)'s github to install this package.~~ Changed UR to FANUC

3. To install all other ROS libraries (except ur_robot_driver) needed in one command:
```bash
sudo apt-get install ros-noetic-pcl-ros ros-noetic-velodyne ros-noetic-librealsense2 ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-vision-opencv ros-noetic-robot-localization ros-noetic-imu-filter-madgwick ros-noetic-jsk-rviz-plugins
```

### Lucid ArenaSDK (used for Helios camera)

1. Download the [ArenaSDK](https://thinklucid.com/arena-software-development-kit/) on the [download page](https://thinklucid.com/downloads-hub/) (You will need to create an account and login)
2. Unzip the folder in your `home` repository
3. Go in the folder, there should be an other folder with the same name and a readme, put the readme in the folder, rename it to `ArenaSDK_Linux_x64` and cut/paste it in your `home` repository.
4. Delete the now empty repository.
5. Go in the `ArenaSDK_Linux_x64` folder and run the config
```bash
cd ~/ArenaSDK_Linux_x64/
sudo sh Arena_SDK_Linux_x64.conf
```
6. Update the `image_encoding.h` to allow ArenaSDK's custom messages
```bash
cd 
sudo cp /opt/ros/noetic/include/sensor_msgs/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h.bak
sudo cp ~/ABS_GIT_PATH/rcm_poncage/pa_tof/include/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h
```
Note: Checkout [Lucid's guide for use with linux](https://support.thinklucid.com/using-ros-for-linux/) for more information.

### Configuring your Catkin Workspace and Installation

0. Create a [Personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) to allow remote working on the github repository
1. Make shure that the end of your `.bashrc` file in your `home` folder has the following lines
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ARENA_ROOT=~/ArenaSDK_Linux_x64
```
2. Create a new folder named rcm_poncage in your catkin/src
3. clone this repo somewhere on your system (you will need a personal acces token):
```bash
git clone https://github.com/introlab/rcm_poncage.git
```
4. Make a sym-link of all the files in the git folder to the ros folder:
```bash
cp -rs ~/ABS_GIT_PATH/rcm_poncage/* ~/ABS_ROS_PATH/catkin/src/rcm_poncage/
```
5. Compile the project:
```bash
cd catkin_ws
catkin_make
```
Note: Restart terminals to apply `.bashrc` changes.

### Configuring your ethernet conection to speek with the robot and the lidar
1. Make shure that the pc, the robot and the lidar are all connected in the same ethernet switch.
2. Setup static IP adress on the pc:

|    Hardware   |       IP      |      Mask     |
| ------------- | ------------- | ------------- |
| PC -> Velodyne lidar | 192.168.1.100  | 255.255.0.0 |
| PC -> CRX-20ia/L | 192.168.56.1 | 255.255.0.0 |
| Velodyne lidar | 192.168.1.201 | 255.255.0.0 |
| CRX-20ia/L | 192.168.56.1 | 255.255.0.0 |
| Helios2 | Local_link | 255.255.0.0 |
3. Try to ping the hardware:
```bash
ping 192.168.1.201
ping 192.168.56.101
```
4. Open a web browser and entering `192.168.1.201` should display the lidar's web setup page.


## Usage
Before using the CRX robot, make shure that:

1. It is properly setup with the right IP adress, 
2. The robot is powered up and clear fault
3. The program runing is ROS or ROSTRAJ

### Using the project with force controle
Using the robot to sand a UV exposed joint

1. Make sure the space around the robot is clear and safe
2. Move the robot in front of the wall and align the camera with the joint
3. Run the ROS program on the CRX
4. Launch the workcell planning
```bash
roslaunch pa_ctrl workcell_planning_execution.launch sim:=false robot_ip:=192.168.56.1 use_bswap:=false
```
5. Make sure the camera is connected and launch uv analysis
```bash
roslaunch pa_uv uv_camera_node.launch
``` 
6. Verify the boundary rectangle is aligne with the actual joint
7. Send topic to start path planning and execution
```bash
rostopic pub -1 /console_traj_planner std_msgs/Empty
```

### Using the with move preset
Using ros to move the robot

1. Make sure the space around the robot is clear and safe
2. Launch the move_preset script
```bash
rosrun pa_ctrl move_preset.py
```
3. Send the empty message to the topic corresponding to the chosen to reach
```bash
rostopic pub -1 /console_chosen_position std_msgs/Empty
```

### Using the project with the rgb package
```bash
roslaunch pa_rgb acqui_pict.launch
```
To launch with all 3 parts (robot, lidar, camera). If they are not all connected/wanted use:
```bash
robot:=false
lidar:=false
camera:=false
```
Directly after the `roslaunch` line in combination depending on your hardware setup. The right Rviz config should pop up a few secondes after the launch.

Each time a marker is clicked MoveIt should display the plan to reach the target. To execute it with the robot. press `play` in the `External control` programme on the UR5e teach and press the `publish` button in the `execute plan` tab in Rviz. **WARNING**: make shure the robot is free to move before executing the command.

To save pictures, press `publish` button in the `save images` tab in Rviz. If a different setup of images are wanted, update the `callbackSaveImage` in the [`video_to_cv_images.py`](https://github.com/introlab/rcm_poncage/blob/main/pa_rgb/src/video_to_cv_images.py) file.

### Using the project with the tof package
```bash
roslaunch pa_tof tof_camera_node.launch
```

### Using the project with the uv package
```bash
roslaunch pa_uv uv_camera_node.launch
```
TODO : In progress

### Update the urdf
**WARNING**: Update with actual coupling before doing any physical applications.

To update the urdf:
1. Make the wanted modification(s) notably add the new stl files (collision and apperance) in the [folder](https://github.com/introlab/rcm_poncage/tree/main/pa_rgb/rviz) and update the name in the [xacro file](https://github.com/introlab/rcm_poncage/blob/main/pa_rgb/rviz/festool_kit.xacro).
2. Go in your folder
```bash
cd /ABS_GIT_PATH/rcm_poncage/pa_rgb/rviz/
```
3. Update the URDF
```bash
rosrun xacro xacro --inorder -o ur5e_with_festool_kit.urdf ur5e_with_festool_kit.xacro
```
4. Check if URDF is valid
```bash
check_urdf ur5e_with_festool_kit.urdf
```
It should look something like:
```bash
robot name is: ur5e_with_festool_kit
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  base
        child(2):  base_link_inertia
            child(1):  shoulder_link
                child(1):  upper_arm_link
                    child(1):  forearm_link
                        child(1):  wrist_1_link
                            child(1):  wrist_2_link
                                child(1):  wrist_3_link
                                    child(1):  flange
                                        child(1):  tool0
                                            child(1):  festool_kit
                                                child(1):  tcp
```



