## Important Script
Description of the important script. Scripts not listed here are obsolete and can be ignored.

### manual_camera_interface.py
Python script used manually identify the plaster in the camera stream. You can use this if the uv_camera_node.launch is having difficulties.

### move_preset.py
Python script used to move the robot in preset position with ros command. Make sure the robot is using the ROS or ROSTRAJ program.

### ransac_plane.py
Python script used by traj_planner for finding the plane during trajectory planification

### traj_planner.py
Main python script used for trajectory planification. Make sure the uv_camera_node or manual_camera_interface is used. Use *workcell_planning_execution.launch* to launch this script. 

### uv_image_analysis.py
This script is copy of pa_uv script. Use uv_camera_node.launch instead.
