# JENGABELLS

**Group Members:**
Katie Hughes, Alyssa Chen, Hang Yin, Liz Metzger

This package is meant to turn a Franka Emika Panda arm into a Jenga assistant! It uses computer vision to detect Jenga bricks and place them on top of the tower. The robot plans and executes trajectories using a custom MoveGroup API.

Upon starting the package for the first time the robot needs to be calibrated. This process adds a tf to the tf tree between the panda hand and the camera by using known transforms between the camera and an april tag and the end effector and the base of the robot.

Once calibrated it uses a depth map and computer vision to find the top of the tower, the orientation of the top of the tower, and the table. Once these are determined then the program inters a mode where it scans between the top of the tower and the table to find any pushed out blocks. After finding a block, the centroid is added to the tf tree so that the robot can move to the location of the block. Once the robot has moved to the block it will grab the block, pull it out of the tower, move to the top of the tower, and place the block in the appropriate orientation. Once the block is placed the program goes back into scanning mode and looks for another block.

While scanning the program will not send a block postion if the machine learning model detects that there is a person in the frame. Once it detects 80 frames without detecting anyone then it will start looking for a block to send the position to.

In the main branch the program runs where the robot completes every turn but in the branch turns it is implemented so that the robot takes turns with another player. This can be adjusted by changing the number of players in the turns.yaml from 1 to 2.

How to run:

* Plug into the Franka and the realsense camera.
* `ssh student@station`, then `ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot`
* `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot`
* From the workspace containing our packages, run `ros2 run plan_execute cv_test`
If you need to calibrate:       
   * From your root workspace, run `vcs import --recursive --input src/jengavision/camera/config/jenga_vision.repo src/` to install the necessary april tag packages.
   * Run `ros2 service call /calibrate std_srvs/srv/Empty` to move the robot into the calibration position. Insert the april tag into the grippers.
   * Run `ros2 launch camera jenga_vision.launch.py calibrate:=true`. After you see a message that indicates that calibration is complete, you can CTRL-C. The file `tf.yaml` will be saved in `${root_workspace}/install/camera/share/camera` and will be loaded automatically in future runs. 
   * Run `ros2 service call /ready std_srvs/srv/Empty` to return the robot to the ready position. Remove the april tag from the grippers.
Otherwise:
* Run `ros2 launch camera jenga_vision.launch.py`
* In the pop up window, ensure that the tower is visible in the color frame and make sure that it is inside the bounding square (if not, adjust the size with the trackbars)
* Remove a piece about halfway from the tower (and ensure you can see it from the camera window). When you step away from the table, the robot will grab and place it!