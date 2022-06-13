# velma_kinematics

```bash
roscore
roslaunch velma_description upload_robot.launch
rosrun robot_state_publisher robot_state_publisher
rosrun rviz rviz
rosrun velma_kinematics test_velma_workspace.py
```
Add the following visualizations in rviz:
* RobotModel
* markers on topic /velma_workspace

You can change slice plane in the source code by changing the line:
```Python
slice_coord = 'y'
```
