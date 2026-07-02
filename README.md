# 25b2180_SOR_assignment-5

<b>for the ARM robot</b><br>
open the ARM ROBOT  package and build and source the package. then launch the GAZEBO and in another terminal type the command below:-<br>
for GAZEBO- ros2 launch bme_ros2_simple_arm spawn_robot.launch.py<br>
for giving the coordinates as the input- ros2 run arm_ik_solver manual_ik_node x y z<br>

here the x,y,z representes the coodinates we want the arm to move to. <br>
after typing the command for the first time , we no need to type the full command again. we can directly give the x,y,z coordinates to the terminal.<br>


<b>for the SLAM navigation</b>
<br>
first build and source the complete package and then run the foloowing commands.<br>
for GAZEBO - ros2 launch bme_ros2_navigation spawn_robot.launch.py<br>
then in another terminal type the code below:-<br>
terminal 2 - ros2 run bme_ros2_navigation_py waypoint_patrol<br>

the coordinates to which the bot should move are already inserted in the waypoint.py file.