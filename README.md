### requirement:
```   
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers  ros-melodic-moveit-*    
```  
###command to control robot arm
```   
roslaunch sawyer_gazebo sawyer_world.launch    
rosrun intera_interface enable_robot.py -e    
rosrun intera_interface joint_trajectory_action_server.py  
roslaunch sawyer_moveit_config sawyer_moveit.launch       
```   
### record data:   
```   
roslaunch basics testMoveIt.launch     
```   
### tranform joint angles to pos(clone from pykdl but replace "joint_states" by "robot/joint_states")  
```    
rosrun  pykdl_utils joint_kinematics.py            
```   
