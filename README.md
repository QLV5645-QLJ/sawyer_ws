roslaunch sawyer_gazebo sawyer_world.launch    
rosrun intera_interface enable_robot.py -e    
rosrun intera_interface joint_trajectory_action_server.py  
roslaunch sawyer_moveit_config sawyer_moveit.launch       
roslaunch basics testMoveIt.launch     