#!/usr/bin/env python
import os
import time
import sys
import re
import matplotlib.pyplot as plt
import glob

# def generate_roscoreCmd(rosport):
#     cmd_rosocre = "roscore -p %d"%(rosport)
#     cmd_bash = "exec bash"
#     cmd = cmd_rosocre + ";"+ cmd_bash
#     return cmd

def generate_world_cmd(rosport,gazeboport):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gazeboport)
    cmd_target = "roslaunch sawyer_gazebo sawyer_world.launch gui:=false"
    cmd_bash = "exec bash"
    cmd = cmd_rosURI + ";" + cmd_gazeboURI + ";"+cmd_target+";"+ cmd_bash
    return cmd

def generate_enable_cmd(rosport,gazeboport):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gazeboport)
    cmd_target = "rosrun intera_interface enable_robot.py -e"
    cmd_bash = "exec bash"
    cmd = cmd_rosURI + ";" + cmd_gazeboURI + ";"+cmd_target+";"+ cmd_bash
    return cmd

def generate_service_cmd(rosport,gazeboport):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gazeboport)
    cmd_target = "rosrun intera_interface joint_trajectory_action_server.py "
    cmd_bash = "exec bash"
    cmd = cmd_rosURI + ";" + cmd_gazeboURI + ";"+cmd_target+";"+ cmd_bash
    return cmd

def generate_moveIt_cmd(rosport,gazeboport):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gazeboport)
    cmd_target = "roslaunch sawyer_moveit_config sawyer_moveit.launch"
    cmd_bash = "exec bash"
    cmd = cmd_rosURI + ";" + cmd_gazeboURI + ";"+cmd_target+";"+ cmd_bash
    return cmd

def generate_collection_cmd(rosport,gazeboport,fileId):
    cmd_rosURI = "export ROS_MASTER_URI=http://localhost:%s"%str(rosport)
    cmd_gazeboURI = "export GAZEBO_MASTER_URI=http://localhost:%s"%str(gazeboport)
    cmd_target = "roslaunch basics testMoveIt.launch stage:=%d"%fileId
    cmd_bash = "exec bash"
    cmd = cmd_rosURI + ";" + cmd_gazeboURI + ";"+cmd_target+";"+ cmd_bash
    return cmd

if __name__ == "__main__":
    rosport_start = 11366#11333
    gazebo_start = 11466
    # worlds_path = ["../worlds/stage_adMap2.world","../worlds/stage_adMap3.world","../worlds/stage_adMap4.world"]
    # worlds_path = ["../worlds/stage_adMap5.world","../worlds/stage_adMap6.world","../worlds/stage_adMap7.world"]
    # worlds_path = ["../worlds/stage_adMap8.world","../worlds/stage_adMap9.world","../worlds/stage_adMap10.world"]
    # worlds_path = ["../worlds/stage_adMap14.world","../worlds/stage_adMap15.world","../worlds/stage_adMap16.world"]
    # worlds_path = ["../worlds/stage_adMap11.world","../worlds/stage_adMap12.world","../worlds/stage_adMap13.world"]
    # worlds_path = ["../worlds/stage3.world"]
    worlds_num = 2 #parallel stage nums
    gazeboport = gazebo_start
    rosport = rosport_start
    for i in range(worlds_num):
        cmd_world = generate_world_cmd(rosport,gazeboport)
        cmd_enable = generate_enable_cmd(rosport,gazeboport)
        cmd_service = generate_service_cmd(rosport,gazeboport)
        cmd_moveIt = generate_moveIt_cmd(rosport,gazeboport)
        cmd_collection = generate_collection_cmd(rosport,gazeboport,i)
        print(cmd_world)
        print(cmd_enable)
        print(cmd_service)
        print(cmd_moveIt)
        print(cmd_collection)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_world)
        time.sleep(3)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_enable)
        time.sleep(3)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_service)
        time.sleep(3)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_moveIt)
        time.sleep(3)
        os.system("gnome-terminal --tab -e 'bash -c \"%s\" '"%cmd_collection)
        time.sleep(3)
        rosport +=1
        gazeboport +=1