#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import random
import numpy
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import DisplayTrajectory

class MoveGroupCom(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        self.status_subsriber = rospy.Subscriber("/move_group/status", GoalStatusArray, self.goalStatus_callback)
        self.status = 1
        self.goalId = 0
        self.lastGoalId = -1
        self.reachGoal = False

        self.trajectory_subscriber = rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory,self.trajectory_callback)
        self.start = [0,0,0,0,0,0,0]
        self.startPose = [0,0,0]
        self.traectories = None
        self.end = [0,0,0,0,0,0,0]
        self.endPose = [0,0,0]

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

    def reach_jointGoal(self,my_goal):
        # We can get the joint values from the group and adjust some of the values:
        self.reachGoal = False
        joint_goal = self.move_group.get_current_joint_values()
        self.setStart(joint_goal)
        joint_goal[0] = my_goal[0] #0
        joint_goal[1] = my_goal[1] #-pi/4
        joint_goal[2] = my_goal[2] #0
        joint_goal[3] = my_goal[3] #-pi/2
        joint_goal[4] = my_goal[4]#0
        joint_goal[5] = my_goal[5]#pi/3
        joint_goal[6] = my_goal[6]#0
        self.setEnd(joint_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        #qinjielin: clear targets after palnning with joint
        # self.move_group.clear_joint_value_targets()
    
    def setStart(self,joint_goal):
        for i in range(7):
            self.start[i] = joint_goal[i]
    
    def setEnd(self,joint_goal):
        for i in range(7):
            self.end[i] = joint_goal[i]
    
    def reach_xyzGoal(self,random_goal_value):
        self.reachGoal = False

        self.set_StartState()
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = random_goal_value[0]#0.2
        pose_goal.position.y = random_goal_value[1]#-0.4
        pose_goal.position.z = random_goal_value[2]#0.4

        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # Storing the end state
        self.set_endState()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def goalStatus_callback(self,msg):
        self.status = msg.status_list[0].status
        self.goalId = msg.status_list[0].goal_id.stamp.secs
        if (not (self.goalId == self.lastGoalId)):
            if(self.status == 3):
                self.lastGoalId = self.goalId
                self.reachGoal=True
                print("**************** reach new goal *****************")
    
    def get_traj(self):
        traj = None
        points = None
        while points is None:
            try:
                print("############getting plan#############")
                points = rospy.wait_for_message("/move_group/display_planned_path",DisplayTrajectory,timeout=5).trajectory[0].joint_trajectory.points
            except:
                pass
        for p in points:
            traj.append(p.positions)
        return traj
 
    def trajectory_callback(self,msg):
        print("############### get new plan #################")
        self.traectories = []
        points = msg.trajectory[0].joint_trajectory.points
        for p in points:
            self.traectories.append(p.positions)
    
    def add_obs0(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.2 # slightly above the end effector
        box_pose.pose.position.x = 0.3 # slightly above the end effector
        box_name = "box0"
        self.scene.add_box(box_name, box_pose, size=(0.5, 0.5,0.1))
        time.sleep(1)

    def add_obs1(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.0 # slightly above the end effector
        box_pose.pose.position.x = 0.5 # slightly above the end effector
        box_name = "box1"
        self.scene.add_box(box_name, box_pose, size=(0.4, 0.4,0.1))
        time.sleep(1)

    def add_obs2(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.0 # slightly above the end effector
        box_pose.pose.position.x = 0.5 # slightly above the end effector
        box_name = "box2"
        self.scene.add_box(box_name, box_pose, size=(0.5, 0.1,0.2))
        time.sleep(1)

    def delete_obs(self):
        self.scene.remove_world_object("box2")
    
    def set_StartState(self):
        joint_goal = self.move_group.get_current_joint_values()
        wpose = self.move_group.get_current_pose().pose
        self.startPose = [wpose.position.x,wpose.position.y,wpose.position.z]
        self.setStart(joint_goal)

    def set_endState(self):
        joint_goal = self.move_group.get_current_joint_values()
        wpose = self.move_group.get_current_pose().pose
        self.endPose = [wpose.position.x,wpose.position.y,wpose.position.z]
        self.setEnd(joint_goal)

    def get_currentPose(self):
        wpose = self.move_group.get_current_pose().pose
        return [wpose.position.x,wpose.position.y,wpose.position.z]

        
def record_trajectory(start,end,traj):
    end = list(numpy.around(numpy.array(end),5))
    start = list(numpy.around(numpy.array(start),5))
    traj =  (numpy.around(numpy.array(traj),5)).tolist()
    with open('/home/qinjielin/Documents/dataset/roboArm/data_2.txt', 'a') as f:
        f.write("start: "+str(start)+"\n")
        f.write("tajectories: "+str(traj)+"\n")
        f.write("end: "+str(end)+"\n")
    f.close()

def record_poses(pose1,pose2):
    pose1 = list(numpy.around(numpy.array(pose1),5))
    pose2 = list(numpy.around(numpy.array(pose2),5))
    with open('/home/qinjielin/Documents/dataset/roboArm/data_Pose.txt', 'a') as f:
        f.write("start pose: "+str(pose1)+"\n")
        f.write("end pose: "+str(pose1)+"\n")
    f.close()

def generate_pose(Posi):
    target = [0,0,0]#x,y,z
    if(Posi):
        target[1] = random.uniform(0.4,0.8)
    else:
        target[1] = random.uniform(-0.4,-0.8)
    target[0] = random.uniform(0.0,0.8)
    target[2] = random.uniform(-0.5,0.5)
    return target

if __name__ == "__main__":
    my_arm = MoveGroupCom()
    terminal = 0
    trajectoryNum = 3000 + 2
    target = None

    # my_arm.delete_obs()
    my_arm.add_obs1()
    time.sleep(10)

    while(terminal < trajectoryNum):
        # random_joint_value = numpy.random.uniform(-pi+1, pi-1, size=(7, ))
        # my_arm.reach_jointGoal(random_joint_value)

        if(terminal%2 == 0):
            target = generate_pose(True)
        else:
            target = generate_pose(False)
        print("try target pose:",target)

        my_arm.reach_xyzGoal(target)
        time.sleep(4)
        if(my_arm.reachGoal):
            terminal+=1
            start = list(my_arm.start)
            end = list(my_arm.end)
            traj = list(my_arm.traectories)
            # sychronize the trajectory
            if(terminal > 1):
                print("record %d data"%terminal)
                record_trajectory(start,end,traj)
                record_poses(list(my_arm.startPose),list(my_arm.endPose))
        
