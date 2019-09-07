#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from intera_core_msgs.msg import JointCommand

def talker():
    rospy.init_node('joint_state_publisher_0')
    pub = rospy.Publisher('/robot/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ["head_pan",'right_j0', 'right_j1', 'right_j2', 'right_j3',
                   'right_j4', 'right_j5', 'right_j6']
    hello_str.position = [-0.000723,-0.20288, 0.61753, -1.74399, -1.32579, 1.09791, -1.74968, 0.51062]
    # hello_str.position = [-1.20675, 0.467, -3.04262, -0.39422, 2.96331, -2.21461, 2.87877]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    time.sleep(10)

    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      rate.sleep()

def talker2():
    rospy.init_node('joint_state_publisher_0')
    pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    hello_str = JointCommand()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.mode  = 1 
    hello_str.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                   'right_j4', 'right_j5', 'right_j6']
    # hello_str.position = [-0.20288, 0.61753, -1.74399, -1.32579, 1.09791, -1.74968, 0.51062]
    # hello_str.position = [-1.20675, 0.467, -3.04262, -0.39422, 2.96331, -2.21461, 2.87877]
    positions =     [[-0.20288, 0.61753, -1.74399, -1.32579, 1.09791, -1.74968, 0.51062], [-0.36308, 0.48712, -1.88532, -1.08158, 1.30146, -1.8188, 0.22098], [-0.39752, 0.44012, -1.93739, -0.98365, 1.25842, -1.75613, 0.24877], [-0.43197, 0.39311, -1.98946, -0.88573, 1.21539, -1.69347, 0.27656], [-0.46642, 0.3461, -2.04154, -0.7878, 1.17236, -1.6308, 0.30435], [-0.50087, 0.29909, -2.09361, -0.68987, 1.12933, -1.56814, 0.33214], [-0.53532, 0.25209, -2.14568, -0.59195, 1.08629, -1.50547, 0.35993], [-0.56976, 0.20508, -2.19776, -0.49402, 1.04326, -1.4428, 0.38772], [-0.60421, 0.15807, -2.24983, -0.3961, 1.00023, -1.38014, 0.41551], [-0.63936, 0.16653, -2.29415, -0.3935, 1.11641, -1.42953, 0.56089], [-0.67451, 0.17499, -2.33848, -0.39091, 1.2326, -1.47893, 0.70627], [-0.70966, 0.18346, -2.3828, -0.38832, 1.34879, -1.52832, 0.85165], [-0.74481, 0.19192, -2.42712, -0.38573, 1.46497, -1.57772, 0.99702], [-0.77996, 0.20038, -2.47145, -0.38314, 1.58116, -1.62711, 1.1424], [-0.81511, 0.20884, -2.51577, -0.38055, 1.69735, -1.67651, 1.28778], [-0.85026, 0.2173, -2.56009, -0.37796, 1.81354, -1.7259, 1.43316], [-0.88541, 0.22576, -2.60442, -0.37536, 1.92972, -1.7753, 1.57853], [-0.92056, 0.23422, -2.64874, -0.37277, 2.04591, -1.82469, 1.72391], [-0.95571, 0.24268, -2.69306, -0.37018, 2.1621, -1.87409, 1.86929], [-0.99086, 0.25114, -2.73739, -0.36759, 2.27828, -1.92348, 2.01467], [-1.02601, 0.2596, -2.78171, -0.365, 2.39447, -1.97288, 2.16004], [-1.06116, 0.26806, -2.82603, -0.36241, 2.51066, -2.02227, 2.30542], [-1.0963, 0.27652, -2.87036, -0.35981, 2.62684, -2.07167, 2.4508], [-1.13145, 0.28498, -2.91468, -0.35722, 2.74303, -2.12106, 2.59618], [-1.1666, 0.29344, -2.959, -0.35463, 2.85922, -2.17045, 2.74155], [-1.20175, 0.30191, -3.00333, -0.35204, 2.9754, -2.21985, 2.88693]]
    hello_str.velocity = []
    hello_str.effort = []
    # pub.publish(hello_str)
    # time.sleep(10)
    step = 0

    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        hello_str.position = positions[step]
        pub.publish(hello_str)

        step +=1
        if(step >= len(positions)):
            break
        
        time.sleep(2)
        rate.sleep()

if __name__ == "__main__":
    talker2()