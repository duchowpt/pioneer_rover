#!/usr/bin/env python  
####################################
#Runs on the robot, implements control policy
#
#############################
import rospy
import numpy as np
import rospkg
from glob import glob
import geometry_msgs.msg
import actionlib
import move_base_msgs.msg
from math import exp, atan2
from tf.transformations import quaternion_from_euler

state = [0, 0, 0, 0, 0, 0, 0, 0]
def robot_state_callback(data):
    state[4] = data.x
    state[5] = data.y
    state[6] = data.z
    state[7] = data.w

def poi_state_callback(data):
    state[0] = data.x
    state[1] = data.y
    state[2] = data.z
    state[3] = data.w

def sigmoid(x):
    return 1/(1+exp(-x))

sigmoid_vec = np.vectorize(sigmoid)

def calc_next_waypoint(W1, W2):#returns dx and dy
    v1 = np.append(np.array(state),1)
    v2 = W1.dot(v1)
    v3 = sigmoid_vec(v2)
    v4 = np.append(v3, 1)
    v5 = W2.dot(v4)
    v6 = sigmoid_vec(v5)
    return v6[0], v6[1]

def make_goal(dx, dy):
    theta = atan2(dy,dx)

    goal = move_base_msgs.msg.MoveBaseGoal()

    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.pose.position.x = dx
    goal.target_pose.pose.position.y = dy
    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal

def load_weights():
    #find and load the weight matrices
    rospack = rospkg.RosPack()
    weights_path = rospack.get_path('pioneer_rocon_test')+'/weights/'
    w1path = glob(weights_path+'weightMat1_*.csv')[0]
    w2path = glob(weights_path+'weightMat2_*.csv')[0]

    W1 = np.loadtxt(open(w1path, 'rb'), delimiter=',')
    W2 = np.loadtxt(open(w2path, 'rb'), delimiter=',')

    return W1, W2

def run():
    rospy.init_node('agent')

    #I just need these to exist so they can be flipped
    rospy.Publisher('other_agent_poses', geometry_msgs.msg.PoseArray, queue_size=10)
    rospy.Subscriber('other_agent_poses', geometry_msgs.msg.PoseArray)
    rospy.Publisher('quadrant_values', geometry_msgs.msg.Quaternion, queue_size=10)
    #these are needed to navigate
    rospy.Subscriber('quadrant_values', geometry_msgs.msg.Quaternion, robot_state_callback)
    rospy.Subscriber('poi_state', geometry_msgs.msg.Quaternion, poi_state_callback)
    
    #find and load the weight matrices
    W1, W2 = load_weights()

    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()

    rospy.loginfo("Waiting for hub to give state and poi info")
    while not any(state[0:4]) or not any(state[4:8]):#wait for the subscribers to start getting stuff
        rospy.sleep(.5)
    rospy.loginfo("State and poi info received")

    while not rospy.is_shutdown():
        dx, dy = calc_next_waypoint(W1, W2)

        g = make_goal(dx, dy)
        client.send_goal(g);
        client.wait_for_result();



def test():
    rospy.init_node('agent')
    W1, W2 = load_weights()
    print W1
    print W2
    # dx, dy = 1, 1
    # client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction) 
    # client.wait_for_server()
    # g = make_goal(dx, dy)
    # client.send_goal(g)
    # client.wait_for_result()
    # print client.get_result()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
 