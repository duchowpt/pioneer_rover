#!/usr/bin/env python  
#finds the transform between /map and /base_link and rebroadcasts it
import rospy
import tf
import geometry_msgs.msg

def rebroadcast():

    rospy.init_node('position_rebroadcaster')

    listener = tf.TransformListener()    
    rate = rospy.Rate(5) # 5hz
    pub = rospy.Publisher('adjusted_pose', geometry_msgs.msg.Pose, queue_size=10)

    while not rospy.is_shutdown():
        pose = geometry_msgs.msg.Pose();
        try:
            (loc, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0) )

            pose.position.x = loc[0]
            pose.position.y = loc[1]
            pose.position.z = loc[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            
            pub.publish(pose)
        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException, TypeError) as e: 
            rospy.logwarn(e)

        rate.sleep()

if __name__ == '__main__':
    try:
        rebroadcast()
    except rospy.ROSInterruptException:
        pass
