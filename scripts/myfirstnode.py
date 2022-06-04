#!/usr/bin/env python3
import rospy

if __name__== '__main__':
    rospy.init_node("test_node")

    rospy.loginfo("Hello from test node")
    rospy.logwarn("This is a warning")
    rospy.logerr("This is an error")

    #rospy.sleep(1.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()#rate of sleep 10hz

    rospy.loginfo("End of program")
    
