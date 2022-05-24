#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__== '__main__':

    rospy.init_node("draw_circle")
    rospy.loginfo("Hello from draw circle")
    
    pub=rospy.Publisher("/pioneer/cmd_vel", Twist,queue_size=10)


    #rospy.logwarn("Thos is a warning")
    #rospy.logerr("This is an error")

    #rospy.sleep(1.0)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #publish cmd vel
        msg=Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        pub.publish(msg)
        print(msg)
        rate.sleep()#rate of sleep 1hz



    rospy.loginfo("End of program")