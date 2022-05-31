#!/usr/bin/env python3
import time
import rospy
from geometry_msgs.msg import Twist

if __name__== '__main__':

    rospy.init_node("draw_circle")
    rospy.loginfo("Hello from draw circle")
    
    pub=rospy.Publisher("/pioneer/cmd_vel", Twist,queue_size=10)

    
    
    #rospy.logwarn("Thos is a warning")
    #rospy.logerr("This is an error")

    #rospy.sleep(1.0)

    rate = rospy.Rate(5)

    t0=time.time()
    check=1

    while not rospy.is_shutdown() and time.time()-t0<60:
        rospy.loginfo(time.time()-t0)
        #publish cmd vel
        msg=Twist()
        msg.linear.x = -0.4
        msg.angular.z = 0.0

        if check == 1:
            t0=time.time()
            check=0

        pub.publish(msg)
        #print(msg)
        rate.sleep()#rate of sleep 1hz



    rospy.loginfo("End of program")