import math
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



def belief_calculation_callback(msg_toreceive: Odometry):
    global t_final, positionx, positiony, theta

    if(t_final!=0):

        #rospy.loginfo(msg_toreceive)
        msg_tosend=PoseWithCovariance()
        velocity=msg_toreceive.twist.twist.linear.x
        angular_velocity=msg_toreceive.twist.twist.angular.z

        t_inicial=t_final
        t_final=msg_toreceive.header.stamp
        delta_t=(t_final-t_inicial).to_sec()


        positionx=velocity*delta_t*math.cos(theta+math.pi)+positionx
        positiony=velocity*delta_t*math.sin(theta+math.pi)+positiony


        theta=(theta+angular_velocity*delta_t)

        msg_tosend.pose.position.x=positionx
        msg_tosend.pose.position.y=positiony
        msg_tosend.pose.orientation.z=math.cos(theta/2)
        msg_tosend.pose.orientation.w=math.sin(-theta/2)


        #covariance 36 size
        msg_tosend.covariance[0]=2
        
        
        rospy.loginfo(msg_tosend)
        pub.publish(msg_tosend)
    else:
        t_final=msg_toreceive.header.stamp
        positionx=msg_toreceive.pose.pose.position.x
        positiony=msg_toreceive.pose.pose.position.y
        theta=math.acos(msg_toreceive.pose.pose.orientation.z)*2
    

if __name__== '__main__':
    global t_final, positionx, positiony, theta

    t_final=0
    positionx=0
    positiony=0
    theta=0

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    pub=rospy.Publisher("/pioneer/belief_calculation", PoseWithCovariance,queue_size=10)

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_odometry=rospy.Subscriber("/pioneer/odom", Odometry, callback = belief_calculation_callback)#callback means it call a function when it receives information

    rospy.spin()#keeps the node alive