from audioop import ratecv

from sympy import im
import rospy
from geometry_msgs.msg import PoseWithCovariance


def store_pose_callback(msg):
    
    rospy.loginfo(msg)


    


if __name__== '__main__':
    rospy.init_node("Belief_node")
    rospy.loginfo("Hello from belief_odometry")

    
    msg_init=PoseWithCovariance()
    msg_init.pose.position.x=0
    msg_init.pose.position.y=0
    msg_init.pose.orientation.z=0
    pub=rospy.Publisher("/pioneer/pose_belief", PoseWithCovariance,queue_size=10)
    pub.publish(msg_init)

    rospy.loginfo('Inicialização posiçao: ')
    rospy.loginfo(msg_init)


    sub=rospy.Subscriber("/pioneer/belief_calculation", PoseWithCovariance, callback=store_pose_callback)

    rospy.spin()#keeps the node alive