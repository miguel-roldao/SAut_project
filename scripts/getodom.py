"""function getOdometry()
	local result, wL, wR, thetaL, thetaR
"""
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry


if __name__== '__main__':
    global positionx, positiony, yaw,matriz_cov

    rospy.init_node("get_odom")
    rospy.loginfo("Hello from get_odom")

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_odometry=rospy.Subscriber("/p3dx/odom", Odometry)
    odom = Odometry()

    rospy.loginfo(odom.pose.pose.position.x)
    rospy.spin()#keeps the node alive