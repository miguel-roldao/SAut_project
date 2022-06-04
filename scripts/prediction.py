import math
from cv2 import reduce
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion




def belief_calculation_callback(msg_toreceive: Odometry):
    global positionx, positiony, yaw,iniciado,matriz_cov

    if(iniciado!=0):

        #rospy.loginfo(msg_toreceive)
        msg_tosend=PoseWithCovariance()

        #fdx=[]

        msg_tosend.pose.position.x=msg_toreceive.pose.pose.position.x
        msg_tosend.pose.position.y=msg_toreceive.pose.pose.position.y
        msg_tosend.pose.orientation=msg_toreceive.pose.pose.orientation


        coss=math.cos(yaw)#to avoid doing the same calculations
        senn=math.sin(yaw)

        #equivalent ways of calculating total distance by the robot
        #D1=(msg_tosend.pose.position.x-positionx)/math.cos(yaw)
        #D2=(msg_tosend.pose.position.y-positiony)/math.sin(yaw)
        D=math.sqrt((msg_tosend.pose.position.y-positiony)**2 + (msg_tosend.pose.position.x-positionx)**2)
        
        
        dfdx=np.diag((1.,1.,1.))
        dfdx[0][2]=-D*senn
        dfdx[1][2]=D*coss

        dfde=np.zeros((3,2))
        dfde[0][0]=dfde[0][1]=0.5*coss
        dfde[1][0]=dfde[1][1]=0.5*senn
        L=0.2
        dfde[2][0]=dfde[2,1]=1/L

        termo1=dfdx.dot(matriz_cov).dot(dfdx.transpose())
        
        matriz_cov= termo1 + dfde.dot(dfde.transpose())

        #covariance 36 size but use like a 3x3
        msg_tosend.covariance=matriz_cov

        rospy.loginfo(np.linalg.det(matriz_cov))

        #rospy.loginfo(np.linalg.det(np.array(msg_tosend.covariance).reshape((3,3))))

        #save the values
        positionx=msg_toreceive.pose.pose.position.x
        positiony=msg_toreceive.pose.pose.position.y
        orientation_q = msg_toreceive.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        #rospy.loginfo(msg_tosend)
        pub.publish(msg_tosend)
    else:
        positionx=msg_toreceive.pose.pose.position.x
        positiony=msg_toreceive.pose.pose.position.y

        orientation_q = msg_toreceive.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        iniciado=1

if __name__== '__main__':
    global positionx, positiony, yaw,matriz_cov


    matriz_cov=np.ones((3,3))
    iniciado=0
    positionx=0
    positiony=0
    theta=0

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    pub=rospy.Publisher("/p3dx/belief_calculation", PoseWithCovariance,queue_size=10)

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_odometry=rospy.Subscriber("/p3dx/odom", Odometry, callback = belief_calculation_callback)#callback means it call a function when it receives information

    rospy.spin()#keeps the node alive