import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion


def store_update_callback(msg_toreceive: Float32MultiArray):
    global positionx, positiony, yaw,iniciado,matriz_cov

    arr=np.array(msg_toreceive.data)
    #n=msg_toreceive.layout.dim[0]
    n=msg_toreceive.layout.data_offset
    

    positionx=arr[0]
    positiony=arr[1]
    yaw=arr[2]
    cov=arr[n:].reshape(n,n)
    cov=cov[0:3,0:3]
    print(cov)
    matriz_cov= cov
    



def belief_calculation_callback(msg_toreceive: Odometry):
    global positionx, positiony, yaw,iniciado,matriz_cov

    if(iniciado!=0):

        #rospy.loginfo(msg_toreceive)
        msg_tosend=PoseWithCovariance()

        #fdx=[]

        msg_tosend.pose.position.x=msg_toreceive.pose.pose.position.x
        msg_tosend.pose.position.y=msg_toreceive.pose.pose.position.y
        msg_tosend.pose.orientation=msg_toreceive.pose.pose.orientation

        orientation_q = msg_toreceive.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw2) = euler_from_quaternion (orientation_list)


        coss=math.cos(yaw)#to avoid doing the same calculations
        senn=math.sin(yaw)

        #equivalent ways of calculating total distance by the robot
        #D1=(msg_tosend.pose.position.x-positionx)/math.cos(yaw)
        #D2=(msg_tosend.pose.position.y-positiony)/math.sin(yaw)
        deltay=msg_tosend.pose.position.y-positiony
        deltax=msg_tosend.pose.position.x-positionx

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

        eleft=0.01
        eright=0.01
        #edist=(eleft+eright)/2*D
        exx=abs(deltax)*(eleft+eright)*0.5
        eyy=abs(deltay)*(eleft+eright)*0.5
        eyaw=abs(yaw2-yaw)*(eleft+eright)/L
        Rt=np.diag((exx,eyy,eyaw))
        
        matriz_cov= termo1 + dfde.dot(dfde.transpose()).dot(Rt)

        #covariance 36 size but use like a 3x3
        #print(matriz_cov)
        msg_tosend.covariance= np.concatenate( [matriz_cov.flatten(),np.zeros(27)])
        
        #rospy.loginfo(np.linalg.det(matriz_cov))

        #rospy.loginfo(np.linalg.det(np.array(msg_tosend.covariance).reshape((3,3))))

        #save the values
        positionx=msg_toreceive.pose.pose.position.x
        positiony=msg_toreceive.pose.pose.position.y
        yaw=yaw2

        
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


    matriz_cov=np.zeros((3,3))
    iniciado=0
    positionx=0
    positiony=0
    theta=0

    rospy.init_node("Update_calculation")
    rospy.loginfo("Hello from update_calculation")

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_odometry=rospy.Subscriber("/p3dx/odom", Odometry, callback = belief_calculation_callback)#callback means it call a function when it receives information

    sub_update=rospy.Subscriber("/p3dx/update",Float32MultiArray,callback=store_update_callback)
    
    pub=rospy.Publisher("/p3dx/prediction_calculation", PoseWithCovariance,queue_size=10)

    rospy.spin()#keeps the node alive