import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance
#from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def store_update_callback(msg_toreceive: PoseWithCovariance):
    global positionx, positiony, yaw,iniciado,matriz_cov

    positionx=msg_toreceive.pose.pose.position.x
    positiony=msg_toreceive.pose.pose.position.y
    yaw=msg_toreceive.pose.orientation
    matriz_cov=msg_toreceive.covariance




def belief_calculation_callback(msg_toreceive: Odometry):
    global positionx, positiony, yaw,iniciado,matriz_cov, erx, ery, eth, pose

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

        D=math.sqrt((deltay)**2 + (deltax)**2)
        
        #------------- NOISE -------------------------
        sigma=0.01
        if yaw != yaw2:
            eth = eth + np.random.normal(0.,sigma)

        #erx = erx + np.random.normal(0.,sigma)
        #ery = ery + np.random.normal(0.,sigma)

        alpha=1
        a1=0.055*alpha
        a2=0.001*3.1415/180.0*alpha
        a3=0.005*alpha
        a4=0.01*alpha


        rot1 = math.atan2(deltay, deltax) - yaw
        rot2 = yaw2-yaw-rot1

        sd_rot1 = a1*abs(rot1) + a2*D
        sd_rot2 = a1*abs(rot2) + a2*D
        sd_trans = a3*D + a4*(abs(rot1) + abs(rot2))
 
        D +=  np.random.normal(0,sd_trans*sd_trans)
        rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        rot2 += np.random.normal(0, sd_rot2*sd_rot2)

        pose[0] += D*math.cos(yaw+rot1)
        pose[1] += D*math.sin(yaw+rot1)
        pose[2] +=  rot1 + rot2


        fx = open("./coordinates/positionx.txt", "a")
        fx.write(str(pose[0])+"\n")
        fx.close()
        fy = open("./coordinates/positiony.txt", "a")
        fy.write(str(pose[1])+"\n")
        fy.close()
        fth = open("./coordinates/orientation.txt", "a")
        fth.write(str(pose[2])+"\n")
        fth.close()



        #-------------------------------------------

        
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
        pose[0]=positionx=msg_toreceive.pose.pose.position.x
        pose[1]=positiony=msg_toreceive.pose.pose.position.y

        orientation_q = msg_toreceive.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        pose[2]=yaw
        iniciado=1


def store_ground_truth(msg_toreceive:Odometry):

    real_positionx = msg_toreceive.pose.pose.position.x
    real_positiony = msg_toreceive.pose.pose.position.y
    orientation_q = msg_toreceive.pose.pose.orientation
    real_orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (real_roll, real_pitch, real_yaw) = euler_from_quaternion(real_orientation_list)
    

    rfx = open("./coordinates/real_positionx.txt", "a")
    rfx.write(str(real_positionx)+"\n")
    rfx.close()
    rfy = open("./coordinates/real_positiony.txt", "a")
    rfy.write(str(real_positiony)+"\n")
    rfy.close()
    rfth = open("./coordinates/real_orientation.txt", "a")
    rfth.write(str(real_yaw)+"\n")
    rfth.close()


if __name__== '__main__':
    global positionx, positiony, yaw, matriz_cov, erx, ery, eth, pose


    matriz_cov=np.ones((3,3))
    iniciado=0
    positionx=0
    positiony=0
    theta=0
    erx=0.
    ery=0.
    eth=0.
    pose=[0.,0.,0.]

    fx = open("./coordinates/positionx.txt", "w")
    fx.close()
    fy = open("./coordinates/positiony.txt", "w")
    fy.close()
    fth = open("./coordinates/orientation.txt","w")
    fth.close()
    rfx = open("./coordinates/real_positionx.txt", "w")
    rfx.close()
    rfy = open("./coordinates/real_positiony.txt", "w")
    rfx.close()
    rfth = open("./coordinates/real_orientation.txt", "w")
    rfth.close()

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)

    sub_base_ground_truth=rospy.Subscriber("/p3dx/base_pose_ground_truth", Odometry, callback=store_ground_truth)

    sub_odometry=rospy.Subscriber("/p3dx/odom", Odometry, callback = belief_calculation_callback)#callback means it call a function when it receives information

    sub_update=rospy.Subscriber("/p3dx/update",PoseWithCovariance,callback=store_update_callback)
    
    pub=rospy.Publisher("/p3dx/prediction_calculation", PoseWithCovariance,queue_size=10)


    rospy.spin()#keeps the node alive