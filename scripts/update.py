import math
from matplotlib.pyplot import axis
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from shapely.geometry import LineString

def recognition_step(array_antiga : np.array, array_recebida:np.array):

    global landmark_positions
    """
    Nmark_antiga=int(array_antiga.shape[0]/4)
    print(Nmark_antiga)
    array_antiga=array_antiga.reshape(Nmark_antiga,4)

    Nmark_recebida=Nmark_antiga=int(array_recebida.shape[0]/4)
    array_recebida=array_recebida.reshape(Nmark_recebida,4)

    for i in range(0,Nmark_recebida):
        p1=((array_recebida[i][0],array_recebida[i][1]))
        p2=((array_recebida[i][2],array_recebida[i][3]))

        line1 = LineString([p1, p2])

        for j in range(0,Nmark_antiga):
            p3=((array_antiga[j][0],array_antiga[j][1]))
            p4=((array_antiga[j][2],array_antiga[j][3]))

            line2 = LineString([p3, p4])
   """

    Npoints_antiga=int(array_antiga.size/2)
    Npoints_recebida=int(array_recebida.size/2)

    array_antiga=array_antiga.reshape(Npoints_antiga,2)
    array_recebida=array_recebida.reshape(Npoints_recebida,2)

    array_nova=np.zeros((Npoints_antiga,2))
    #print(array_antiga)
    #print(array_recebida)

    for i in range(0,Npoints_recebida):
        p1=((array_recebida[i][0],array_recebida[i][1]))

        for j in range(0,Npoints_antiga):
            p2=((array_antiga[j][0],array_antiga[j][1]))
            
            line = LineString([p1, p2])
            #d=p1.distance(p2)
           # print(line.length)
            #print(p1[0])
            if(line.length<20):
                array_nova[j][0]=p1[0]
                array_nova[j][1]=p1[1]
                print("u")
                break
            if(j==Npoints_antiga-1):
                array_nova=np.concatenate([array_nova,[[p1[0],p1[1]]]  ])
                
    print(array_nova)
    #print(array_nova)
    novo_taman=array_nova.shape[0]
    new_landmarks= array_nova[Npoints_antiga:novo_taman] 
    #print(new_landmarks.reshape(int(new_landmarks.size/4),4)) 
     
    new_landmarks= new_landmarks.reshape(int(new_landmarks.size/4),4)
    print(new_landmarks)
    print(".---.")
    print(landmark_positions)

    landmark_positions=np.concatenate([landmark_positions,new_landmarks],axis=0)
    return 0

def store_prediction_callback(msg_toreceive:PoseWithCovariance):
    global positionx, positiony, yaw,matriz_cov

    positionx=msg_toreceive.pose.position.x
    positiony=msg_toreceive.pose.position.y

    orientation_q = msg_toreceive.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    matriz_cov= msg_toreceive.covariance


def update_calculation_callback(msg_toreceive:Float32MultiArray):
    global landmark_positions

    landmark_recebidos=np.array(msg_toreceive.data)
    #landmark_recebidos=landmark_positions.reshape(int(landmark_positions.shape[0]*0.25),4)    
     
    recognition_step(landmark_positions,landmark_recebidos)
    ###---->Transformar tudo em coordenadas cartesianas, as landmarks tem de ter apenas uma coluna

    #landmark_novos=landmark_positions.append(landmark_recebidos)## e substituir com os novos fazendo correspond
    
    #landmark_positions=landmark_positions.append(landmark_recebidos)##apenas os novos
    ###analyse
    #delta = landmark_positions-landmark_novos

    #q=delta.transpose().dot(delta)

    #for i in range(0,landmark_novos.shape[0]): 
        #z_previsto=np.array([ [math.sqrt(q[i])] , [math.atan2(delta[i])]]
         
    
    #rospy.loginfo(landmark_positions)

if __name__== '__main__':
    global positionx, positiony, yaw,matriz_cov,landmark_positions

    matriz_cov=np.ones((3,3))
    iniciado=0
    positionx=0
    positiony=0
    theta=0
    landmark_positions=np.array([[0,0,0,0]])

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    pub=rospy.Publisher("/p3dx/belief_calculation", PoseWithCovariance,queue_size=10)

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_prediction=rospy.Subscriber("/p3dx/prediction_calculation", PoseWithCovariance, callback =store_prediction_callback)#callback means it call a function when it receives information

    rospy.sleep(2)
    sub_update=rospy.Subscriber("/landmarks",Float32MultiArray,callback=update_calculation_callback)
    rospy.spin()#keeps the node alive