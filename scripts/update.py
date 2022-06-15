import math
from cv2 import sqrt
from matplotlib.pyplot import axis
import numpy as np
from sympy import eye
from torch import block_diag
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from shapely.geometry import LineString
from scipy import linalg

def recognition_step(array_antiga : np.array, array_recebida:np.array):

    global landmark_positions,matriz_covL

    global state_vector

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

    array_recebida[:,0]=array_recebida[:,0]+positionx
    array_recebida[:,1]=array_recebida[:,1]+positiony

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
            if(line.length<0.20):
                array_nova[j][0]=p1[0]
                array_nova[j][1]=p1[1]
                #print("u")
                break
            if(j==Npoints_antiga-1):
                array_nova=np.concatenate([array_nova,[[p1[0],p1[1]]]  ])
                
    #print(array_nova)
    #print(array_nova)
    #print(array_recebida)
    novo_taman=array_nova.shape[0]

    

    new_landmarks= array_nova[Npoints_antiga:novo_taman] 
    #print(new_landmarks.reshape(int(new_landmarks.size/4),4)) 

    #print(new_landmarks)
    #print("!!")
    #print(state_vector)
    #print(new_landmarks)

    #new_landmarks=new_landmarks.reshape(int(new_landmarks.size*2),1)
    state_vector=np.concatenate([state_vector,new_landmarks.reshape(int(new_landmarks.size),1)]) 

    new_landmarks= new_landmarks.reshape(int(new_landmarks.size/4),4)
    #print(new_landmarks)
    #print(".---.")
    #print(landmark_positions)
    
    cov=matriz_covL  ### Adicionar as novas landmarks na matriz covariancia
    matriz_covL=np.full((2*novo_taman+3-4,2*novo_taman+3-4),9999) #4 são os iniciais ridiculos
    matriz_covL[0:cov.shape[0], 0:cov.shape[0]]=cov

    

    landmark_positions=np.concatenate([landmark_positions,new_landmarks],axis=0)

    #print(landmark_positions)
    print("------------------------------------")
    #print(matriz_covL)
    print("------------------------------------")
    #matriz_covL.resize(())
    return array_nova.reshape(int(novo_taman*2),1)

def update_step(old_state,new_measure):
    global yaw
    global state_vector,matriz_covL

    cov=matriz_covL
    
    #old state(linhas,4)
    N=int(old_state.shape[0])
    #print(old_state.shape)
    #print(new_measure.shape)
    Qt=np.diag((0.1,0.1))

    belief_pose=np.array([[positionx],[positiony]])
    old_state=old_state.reshape(old_state.shape[0]*4,1)
    
    delta=np.array([[0],[0]])
    for i in range(0,new_measure.shape[0]-1,2):
        if(new_measure[i]!=0):
            
            j=int(i/4)+1

            delta=old_state[i:i+2]-belief_pose
            q=delta.transpose().dot(delta)
            zt=np.zeros((3,1))
            zt[0]=sqrt(q)
            zt[1]=math.atan2(delta[1],delta[0])-yaw


            #print(N)
            #print(i)
            #print(j)
            #calculate measurment jacobian
            F_1=np.block([[eye(3)], [np.zeros((2,3))]])
            #print(F_1.shape)
            #print(F_1)
            F_2=np.block([ [np.zeros((3,2)) ], [eye(2)] ])
            #print(F_2.shape)
            F_xj=np.block([F_1, np.zeros((5,2*(j)-2)), F_2, np.zeros((5,2*N-2*(j)))])
            #print(F_xj)

            r=sqrt(q)

            d0=delta[0]/r
            d1=delta[1]/r
            h=np.block([ [-d0, -d1, 0, d0, d1] , [d1,-d0,-1,-d1,d0] ])
            H=h.dot(F_xj)

            K=cov.dot(H.transpose())

            #i=i+4
            a=H.dot(cov).dot(K) + Qt 
            K=K.dot(a**(-1) )

            print("aaa")
            print(state_vector.shape)
            print(old_state.shape)
            print(K.shape)
            print("urr")

            state_vector=state_vector+K.dot(old_state[i:i+2]-new_measure[i:i+2])
            matriz_covL=(eye(2*N+3) - (K.dot(H)))
            print(matriz_covL)
            matriz_covL=matriz_covL**2
            #print(matriz_covL)

            
            

    return 0
def store_prediction_callback(msg_toreceive:PoseWithCovariance):
    global positionx, positiony, yaw,matriz_covL
    global state_vector

    positionx=msg_toreceive.pose.position.x
    positiony=msg_toreceive.pose.position.y

    orientation_q = msg_toreceive.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    matriz_covL[0:9]= msg_toreceive.covariance[0:9]

    state_vector[0]=positionx
    state_vector[1]=positiony
    state_vector[2]=yaw


def update_calculation_callback(msg_toreceive:Float32MultiArray):
    global landmark_positions

    landmark_recebidos=np.array(msg_toreceive.data)
    #landmark_recebidos=landmark_positions.reshape(int(landmark_positions.shape[0]*0.25),4)    
     
    measure=recognition_step(landmark_positions,landmark_recebidos)


    update_step(landmark_positions,measure)
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
    global positionx, positiony, yaw,matriz_covL,landmark_positions
    global state_vector

    matriz_covL=np.ones((3,3))
    iniciado=0
    positionx=0
    positiony=0
    theta=0
    yaw=0
    landmark_positions=np.array([[0]*4 ])
    
    state_vector=np.array([[0] , [0], [0]])

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    pub=rospy.Publisher("/p3dx/belief_calculation", PoseWithCovariance,queue_size=10)

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)
        
    sub_prediction=rospy.Subscriber("/p3dx/prediction_calculation", PoseWithCovariance, callback =store_prediction_callback)#callback means it call a function when it receives information

    rospy.sleep(0.4)
    sub_update=rospy.Subscriber("/landmarks",Float32MultiArray,callback=update_calculation_callback)
    rospy.spin()#keeps the node alive