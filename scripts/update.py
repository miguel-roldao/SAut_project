from ctypes.wintypes import PSIZEL
import math
import time
from cv2 import Mahalanobis, sqrt
from matplotlib.pyplot import axis
import numpy as np
import scipy
from sympy import eye
from torch import block_diag
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from shapely.geometry import LineString
from numpy.linalg import inv

def change_dimension(n:int, matrix:np.array):
    N=matrix.shape[0]+n
    
    if(n>0):
        hold = matrix
        matrix=np.zeros((N,N))
        #np.fill_diagonal(matriz_covL,99)
        #matriz_covL = np.full((N, N), 999)
        matrix[0:N-n, 0:N-n] = hold
    return matrix
def change_NLandMark(n: int):

    global matriz_covL, state_vector
    #state_vector=np.vstack([state_vector,[[0],[0]]*n ])
    N = matriz_covL.shape[0]+2*n

    # print(matriz_covL)
    #print(state_vector)
    state_vector.resize(((N, 1)))
    #print(state_vector)

    if(n > 0):
        hold = matriz_covL
        matriz_covL=np.zeros((N,N))
        np.fill_diagonal(matriz_covL,0.2)
        #matriz_covL = np.full((N, N), 0.8)
        matriz_covL[0:N-2*n, 0:N-2*n] = hold
    if(n < 0):
        matriz_covL = matriz_covL[0:N, 0:N]
    #matriz_covL=np.block([ [matriz_covL], [np.full( (2*n,2*n), 999 )] ])
    # print(matriz_covL)


def Mahalanobis_recognition(measurementBar: np.array):
    global matriz_covL, state_vector

    t=time.time()
    
    N = (state_vector.size-3)/2
    N = int(N)
    mahalanobis_D = []
    Qt = np.diag((1, 0.2))

    measurementBar = measurementBar.reshape((measurementBar.size, 1))
    #state_vector=np.vstack([ state_vector   ,     [[0],[0]]  ])
    change_NLandMark(1)

    H_i=[0]*50 #arrays no máximo
    H_ji=[0]*50
    zt_k=[0]*50
    Psi_k=[0]*50
    
    zt_i=[0]*measurementBar.size
    K_i=[0]*measurementBar.size
    zm_i=[0]*measurementBar.size
    # print(state_vector)
   # print(state_vector.shape)
    #print("Measure=",measurementBar)
    #print("state", state_vector)

    for i in range(0, measurementBar.size, 2):
        # print(i)
        state_vector[2 + 2*(N)+1] = state_vector[0]+measurementBar[i]
        state_vector[2 + 2*(N)+2] = state_vector[1]+measurementBar[i+1]

        z_exp = np.zeros((2, 1))
        deltax = measurementBar[i]
        deltay = measurementBar[i+1]
        z_exp[0] = np.sqrt(deltax**2+deltay**2)
        z_exp[1] = math.atan2(deltay, deltax)-state_vector[1]
        zm_i[int(i/2)]=z_exp
        

        for j in range(0, N+1):

            deltax = state_vector[2 + 2*(j)+1]-state_vector[0]
            deltay = state_vector[2 + 2*(j)+2]-state_vector[1]

            #print("X= " ,deltax)
            q = float(deltax**2+deltay**2)
            r = np.sqrt(q)

            zt = np.zeros((2, 1))
            zt[0] = r
            zt[1] = math.atan2(deltay, deltax)-state_vector[2]

            zt_k[j]=zt

            F_1 = np.block([[eye(3)], [np.zeros((2, 3))]])
            # print(F_1.shape)
            # print(F_1)
            F_2 = np.block([[np.zeros((3, 2))], [eye(2)]])
            # print(F_2.shape)
            F_xj = np.block([F_1, np.zeros((5, 2*(j+1)-2)),
                            F_2, np.zeros((5, 2*(N+1)-2*(j+1)))])

            d0 = deltax*r
            d1 = deltay*r
            h = np.block([[-d0, -d1, 0, d0, d1], [d1, -d0, -1, -d1, d0]])
            #h=abs(h)
            H = h.dot(F_xj)/q
            H_i[j]=H

            Psi = np.matmul(matriz_covL,H.transpose())
            Psi = np.matmul(H,Psi) + Qt
            Psi=np.array(Psi,dtype=np.float)
            Psi=inv(Psi) #fica logo o inverso
            Psi_k[j]=Psi
                       
            deltaz = z_exp-zt
           
            d=deltaz.transpose().dot((Psi).dot(deltaz))
            d=deltaz.transpose().dot(deltaz)
            mahalanobis_D.append(d)
                

       
        mahalanobis_D[N]=1

        j = np.min(mahalanobis_D)
        j = mahalanobis_D.index(j)

        N_ant=N
        print(N)
        N = int(max([N, j+1]))
        print(N)
        print("--")

        
        H=np.array(H_i[j])
        Psi=np.array(Psi_k[j])

        dn=N-N_ant
        change_NLandMark(dn - 1)

        hold=H
        #print(hold.shape)
        #H=np.zeros((2,hold.shape[1] - 2*(1-dn)))
        H=H[0:2, 0:hold.shape[1] - 2*(1-dn)]
        #H_i=[0]*100
        H_ji[int(i/2)]=H

        
        K=H.transpose().dot(Psi)
        
        K=np.matmul(matriz_covL,K)
        K_i[int(i/2)]=K
        #print(K.shape)
        zt_i[int(i/2)]=zt_k[j]

    

        mahalanobis_D = []

        change_NLandMark(-(N-N_ant - 1))
        change_NLandMark(N-N_ant )
        print(time.time()-t)
    
    change_NLandMark(-1)
    

    ##Ultimas 2 linhas do pseudo codigo

    print("t1=" , time.time()-t)
    R=[0]*measurementBar.size
    R2=[0]*measurementBar.size
    previous=np.zeros((2,1))
    #previous2=np.zeros((2,2))
    previous2=0
    
    for i in range(0,int(measurementBar.size/2)):
        K=np.array(K_i[i])
        zm=np.array(zm_i[i])
        zt=np.array(zt_i[i])
        delta=zm-zt

        R[i]=K.dot(delta)
        H=np.array(H_ji[i])
        R2[i]=K.dot(H)

        if i>0:
            #print("i", i)
            dn=R2[i].shape[0]-R2[i-1].shape[0]
            R2[i-1]=change_dimension(dn,R2[i-1])
            previous2=R2[i]+R2[i-1]

            #print("i", i)
            dn=R[i].shape[0]-R[i-1].shape[0]
            R[i-1]=np.vstack((R[i-1], np.zeros((dn,1))))
            previous=R[i]+R[i-1]


    state_vector=state_vector+previous
    matriz_covL=(np.eye(2*N+3) - previous2).dot(matriz_covL)
    print("STATEE" ,state_vector)
    #print("MMMAtttri", matriz_covL)
    matriz_covL=np.array(matriz_covL, dtype=float)

    
    print("Det", np.linalg.det(matriz_covL))

    print("t2=" , time.time()-t)

def update(measurementBar: np.array):
    global matriz_covL, state_vector

    N = (state_vector.size-3)/2
    N = int(N)
    
    Qt = np.diag((10., 1.))

    measurementBar = measurementBar.reshape((measurementBar.size, 1))

    print("N=",N)
    #print(state_vector.shape)
    #print(matriz_covL.shape)

    
    for i in range(0, measurementBar.size, 2):
        
        for j in range(0, N):

            deltax = state_vector[2 + 2*(j)+1]-state_vector[0]
            deltay = state_vector[2 + 2*(j)+2]-state_vector[1]

            #print("X= " ,deltax)
            q = float(deltax**2+deltay**2)
            r = np.sqrt(q)

            zt = np.zeros((2, 1))
            zt[0] = r
            zt[1] = math.atan2(deltay, deltax)-state_vector[2]

            F_1 = np.block([[eye(3)], [np.zeros((2, 3))]])
            # print(F_1.shape)
            # print(F_1)
            F_2 = np.block([[np.zeros((3, 2))], [eye(2)]])
            # print(F_2.shape)
            F_xj = np.block([F_1, np.zeros((5, 2*(j+1)-2)),
                            F_2, np.zeros((5, 2*(N)-2*(j+1)))])

            F_1=eye(3)
            F_2=np.zeros((3,2*N))

            
            F_3=np.zeros((2,3))
            F_4=np.zeros((2,2*(j+1)-2 ))
            F_5=eye(2)
            F_6=np.zeros((2,2*N-2*(j+1)))

            F_7=np.concatenate((F_3,F_4,F_5,F_6),axis=1)
            F_8=np.concatenate((F_1,F_2),axis=1)

            #print(F_8)
            #print(F_7)

            #rospy.sleep(2)
            F_xj=np.concatenate((F_8,F_7),axis=0)
            #F_xj=np.block( [ [F_1,F_2] ,  [F_3,F_4,F_5,F_6] ])
            #print(F_xj)
            #rospy.sleep(2)
            #print("F:", F_xj.shape)
            #print(F_xj)
            d0 = deltax/r
            d1 = deltay/r
            h = np.block([[-d0, -d1, 0, d0, d1], [d1, -d0, -1, -d1, d0]])
            #h=abs(h)
            H = h.dot(F_xj)

            K=0
            K=np.matmul(matriz_covL,H.transpose())
            K=np.matmul(H,K)+Qt
            K=matriz_covL.dot(H.transpose()).dot(K**(-1))

            
            z_exp = np.zeros((2, 1))
            deltax = measurementBar[i]
            deltay = measurementBar[i+1]
            z_exp[0] = np.sqrt(deltax**2+deltay**2)
            z_exp[1] = math.atan2(deltay, deltax)-state_vector[1]

            deltaz = z_exp-zt
            ##print("lllll")
            #print(j)
            #print(deltaz)
            #print("jjjjj")

            #print(K.shape)
            #print(state_vector.shape)
            #print(state_vector)
            #####################################################################
            print("J=" , j)
            print("Antes", state_vector)

            state_vector=state_vector+K.dot(deltaz)

            print("Depois",state_vector)

             

            #print(matriz_covL.shape)
            matriz_covL=np.eye(2*N+3) - K.dot(H).dot(matriz_covL)
            #print(state_vector)
            #print(matriz_covL.shape)
            #a=scipy.linalg.det(matriz_covL)
            #print("Determinante = ", a)

            


def recognition_step(array_antiga: np.array, array_recebida: np.array):

    global landmark_positions, matriz_covL

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

    Npoints_antiga = int(array_antiga.size/2)
    Npoints_recebida = int(array_recebida.size/2)

    array_antiga = array_antiga.reshape(Npoints_antiga, 2)
    array_recebida = array_recebida.reshape(Npoints_recebida, 2)

    array_recebida[:, 0] = array_recebida[:, 0]+positionx
    array_recebida[:, 1] = array_recebida[:, 1]+positiony

    array_nova = np.zeros((Npoints_antiga, 2))
    # print(array_antiga)
    # print(array_recebida)

    for i in range(0, Npoints_recebida):
        p1 = ((array_recebida[i][0], array_recebida[i][1]))

        for j in range(0, Npoints_antiga):
            p2 = ((array_antiga[j][0], array_antiga[j][1]))

            line = LineString([p1, p2])
            # d=p1.distance(p2)
           # print(line.length)
            # print(p1[0])
            if(line.length < 0.20):
                array_nova[j][0] = p1[0]
                array_nova[j][1] = p1[1]
                # print("u")
                break
            if(j == Npoints_antiga-1):
                array_nova = np.concatenate([array_nova, [[p1[0], p1[1]]]])

    # print(array_nova)
    # print(array_nova)
    # print(array_recebida)
    novo_taman = array_nova.shape[0]

    new_landmarks = array_nova[Npoints_antiga:novo_taman]
    # print(new_landmarks.reshape(int(new_landmarks.size/4),4))

    # print(new_landmarks)
    # print("!!")
    # print(state_vector)
    # print(new_landmarks)

    # new_landmarks=new_landmarks.reshape(int(new_landmarks.size*2),1)
    state_vector = np.concatenate(
        [state_vector, new_landmarks.reshape(int(new_landmarks.size), 1)])

    new_landmarks = new_landmarks.reshape(int(new_landmarks.size/4), 4)
    # print(new_landmarks)
    # print(".---.")
    # print(landmark_positions)

    cov = matriz_covL  # Adicionar as novas landmarks na matriz covariancia
    # 4 são os iniciais ridiculos
    matriz_covL = np.full((2*novo_taman+3-4, 2*novo_taman+3-4), 9999)
    matriz_covL[0:cov.shape[0], 0:cov.shape[0]] = cov

    landmark_positions = np.concatenate(
        [landmark_positions, new_landmarks], axis=0)

    # print(landmark_positions)
    print("------------------------------------")
    # print(matriz_covL)
    print("------------------------------------")
    # matriz_covL.resize(())
    return array_nova.reshape(int(novo_taman*2), 1)


def update_step(old_state, new_measure):
    global yaw
    global state_vector, matriz_covL

    cov = matriz_covL

    # old state(linhas,4)
    N = int(old_state.shape[0])
    # print(old_state.shape)
    # print(new_measure.shape)
    Qt = np.diag((0.1, 0.1))

    belief_pose = np.array([[positionx], [positiony]])
    old_state = old_state.reshape(old_state.shape[0]*4, 1)

    delta = np.array([[0], [0]])
    for i in range(0, new_measure.shape[0]-1, 2):
        if(new_measure[i] != 0):

            j = int(i/4)+1

            delta = old_state[i:i+2]-belief_pose
            q = delta.transpose().dot(delta)
            zt = np.zeros((3, 1))
            zt[0] = sqrt(q)
            zt[1] = math.atan2(delta[1], delta[0])-yaw

            # print(N)
            # print(i)
            # print(j)
            # calculate measurment jacobian
            F_1 = np.block([[eye(3)], [np.zeros((2, 3))]])
            # print(F_1.shape)
            # print(F_1)
            F_2 = np.block([[np.zeros((3, 2))], [eye(2)]])
            # print(F_2.shape)
            F_xj = np.block([F_1, np.zeros((5, 2*(j)-2)),
                            F_2, np.zeros((5, 2*N-2*(j)))])
            # print(F_xj)

            r = sqrt(q)

            d0 = delta[0]/r
            d1 = delta[1]/r
            h = np.block([[-d0, -d1, 0, d0, d1], [d1, -d0, -1, -d1, d0]])
            H = h.dot(F_xj)

            K = cov.dot(H.transpose())

            # i=i+4
            a = H.dot(cov).dot(K) + Qt
            K = K.dot(a**(-1))

            print("aaa")
            print(state_vector.shape)
            print(old_state.shape)
            print(K.shape)
            print("urr")

            state_vector = state_vector + \
                K.dot(old_state[i:i+2]-new_measure[i:i+2])
            matriz_covL = (eye(2*N+3) - (K.dot(H)))
            print(matriz_covL)
            matriz_covL = matriz_covL**2
            # print(matriz_covL)

    return 0


def store_prediction_callback(msg_toreceive: PoseWithCovariance):
    global positionx, positiony, yaw, matriz_covL
    global state_vector

    positionx = msg_toreceive.pose.position.x
    positiony = msg_toreceive.pose.position.y

    orientation_q = msg_toreceive.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
    cov=np.array(msg_toreceive.covariance[0:9]).reshape((3,3))
    matriz_covL[0:3,0:3] = cov

    state_vector[0] = positionx
    state_vector[1] = positiony
    state_vector[2] = yaw


def update_calculation_callback(msg_toreceive: Float32MultiArray):
    global landmark_positions

    landmark_recebidos = np.array(msg_toreceive.data)

    Mahalanobis_recognition(landmark_recebidos)

    msg_tosend=Float32MultiArray()

    n=state_vector.size

    
    msg_tosend.layout.data_offset=n

    print("n", n)
    
    cov=matriz_covL.flatten()
    state=state_vector.flatten()
    msg_tosend.data=np.hstack((state,cov))
    print(np.array(msg_tosend.data).size)
    pub_update.publish(msg_tosend)
    #msg_tosend.layout.dim.si

    #update(landmark_recebidos)

    # landmark_recebidos=landmark_positions.reshape(int(landmark_positions.shape[0]*0.25),4)

    # measure=recognition_step(landmark_positions,landmark_recebidos)

    # update_step(landmark_positions,measure)
    # ---->Transformar tudo em coordenadas cartesianas, as landmarks tem de ter apenas uma coluna

    # landmark_novos=landmark_positions.append(landmark_recebidos)## e substituir com os novos fazendo correspond

    # landmark_positions=landmark_positions.append(landmark_recebidos)##apenas os novos
    # analyse
    #delta = landmark_positions-landmark_novos

    # q=delta.transpose().dot(delta)

    # for i in range(0,landmark_novos.shape[0]):
    # z_previsto=np.array([ [math.sqrt(q[i])] , [math.atan2(delta[i])]]

    # rospy.loginfo(landmark_positions)


if __name__ == '__main__':
    global positionx, positiony, yaw, matriz_covL, landmark_positions
    global state_vector

    matriz_covL = np.full((3, 3),0.)
    iniciado = 0
    positionx = 0
    positiony = 0
    theta = 0
    yaw = 0
    landmark_positions = np.array([[0]*4])

    state_vector = np.array([[0.], [0.], [0.]])

    rospy.init_node("Belief_calculation")
    rospy.loginfo("Hello from belief_calculation")

    pub_update = rospy.Publisher("/p3dx/update",
                          Float32MultiArray, queue_size=10)

    #sub_pose=rospy.Subscriber("/pioneer/pose_belief", PoseWithCovariance)

    # callback means it call a function when it receives information
    sub_prediction = rospy.Subscriber(
        "/p3dx/prediction_calculation", PoseWithCovariance, callback=store_prediction_callback)

    rospy.sleep(0.4)
    sub_land = rospy.Subscriber(
        "/landmarks", Float32MultiArray, callback=update_calculation_callback)
    rospy.spin()  # keeps the node alive
