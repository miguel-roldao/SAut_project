from ctypes.wintypes import PSIZEL
import math
import time
#from cv2 import Mahalanobis, sqrt
import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy
from sympy import eye
#from torch import block_diag
import rospy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
#from shapely.geometry import LineString
from numpy.linalg import inv

from landmark_extractor import (robot_to_world_frame, 
								closest_line_seg_line_seg,
								pixel_to_coordinates)



def import_match_data(str_pred, str_laser, threshold):

	f_pred = open(str_pred, "r")
	f_laser= open(str_laser, "r")

	pred_t = float(f_pred.readline().split(": ")[1])
	laser_t= float(f_laser.readline().split(":")[1])

	time_s=" "

	while pred_t < threshold and laser_t < threshold:
		while time_s != "time":
			t_l = f_pred.readline().split(": ")
			time_s = t_l[0]

		pred_t = float(t_l[1])
		time_s = " "

		while time_s != "time":
			t_l = f_laser.readline().split(":")
			time_s = t_l[0]

		laser_t = float(t_l[1])
		time_s = " "

	time_comp = pred_t - laser_t
	threshold = np.max([pred_t, laser_t])

	while abs(time_comp) > 0.05:
		
		if time_comp < 0:
			while time_s != "time":
				t_l = f_pred.readline().split(": ")
				time_s = t_l[0]

			pred_t = float(t_l[1])


		else:
			while time_s != "time":
				t_l = f_laser.readline().split(":")
				time_s = t_l[0]

			laser_t = float(t_l[1])

		time_comp = pred_t - laser_t
		time_s = " "

	#print(pred_t,laser_t)

	pose = []
	pose_s = f_pred.readline().split("\n")[0].split("\t")
	x, y, th = float(pose_s[0]), float(pose_s[1]), float(pose_s[2])
	pose.append(x)
	pose.append(y)
	pose.append(th)
	
	cov = []
	cov_s = f_pred.readline().split("\n")[0].split("\t")
	cov = [float(x) for x in cov_s[:-1]]
	#print(cov)

	X = []
	Y = []
	
	while True:

		line = f_laser.readline().split("\n")[0].split("\t")
		
		if line[0].split(":")[0] == "time":
			break

		laser_x = line[0]
		laser_y = line[1]

		X.append(float(laser_x))
		Y.append(float(laser_y))

	return pose, cov, X, Y, threshold


def laser_plot_landmarks(X,Y, P=False):
	#Plot if P is True


	#Parameters
    n_grid = 600            # resolution of the image    ----> n_grid x n_grid
    laser_range = 6         # range of the laser that is plotted 
    

    scale_f=int(n_grid/(2*laser_range))  # helping constant

    img = np.uint8([[[255]*3]*n_grid]*n_grid)   #uint8 is the right format for cv2 apparently

    for i, j in zip(X,Y):
        if np.abs(i) < laser_range and np.abs(j) < laser_range:
            for k in range(-2,2):
                for l in range(-2,2):
                    img[n_grid//2 - int(j*scale_f) + k][int(i*scale_f) + n_grid//2 + l] = [255,0,0]



    if P:
    	cv2.imwrite("./imagens_mapa/map_inst.png", img)   #uncomment to see the result

    #data = np.zeros((512, 512, 3), dtype=np.uint8)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
        
    linesP = cv2.HoughLinesP(edges,
                1,1*np.pi/180,25,minLineLength= 20,maxLineGap = 48) # Probabilistic Hough Transform
        

    #linesP = cv2.HoughLines(edges,1,np.pi/180,20,None,0,0, 0,np.pi/180*5) # Probabilistic Hough Transform

    #print(linesP)

    if linesP is None:
        rospy.loginfo("No landmarks identified!")
        return -1

    linesP=linesP.reshape((len(linesP),4))
    points=linesP.astype(np.float)

        
    remove=[]
        
    for i in range(0, points.shape[0]):
        adicionar=True
        y2=points[i][3]
        y1=points[i][1]
        x2=points[i][2]
        x1=points[i][0]
        if(x1==x2):
            x1=x2+0.1

        angles1=math.atan2(y2-y1,x2-x1)    
        distance6=(x2-x1)**2+(y2-y1)**2
            
        for j in range(i+1, points.shape[0]):
            y2=points[j][3]
            y1=points[j][1]
            x2=points[j][2]
            x1=points[j][0]

            if(x1==x2):
                x1=x2+0.1
    
            distance12=(x2-x1)**2+(y2-y1)**2

            #replace min distance function
            d=closest_line_seg_line_seg([points[i][0], points[i][1]],[points[i][2], points[i][3]],[x1,y1],[x2,y2])
            #print(":::::::::::::::::::")
            #print([points[i][0], points[i][1]],[points[i][2], points[i][3]],[x1,y1],[x2,y2])
            #print(d)
            angles2=math.atan2(y2-y1,x2-x1)
            diff = abs(angles1-angles2)*180/math.pi
               
            #print("-----------")
            #print(angles1)
            #print(angles2)
            #print(diff)
            #print("-----------")
            #print(distance[i][j])
            #print(distance[i][j])
            #parede_par=((points[i][0]-points[j][0])**2 + (points[i][0]-points[j][0])**2)
            
            if(math.sqrt(d)<20 and (diff<30 or abs(diff-180)<30)):#and (diff<30)
                if(distance12>distance6):
                    remove.append(i)
                if(distance6>=distance12):
                    remove.append(j)
                #print(len(remove))

        
    data=np.delete(points,remove,axis=0)   

    linesP=data.astype(np.int32)
        
    if P:    
        if linesP is not None:
    	    for i in range(len(linesP)):
                l = linesP[i]
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                print("line "+ str(i) + " : x0=("+str(l[0])+","+str(l[1])+"), x1=("+str(l[2])+","+str(l[3])+")")

        cv2.imwrite("./houghlines/houghlines_inst.png",img)        

    #landmark_mx = Float32MultiArray()
    #landmark_mx.data=linesP.reshape(len(linesP)*4,1)
    xdata=linesP.reshape(1,len(data)*4)[0]
    landmark_coord = []
    #print(xdata)
        
    for it in range(len(xdata)//2):
        (xx,yy) = pixel_to_coordinates(xdata[2*it+1], xdata[2*it], len(img), len(img), -laser_range, laser_range)
        landmark_coord.append(xx)
        landmark_coord.append(yy)
        #plt.scatter(xx,yy)
        # Convert to coordinates

    #print(landmark_coord)
    #plt.scatter(xx,yy)
    #plt.axis('scaled')
    #plt.savefig("./houghlines/land_test.png")
    #plt.cla()

    #landmark_mx.data = landmark_coord

    return np.array(landmark_coord)

def new_change_NLandMark(n: int, state_vector, matriz_covL):

    #global matriz_covL, state_vector
    #state_vector=np.vstack([state_vector,[[0],[0]]*n ])
    N = matriz_covL.shape[0]+2*n
    #print("---------------")
    #print(N)

    state_vector = state_vector.copy() 
    # print(matriz_covL)
    #print(state_vector)
    state_vector.resize(((N, 1)), refcheck=False)
    #print(state_vector)

    if(n > 0): #parametros(o valor que se inicializa a covariancia duma landmark)
        hold = matriz_covL
        matriz_covL=np.zeros((N,N))
        np.fill_diagonal(matriz_covL,0.2)
        #matriz_covL = np.full((N, N), 0.8)
        matriz_covL[0:N-2*n, 0:N-2*n] = hold
    if(n < 0):
        matriz_covL = matriz_covL[0:N, 0:N]
    #matriz_covL=np.block([ [matriz_covL], [np.full( (2*n,2*n), 999 )] ])
    # print(matriz_covL)

    return state_vector, matriz_covL

def sq_mat_build(mat):
	L = len(mat)
	#print(L)
	N = np.sqrt(L)
	if N.is_integer():
		N = int(N)
	
	else:
		raise ValueError("Matrix can't be turn into square!")

	#print(N)

	new_mat = mat.reshape(N,N)

	return new_mat 


def change_dimension(n:int, matrix:np.array):
    N=matrix.shape[0]+n
    
    if(n>0):
        hold = matrix
        matrix=np.zeros((N,N))
        #np.fill_diagonal(matriz_covL,99)
        #matriz_covL = np.full((N, N), 999)
        matrix[0:N-n, 0:N-n] = hold
    return matrix

def new_update(pose, cov, landmark_coord, state_vector, matriz_covL):
	

	
	#print(state_vector)

	"""matriz_covL = np.array(cov)
	matriz_covL = sq_mat_build(matriz_covL)
	"""
	#test = np.full((3, 3),0.)
	#print(test)
	state_vector, matriz_covL = new_Mahalanobis_recognition(state_vector,matriz_covL, landmark_coord)
	
	return state_vector, matriz_covL

def new_Mahalanobis_recognition(state_vector, matriz_covL, measurementBar):

    N = (state_vector.size-3)//2
    #N = int(N)
    mahalanobis_D = []
    Qt = np.diag((0.1, 2))

    try:
    	len(measurementBar)
    except:
    	#measurementBar=np.array([])
    	return state_vector, matriz_covL

    measurementBar = measurementBar.reshape((measurementBar.size, 1))
    #print(measurementBar)
    #state_vector=np.vstack([ state_vector   ,     [[0],[0]]  ])
    state_vector, matriz_covL = new_change_NLandMark(1,state_vector, matriz_covL)

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
    x_plot=[]
    y_plot=[]

    for i in range(0, measurementBar.size, 2):

        """print(measurementBar[i], state_vector[0])
        x_i,y_i = robot_to_world_frame(measurementBar[i],measurementBar[i+1],state_vector[0],state_vector[1],state_vector[2])  #state_vector[0]+measurementBar[i]     # C H A N G E D 
        test = state_vector[1]+measurementBar[i+1]

        #print("TEST")
        #print(y_i)
        #print(test)
        #print("-----")


        x_plot.append(x_i[0])
        y_plot.append(y_i[0])
		"""

        state_vector[2 + 2*(N)+1], state_vector[2 + 2*(N)+2] = robot_to_world_frame(measurementBar[i],
                                                                                    measurementBar[i+1],
                                                                                    state_vector[0],
                                                                                    state_vector[1],
                                                                                    state_vector[2])  #state_vector[0]+measurementBar[i]     # C H A N G E D 
        #print("spot1",state_vector)

        #state_vector[2 + 2*(N)+2] = state_vector[1]+measurementBar[i+1]   # C H A N G E D


        #state_vector[2 + 2*(N)+1] = [robot_to_world_frame(measurementBar[i][0],measurementBar[i+1][0],state_vector[0][0],state_vector[1][0],state_vector[2][0])[0]]  #state_vector[0]+measurementBar[i] * np.cos(yaw)      # C H A N G E D 
        #state_vector[2 + 2*(N)+2] = [robot_to_world_frame(measurementBar[i][0],measurementBar[i+1][0],state_vector[0][0],state_vector[1][0],state_vector[2][0])[1]]  #state_vector[1]+measurementBar[i+1] * np.sin(yaw)    # C H A N G E D

        z_exp = np.zeros((2, 1))
        deltax = measurementBar[i] 
        deltay = measurementBar[i+1] 
        #deltax = state_vector[2+2*(N)+1] - state_vector[0]  #measurementBar[i]   # C H A N G E D
        #deltay = state_vector[2+2*(N)+2] - state_vector[1]  #measurementBar[i+1] # C H A N G E D

        #deltax = robot_to_world_frame(measurementBar[i][0],measurementBar[i+1][0],state_vector[0][0],state_vector[1][0],state_vector[2][0])[0]
        #deltay = robot_to_world_frame(measurementBar[i][0],measurementBar[i+1][0],state_vector[0][0],state_vector[1][0],state_vector[2][0])[1]
        z_exp[0] = np.sqrt(deltax**2+deltay**2)
        z_exp[1] = math.atan2(deltay, deltax)
        zm_i[i//2]=z_exp
        

        for j in range(0, N+1):

            deltax = state_vector[2 + 2*(j)+1]-state_vector[0]
            deltay = state_vector[2 + 2*(j)+2]-state_vector[1]

            #print("X= " ,deltax)
            q = float(deltax**2+deltay**2)
            r = np.sqrt(q)

            zt = np.zeros((2, 1))
            zt[0] = r
            zt[1] = math.atan2(deltay, deltax) - state_vector[2]   #verificar

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
            h = np.block([[d0, -d1, 0, -d0, d1], [d1, d0, -1, -d1, -d0]])
            #h=abs(h)
            H = h.dot(F_xj)/q
            H_i[j]=H

            Psi = np.matmul(matriz_covL,H.transpose())
            Psi = np.matmul(H,Psi) + Qt
            Psi=np.array(Psi,dtype=np.float)
            Psi=inv(Psi) #fica logo o inverso
            Psi_k[j]=Psi
                       
            deltaz = z_exp-zt
           
            pi=deltaz.transpose().dot((Psi).dot(deltaz))
            #d=deltaz.transpose().dot(deltaz)
            mahalanobis_D.append(pi)
                

        #print("before,mahala",mahalanobis_D)
        mahalanobis_D[N] = 0.3 #parametro
        #print("after,mahala: ",mahalanobis_D)

        j = np.min(mahalanobis_D)
        j = mahalanobis_D.index(j)

        N_ant = N
        N = int(max([N, j+1]))

        
        

        
        H=np.array(H_i[j])
        Psi=np.array(Psi_k[j])

        dn=N-N_ant
        state_vector, matriz_covL = new_change_NLandMark(dn - 1, state_vector, matriz_covL)

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

        state_vector, matriz_covL = new_change_NLandMark(1, state_vector, matriz_covL)
        #print(time.time()-t)
    
    #print("spot2",state_vector)
    state_vector, matriz_covL = new_change_NLandMark(-1, state_vector, matriz_covL)
    #plt.scatter(x_plot,y_plot)
    #plt.savefig("./map_in.png")
    #plt.cla()

    ##Ultimas 2 linhas do pseudo codigo

    #print("t1=" , time.time()-t)
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


        #delta[1] = np.arctan(np.tan(zm[1]-zt[1]))


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
    #print("STATEE" ,state_vector.shape)
    #print("MMMAtttri", matriz_covL)
    matriz_covL=np.array(matriz_covL, dtype=float)

    
    #print("Det", np.linalg.det(matriz_covL))

    #print("t2=" , time.time()-t)

    return state_vector, matriz_covL	


if __name__ == '__main__':

	threshold = 0
	it = 0
	state_vector = None

	while threshold < 1656066109.0:

		pose, cov, X, Y, threshold = import_match_data("./static/prediction_data.txt", "./static/laser_data.txt",threshold)

		#print(cov)

		print("t= ",threshold)


		if it == 0:
			state_vector = np.array(pose).reshape(3,1)
			matriz_covL = np.full((3, 3),0.)



		landmark_coord = laser_plot_landmarks(X,Y,P=True)
		

		#matriz_covL[0:3,0:3] = sq_mat_build(np.array(cov[:9]))


		state_vector, matriz_covL = new_update(pose,cov,landmark_coord, state_vector, matriz_covL)

		
		if it % 1 == 0:

			#print(state_vector)


			
			N = (state_vector.size-3)//2
			x_plot = [state_vector[3+2*i] for i in range(N)]
			y_plot = [state_vector[4+2*i] for i in range(N)]

			plt.scatter(x_plot,y_plot)
			
			x_val=float(state_vector[0][0])
			y_val=float(state_vector[1][0])
			yaw_val=float(state_vector[2][0])

			arrowx=[x_val]
			arrowx.append(x_val + 0.1 * np.cos(yaw_val))
			arrowy=[y_val]
			arrowy.append(y_val + 0.1 * np.sin(yaw_val))


			plt.plot(arrowx,arrowy, c="r", linewidth=3)
			plt.scatter(x_val,y_val,c="r")

			plt.show()
			#plt.savefig("./map_intest.png")
			plt.cla()
			
			#s = "Oh sailor we have no fish!"

			#m = s.split(", ")
			#print(m)

		it +=1
		threshold += 0.1