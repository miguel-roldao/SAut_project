import math
from turtle import distance
from sympy import rem
#from torch import atan2
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
#from ransac_fit import fitar
import time
import cv2
from io import BytesIO
from PIL import Image

import matplotlib.pyplot as plt


def distance(x1,y1,x2,y2):
    return((x1-x2)**2+(y2-y1)**2)

def closest_line_seg_line_seg(p1, p2, p3, p4):
    p1=np.array(p1)
    p2=np.array(p2)
    p3=np.array(p3)
    p4=np.array(p4)
    P1 = p1
    P2 = p3
    V1 = p2 - p1
    V2 = p4 - p3
    V21 = P2 - P1

    v22 = np.dot(V2, V2)
    v11 = np.dot(V1, V1)
    v21 = np.dot(V2, V1)
    v21_1 = np.dot(V21, V1)
    v21_2 = np.dot(V21, V2)
    denom = v21 * v21 - v22 * v11


    if np.isclose(denom, 0.):
        s = 0.
        t = (v11 * s - v21_1) / v21
    else:
        s = (v21_2 * v21 - v22 * v21_1) / denom
        t = (-v21_1 * v21 + v11 * v21_2) / denom

    s = max(min(s, 1.), 0.)
    t = max(min(t, 1.), 0.)

    p_a = P1 + s * V1
    p_b = P2 + t * V2

    d=(p_a[0]-p_b[0])**2 + (p_a[1]-p_b[1])**2 

    d=min(d,np.linalg.norm(p1-p3)**2,np.linalg.norm(p1-p4)**2,np.linalg.norm(p2-p3)**2, np.linalg.norm(p2-p4)**2)
    return d
                 

def pixel_to_coordinates(px:int, py:int, xpix:int, ypix:int, xmin:float, xmax:float, ymin = 0, ymax = 0) -> (float,float):
    """
    px: position in pixels of the x
    py: position in pixels of the y
    xpix: number of pixels in x direction
    ypix: number of pixels in y direction
    xmin, xmax, ymin and ymax: range of the plot (real coordinates)

    return:
    x real coordinate
    y real coordinate

    ------------>                       
    |           py                      Λ
    |                                   | y
    |                   ---->           |
    |                                   |
    |px                                 |
    v                                   |         x
                                        ----------->
    """

    if ymin == ymax:
        ymin=xmin
        ymax=xmax

    #print(px)
    #print(py)
    xdelta = xmax - xmin
    ydelta = ymax - ymin
    mx = xdelta/ypix    
    my = ydelta/xpix
    x = py * mx + xmin
    y = -px * my - ymin

    return x,y



def laser_to_robot_frame(x:float,y:float, dx=0.05, dy=0.0) -> (float,float):
    #Changes the coordinates of the landmarks to be in the reference frame of the robot

    """
    dx, dy: distances between laser and center of the robot wheel axis
    """
    dx = 0.05
    dy = 0.0

    return x + dx,y + dy


def robot_to_world_frame(x, y, xrobot:float, yrobot:float, theta_robot:float) -> (float, float):

    """
    x, y coordinates of a point measured by the robot while with pose 
    (xrobot, yrobot, theta_robot)
    
    """
    #calculate the rotated coordinates of the point as if the robot had null orientation
    
    array_bool=False

    if type(x) is np.ndarray:
        x=x[0]
        y=y[0]
        xrobot=xrobot[0]
        yrobot=yrobot[0]
        theta_robot=theta_robot[0]

        array_bool = True

    xrot = x * np.cos(theta_robot) - y * np.sin(theta_robot)
    yrot = x * np.sin(theta_robot) + y * np.cos(theta_robot)

    #then we just have to perform a translation

    x = xrobot + xrot
    y = yrobot + yrot
    
    if array_bool:
        x=np.array([x])
        y=np.array([y])

    return x,y

def Simple_laser_callback(msg_toreceive:LaserScan):
    global t0, it

    t2=time.time()

    if it%10 ==0 :

        samples=len(msg_toreceive.ranges)

        R=np.array(msg_toreceive.ranges)
        #print(R)
        salto=1
        for i in range(0,len(R),salto):
            R[i]=np.sum(R[i:i+salto])/(salto)

        #retrieve data from the laser
        theta_array=[]
        theta_min=msg_toreceive.angle_min
        theta_max=msg_toreceive.angle_max
        incremento=msg_toreceive.angle_increment
        theta_array=[i for i in  np.arange (theta_min,theta_max+incremento,incremento)]

        #Convert data to X-Y
        X=R*np.cos(theta_array)
        Y=R*np.sin(theta_array)


        #salto=3
        #for i in range(0,X.shape[0],salto):
            #Y[i]=np.sum(Y[i:i+salto])/salto
            #X[i]=np.sum(X[i:i+salto])/(salto)
        """
        if(time.time()-t0>0.5):
            ni=ni+1
            t0=time.time()
            np.savetxt("./coordinates/X" + str(ni) + ".txt", X)
            np.savetxt("./coordinates/Y" + str(ni) + ".txt", Y)
        """
            

        #Convert to robot frame

        X, Y = laser_to_robot_frame(X,Y)


        with open("./lab_SA/laser_data.txt","a") as f1:
            f1.write("time:"+str(t2)+"\n")

            for i in range(len(X)):
                f1.write(str(X[i])+"\t"+str(Y[i])+"\n")

            f1.close()

    it+=1



def Laser_callback(msg_toreceive:LaserScan):

    global t0
    global ni

    if time.time()-t0 > 0.4:


        samples=len(msg_toreceive.ranges)

        R=np.array(msg_toreceive.ranges)
        #print(R)
        salto=1
        for i in range(0,len(R),salto):
            R[i]=np.sum(R[i:i+salto])/(salto)

        #retrieve data from the laser
        theta_array=[]
        theta_min=msg_toreceive.angle_min
        theta_max=msg_toreceive.angle_max
        incremento=msg_toreceive.angle_increment
        theta_array=[i for i in  np.arange (theta_min,theta_max+incremento,incremento)]

        #Convert data to X-Y
        X=R*np.cos(theta_array)
        Y=R*np.sin(theta_array)


        #salto=3
        #for i in range(0,X.shape[0],salto):
            #Y[i]=np.sum(Y[i:i+salto])/salto
            #X[i]=np.sum(X[i:i+salto])/(salto)
        """
        if(time.time()-t0>0.5):
            ni=ni+1
            t0=time.time()
            np.savetxt("./coordinates/X" + str(ni) + ".txt", X)
            np.savetxt("./coordinates/Y" + str(ni) + ".txt", Y)
        """
        

        #Convert to robot frame

        X, Y = laser_to_robot_frame(X,Y)


        #---------------------------------------------------

        #Plot

        n_grid = 400            # resolution of the image    ----> n_grid x n_grid
        laser_range = 4         # range of the laser that is plotted 
        scale_f=int(n_grid/(2*laser_range))  # helping constant

        img = np.uint8([[[255]*3]*n_grid]*n_grid)   #uint8 is the right format for cv2 apparently

        for i, j in zip(X,Y):
            if np.abs(i) < laser_range and np.abs(j) < laser_range:
                for k in range(-2,2):
                    for l in range(-2,2):
                        img[n_grid//2 - int(j*scale_f) + k][int(i*scale_f) + n_grid//2 + l] = [255,0,0]



        cv2.imwrite("./imagens_mapa/map_inst.png", img)   #uncomment to see the result


        #data = np.zeros((512, 512, 3), dtype=np.uint8)

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        
        linesP = cv2.HoughLinesP(edges,
                    1,1*np.pi/180,25,minLineLength= 20,maxLineGap = 28) # Probabilistic Hough Transform
        

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
        
        
        if linesP is not None:
            for i in range(len(linesP)):
                l = linesP[i]
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
                print("line "+ str(i) + " : x0=("+str(l[0])+","+str(l[1])+"), x1=("+str(l[2])+","+str(l[3])+")")

        cv2.imwrite("./houghlines/houghlines_inst.png",img)        

        landmark_mx = Float32MultiArray()
        landmark_mx.data=linesP.reshape(len(linesP)*4,1)
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

        landmark_mx.data = landmark_coord

        pub.publish(landmark_mx)
        #"""
        #rospy.loginfo(time.time()-t0)
        t0=time.time()
        
    return 1

if __name__== '__main__':
    rospy.init_node("Display_node")
    rospy.loginfo("Hello from display")

    global t0,it
    global ni
    ni=0
    t0=time.time()
    it=0

    f1 = open("./lab_SA/laser_data.txt","w") #clear file
    f1.close()

    sub=rospy.Subscriber("/scan",LaserScan,callback=Simple_laser_callback)

    #------------------

    #sub=rospy.Subscriber("/p3dx/laser/scan",LaserScan,callback=Simple_laser_callback)

    pub=rospy.Publisher("/landmarks",Float32MultiArray,queue_size=10)

    #sub=rospy.Subscriber("/scan",LaserScan,callback=ransac_callback)
    rospy.spin()