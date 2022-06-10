import math
from turtle import distance
from sympy import rem
from torch import atan2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
#from ransac_fit import fitar
import time
import cv2
from io import BytesIO
from PIL import Image

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
                 




def Laser_callback(msg_toreceive:LaserScan):

    global t0
    global ni

    t0=time.time()

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
    #Plot
    
    plt.scatter(X,Y,s=1.0)
    plt.axis('equal')
    plt.axis('off')
    plt.savefig("./imagens_mapa/mapa_inst.png")
    plt.close()
   
    
    img = cv2.imread("./imagens_mapa/mapa_inst.png")
    #data = np.zeros((512, 512, 3), dtype=np.uint8)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    linesP = cv2.HoughLinesP(edges,1,10*np.pi/180,25,minLineLength= 12,maxLineGap = 8) # Probabilistic Hough Transform
    #linesP = cv2.HoughLines(edges,1,np.pi/180,20,None,0,0, 0,np.pi/180*5) # Probabilistic Hough Transform

    #print(linesP)

    linesP=linesP.reshape((len(linesP),4))

    points=linesP.astype(np.float_)

    
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
    
    """
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i]
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
            print("line "+ str(i) + " : x0=("+str(l[0])+","+str(l[1])+"), x1=("+str(l[2])+","+str(l[3])+")")

    cv2.imwrite("./houghlines/houghlines_inst.png",img)
    rospy.sleep(1)
    """

    landmark_mx = Float32MultiArray()
    landmark_mx.data=data.reshape(len(data)*4,1)
    

    pub.publish(landmark_mx)
    #"""
    rospy.loginfo(time.time()-t0)
    print(landmark_mx)
    
    return 1

if __name__== '__main__':
    rospy.init_node("Display_node")
    rospy.loginfo("Hello from display")

    global t0
    global ni
    ni=0
    t0=time.time()

    sub=rospy.Subscriber("/p3dx/laser/scan",LaserScan,callback=Laser_callback)

    pub=rospy.Publisher("/landmarks",Float32MultiArray,queue_size=10)

    #sub=rospy.Subscriber("/scan",LaserScan,callback=ransac_callback)
    rospy.spin()