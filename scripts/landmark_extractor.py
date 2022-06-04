from email.policy import strict
import rospy
import numpy as np
from pylab import *
from matplotlib.markers import MarkerStyle
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
#from ransac_fit import fitar
import time
import cv2

def Laser_callback(msg_toreceive:LaserScan):

    global t0
    global ni

    samples=len(msg_toreceive.ranges)

    #retrieve data from the laser
    theta_array=[]
    theta_min=msg_toreceive.angle_min
    theta_max=msg_toreceive.angle_max
    incremento=msg_toreceive.angle_increment
    theta_array=[i for i in  np.arange (theta_min,theta_max+incremento,incremento)]

    #Convert data to X-Y
    X=msg_toreceive.ranges*np.cos(theta_array)
    Y=msg_toreceive.ranges*np.sin(theta_array)

    """
    if(time.time()-t0>0.5):
        ni=ni+1
        t0=time.time()
        np.savetxt("./coordinates/X" + str(ni) + ".txt", X)
        np.savetxt("./coordinates/Y" + str(ni) + ".txt", Y)
    """
    #Plot
    
    plt.scatter(X,Y,s=1.0)
    plt.axis('scaled')
    plt.axis('off')
    plt.savefig("./imagens_mapa/mapa_inst.png")
    plt.close()


    #fitar(X,Y)
    #Hough transform

    img = cv2.imread("./imagens_mapa/mapa_inst.png")

    #img_s = img.copy()

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    linesP = cv2.HoughLinesP(edges,1,np.pi/180,10,None,10,10) # Probabilistic Hough Transform

    landmark_mx = Float32MultiArray(data=linesP)

    """
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            print("line "+ str(i) + " : x0=("+str(l[0])+","+str(l[1])+"), x1=("+str(l[2])+","+str(l[3])+")")

    #cv2.imwrite("./houghlines/houghlines_inst.png",img)
    """

    rospy.loginfo(time.time()-t0)
    print(samples)
    print(len(theta_array))


    pub.publish(landmark_mx)
    
    return 1

if __name__== '__main__':
    rospy.init_node("Display_node")
    rospy.loginfo("Hello from display")

    global t0
    global ni
    ni=0
    t0=time.time()

    sub=rospy.Subscriber("/p3dx/laser/scan",LaserScan,callback=Laser_callback)

    pub=rospy.Publisher("/landmarks",Float32MultiArray)

    #sub=rospy.Subscriber("/scan",LaserScan,callback=ransac_callback)
    rospy.spin()