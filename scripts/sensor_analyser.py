from email.policy import strict
import rospy
import numpy as np
from pylab import *
from sensor_msgs.msg import LaserScan
from ransac_fit import fitar
import time

def ransac_callback(msg_toreceive:LaserScan):

    global t0
    global ni

    samples=len(msg_toreceive.ranges)

    #retrieve data from the laser
    theta_array=[]
    theta_min=msg_toreceive.angle_min
    theta_max=msg_toreceive.angle_max
    incremento=msg_toreceive.angle_increment
    theta_array=[i for i in  np.arange (theta_min,theta_max+incremento,incremento)]

    #Covert data to X-Y
    X=msg_toreceive.ranges*np.cos(theta_array)
    Y=msg_toreceive.ranges*np.sin(theta_array)


    if(time.time()-t0>2):
        ni=ni+1
        t0=time.time()
        np.savetxt("./coordinates/X" + str(ni) + ".txt", X)
        np.savetxt("./coordinates/Y" + str(ni) + ".txt", Y)
    
    #fitar(X,Y)

    #Hough transform

    rospy.loginfo(time.time()-t0)
    print(samples)
    print(len(theta_array))

    return 1

if __name__== '__main__':
    rospy.init_node("Display_node")
    rospy.loginfo("Hello from display")

    global t0
    global ni
    ni=0
    t0=time.time()

    sub=rospy.Subscriber("/p3dx/laser/scan",LaserScan,callback=ransac_callback)

    #sub=rospy.Subscriber("/scan",LaserScan,callback=ransac_callback)
    rospy.spin()