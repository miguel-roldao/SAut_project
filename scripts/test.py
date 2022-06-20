import numpy as np
from convert import pixel_to_coordinates

"""
y = np.array([[1,0],[2,2],[3,4]])

z=y.reshape(len(y)*2,1)

print(z)


im = [[255]*3]*4
print(im)


x,y = pixel_to_coordinates(90,90,100,100,-5,5)

print(x,y)

"""

def robot_to_world_frame(x:float, y:float, xrobot:float, yrobot:float, theta_robot:float) -> (float, float):

    """
    x, y coordinates of a point measured by the robot while with pose 
    (xrobot, yrobot, theta_robot)
    
    """
    #calculate the rotated coordinates of the point as if the robot had null orientation
    
    xrot = x*np.cos(theta_robot) - y*np.sin(theta_robot)
    yrot = x*np.sin(theta_robot) + y*np.cos(theta_robot)

    #then we just have to perform a translation

    x = xrobot + xrot
    y = yrobot + yrot
    
    return x,y

if __name__== '__main__':


	print(robot_to_world_frame(1,1,2,1,np.pi/4))
	print(1+np.random.normal(0.,1.))