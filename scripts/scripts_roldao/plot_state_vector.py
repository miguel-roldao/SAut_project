import matplotlib.pyplot as plt
import numpy as np

def robot_to_world_frame(x:float, y:float, xrobot:float, yrobot:float, theta_robot:float) -> (float, float):

    """
    x, y coordinates of a point measured by the robot while with pose 
    (xrobot, yrobot, theta_robot)
    
    """
    #calculate the rotated coordinates of the point as if the robot had null orientation
    
    x = x - xrobot
    y = y - yrobot

    xrot = x*np.cos(theta_robot) - y*np.sin(theta_robot)
    yrot = x*np.sin(theta_robot) + y*np.cos(theta_robot)

    #then we just have to perform a translation

    x = xrobot + xrot
    y = yrobot + yrot
    
    return x,y



n=0
state_vector = []

with open("./update.txt","r") as f1:
	
	while True:

		s = [f1.readline() for i in range(2)]
		
		if s == "":
			break

		n = int(f1.readline().split(": ")[1])
		print(n)


		whole_vector = [float(x) for x in f1.readline().split(": ")[1][1:-2].split(", ")]
		#print(whole_vector)
		state_vector = whole_vector[:n]
		print(state_vector)
		#print(len(state_vector))



		(x, y, yaw) = (state_vector[0], state_vector[1], state_vector[2])

		landmarks = state_vector[3:]
		n_landmarks = len(landmarks)//2

		landmarks = np.reshape(landmarks,(n_landmarks,2)).T
		print(landmarks)

		world_landmarksx = [robot_to_world_frame(landmarks[0][i],landmarks[1][i], x, y, yaw)[0] for i in range(len(landmarks[0]))]
		world_landmarksy = [robot_to_world_frame(landmarks[0][i],landmarks[1][i], x, y, yaw)[1] for i in range(len(landmarks[0]))]

		#plt.scatter(world_landmarksx, world_landmarksy)
		plt.scatter(landmarks[0],landmarks[1])
		plt.axis("equal")
		arrowx=[x]
		arrowx.append(x+0.1*np.cos(yaw))
		arrowy=[y]
		arrowy.append(y+0.1*np.sin(yaw))


		plt.plot(arrowx,arrowy, c="r", linewidth=3)
		plt.scatter(x,y,c="r")

		plt.show()

		f1.readline()

		 

	f1.close()


#rostopic echo /p3dx/update > update.txt
