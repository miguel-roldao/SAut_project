import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

with open("./coordinates/positionx.txt") as fx:
    stringx = fx.read().splitlines()

with open("./coordinates/positiony.txt") as fy:
    stringy = fy.read().splitlines()

with open("./coordinates/orientation.txt") as fth:
    stringth = fth.read().splitlines()

with open("./coordinates/l1.txt") as fl1:
	stringl1 = fl1.read().splitlines()

with open("./coordinates/l2.txt") as fl2:
	stringl2 = fl2.read().splitlines()

with open("./coordinates/lth.txt") as flth:
	stringlth = flth.read().splitlines()

positionx = []
positiony = []
orientation= []
l1 = []
l2 = []
lth= []

for i in stringx:
	positionx.append(float(i))
for i in stringy:
	positiony.append(float(i))
for i in stringth:
	orientation.append(float(i))

for i in stringl1:
	l1.append(np.abs(float(i)))
for i in stringl2:
	l2.append(np.abs(float(i)))
for i in stringlth:
	lth.append(float(i))

#print(l1)



with open("./coordinates/real_positionx.txt") as fx:
    real_stringx = fx.read().splitlines()

with open("./coordinates/real_positiony.txt") as fy:
    real_stringy = fy.read().splitlines()

with open("./coordinates/real_orientation.txt") as fth:
    real_stringth = fth.read().splitlines()


real_positionx = []
real_positiony = []
real_orientation = []


for i in real_stringx:
	real_positionx.append(float(i))
for i in real_stringy:
	real_positiony.append(float(i))
for i in real_stringth:
	real_orientation.append(float(i))


#fig,axs = plt.subplots(2)


#ax=plt.axes()
#n_img=25
it=0

print(len(positionx))

for i,j in zip(range(0,len(positionx)),range(0,len(real_positionx))):
	fig, ax = plt.subplots()
	angle = np.float(lth[i]*180/np.pi)
	print(i,positionx[i],positiony[i],l1[i],l2[i],angle)
	ax.set_xlim([-2.2,4.3])
	ax.set_ylim([-2.0,4.4])
	ax.plot(positionx[0:i],positiony[0:i])
	ellipse = Ellipse((positionx[i], positiony[i]), np.sqrt(l1[i]), np.sqrt(l2[i]), angle=angle, alpha=0.8)
	ax.add_artist(ellipse)
	ax.arrow(positionx[i-1], positiony[i-1], 0.4*np.cos(orientation[i-1]), 0.4*np.sin(orientation[i-1]), head_width=0.05, head_length=0.04, fc='r', ec='r')

	ax.plot(real_positionx[0:j],real_positiony[0:j])

	ax.arrow(real_positionx[j-1], real_positiony[j-1], 0.4*np.cos(real_orientation[j-1]), 0.4*np.sin(real_orientation[j-1]), head_width=0.05, head_length=0.04, fc='g', ec='g')

	fig.savefig("./coordinates/plot"+str(it)+".png")
	#plt.cla()
	plt.close(fig)
	it+=1
#plt.show()


#plt.plot(positionx,positiony)
#plt.plot(real_positionx,real_positiony)
#plt.show()