import matplotlib.pyplot as plt
import numpy as np

with open("./coordinates/positionx.txt") as fx:
    stringx = fx.read().splitlines()

with open("./coordinates/positiony.txt") as fy:
    stringy = fy.read().splitlines()

with open("./coordinates/orientation.txt") as fth:
    stringth = fth.read().splitlines()

positionx = []
positiony = []
orientation= []

for i in stringx:
	positionx.append(float(i))
for i in stringy:
	positiony.append(float(i))
for i in stringth:
	orientation.append(float(i))


with open("./coordinates/real_positionx.txt") as fx:
    real_stringx = fx.read().splitlines()

with open("./coordinates/real_positiony.txt") as fy:
    real_stringy = fy.read().splitlines()

with open("./coordinates/real_orientation.txt") as fth:
    real_stringth = fth.read().splitlines()

positionx = []
positiony = []
orientation= []

real_positionx = []
real_positiony = []
real_orientation = []

for i in stringx:
	positionx.append(float(i))
for i in stringy:
	positiony.append(float(i))
for i in stringth:
	orientation.append(float(i))

for i in real_stringx:
	real_positionx.append(float(i))
for i in real_stringy:
	real_positiony.append(float(i))
for i in real_stringth:
	real_orientation.append(float(i))


#fig,axs = plt.subplots(2)


#ax=plt.axes()
n_img=15
it=0
for i,j in zip(range(0,len(positionx),len(positionx)//n_img),range(0,len(real_positionx),len(real_positionx)//n_img)):
	plt.xlim([-4.2,4.3])
	plt.ylim([-1.0,3.4])
	plt.plot(positionx[0:i],positiony[0:i])
	plt.arrow(positionx[i], positiony[i], 0.1*np.cos(orientation[i]), 0.1*np.sin(orientation[i]), head_width=0.05, head_length=0.04, fc='r', ec='r')

	plt.plot(real_positionx[0:j],real_positiony[0:j])
	plt.arrow(real_positionx[j], real_positiony[j], 0.1*np.cos(real_orientation[j]), 0.1*np.sin(real_orientation[j]), head_width=0.05, head_length=0.04, fc='g', ec='g')

	plt.savefig("./coordinates/plot"+str(it)+".png")
	plt.cla()
	it+=1
#plt.show()


#plt.plot(positionx,positiony)
#plt.plot(real_positionx,real_positiony)
#plt.show()