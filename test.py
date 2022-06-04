import numpy as np


x=[]
for i in range(10):
	x.append(i)

np.savetxt("../../scripts/coordinates/" + "test" + ".txt", x)