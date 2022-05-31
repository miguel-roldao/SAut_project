from matplotlib.markers import MarkerStyle
from pylab import *

file = open('/home/pedro/catkin_ws/scripts/sample.txt', "r")
content = file.read()
content=np.array(content)


for i in range(1,250):
    y = np.loadtxt("/home/pedro/catkin_ws/scripts/coordinates/Y"+ str(i) + ".txt")
    x=np.loadtxt("/home/pedro/catkin_ws/scripts/coordinates/X" + str(i) + ".txt")
    plt.scatter(x,y)
    plt.savefig("/home/pedro/catkin_ws/scripts/imagens_mapa/mapa" + str(i))
    plt.close()


