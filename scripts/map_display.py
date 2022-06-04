from matplotlib.markers import MarkerStyle
from pylab import *

PATH_TO_CAT = "/home/miguel_roldao/catkin_ws"   # CHANGE THIS WHEN NEEDED!!!

file = open(PATH_TO_CAT + "/scripts/sample.txt", "r")
content = file.read()
content=np.array(content)


for i in range(1,250):
    y = np.loadtxt(PATH_TO_CAT + "/scripts/coordinates/Y"+ str(i) + ".txt")
    x=np.loadtxt(PATH_TO_CAT + "/scripts/coordinates/X" + str(i) + ".txt")
    plt.scatter(x,y,s=1.0)
    plt.axis('scaled')
    plt.axis('off')
    plt.savefig(PATH_TO_CAT + "/scripts/imagens_mapa/mapa" + str(i))
    plt.close()


