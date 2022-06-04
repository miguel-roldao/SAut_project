from matplotlib.markers import MarkerStyle
from pylab import *

#If there is any error with the path try to generalize it. miguel_roldao

file = open("./sample.txt", "r")
content = file.read()
content=np.array(content)


for i in range(1,250):
    y = np.loadtxt("./coordinates/Y"+ str(i) + ".txt")
    x=np.loadtxt("./coordinates/X" + str(i) + ".txt")
    plt.scatter(x,y,s=1.0)
    plt.axis('scaled')
    plt.axis('off')
    plt.savefig("./imagens_mapa/mapa" + str(i))
    plt.close()


