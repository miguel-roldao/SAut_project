import numpy as np
from matplotlib import pyplot as plt
from sklearn import linear_model, datasets

def fitar(X,Y):
    # Robustly fit linear model with RANSAC algorithm
    plt.scatter(X,Y,s=0.5)
    X= np.reshape(X, (len(X), 1))
    Y= np.reshape(Y,(len(Y),1))
    ransac = linear_model.RANSACRegressor()
    ransac.fit(X,Y)
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    # Predict data of estimated models
    line_X = np.arange(X.min(), X.max())[:, np.newaxis]
    line_y_ransac = ransac.predict(line_X)

    plt.plot(line_X,line_y_ransac,color="cornflowerblue",label="RANSAC regressor",)
    plt.savefig("/home/pedro/catkin_ws/scripts/imagens_mapa/Ransac")
    #plt.show()

    return line_X,line_y_ransac,inlier_mask,outlier_mask