import numpy as np
import matplotlib.pyplot as plt
import math




pos = np.random.randint(1,50,size=(2,5),dtype='int')
print(" pos",pos, "shape",pos.shape)
plt.plot(pos[0,:],pos[1,:])
plt.show()

def drawBat(cX, cY, cXdash , cYdash ):
   plt.plot([cX,cXdash],[cY,cYdash])
   plt.show()


def calcAngle(m1 , m2):
    tanOfang = abs(((m2-m1)/(1+(m2*m1))))
    ang = math.degrees(math.atan(tanOfang))
    return ang

def calcShortestPoint( Robot , Obstacle):

    m = ((Obstacle[0,1]-Obstacle[1,1])/(Obstacle[0,0]-Obstacle[1,0]))

    c1 = (-m*Obstacle[1,0]) + Obstacle[1,1]
    c2 = (1/m)*Robot[0] + Robot[1]


    ShPointX = ((c2- c1)*m)/(pow(m,2)+1)

    ShPointY = (((c1- c2))/(pow(m,2)+1)) + c2

    print("sh",ShPointX,ShPointY)

    vectorObstacle=[Obstacle[0,0]-Obstacle[1,0] , Obstacle[0,1]-Obstacle[1,1]]
    vectorSh = [ShPointX-Obstacle[1,0] , ShPointY -Obstacle[1,1]]
    dotvecObs = np.dot(vectorObstacle,vectorObstacle)
    dotvecSh = np.dot(vectorObstacle,vectorSh)
    if (dotvecObs <= dotvecSh):
        shortestPt =[Obstacle[0,0],Obstacle[0,1]]
    else:
        if (dotvecSh <= 0):
            shortestPt = [Obstacle[1,0] , Obstacle[1,1]]
        else:
            shortestPt = [ShPointX , ShPointY]

    return shortestPt

Robot = np.array([2,2])
Obs = np.array([[1,3],[4,6]])
print(calcShortestPoint(Robot,Obs))