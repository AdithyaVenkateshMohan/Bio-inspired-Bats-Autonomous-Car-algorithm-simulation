import numpy as np
import matplotlib.pyplot as plt
import math


Robotpath = np.array([])


def drawBat(cX, cY, cXdash , cYdash ):
   plt.plot([cX,cXdash],[cY,cYdash])
   plt.show()

def calcVectorArr(Current):
    vecCurrent = np.array([(Current[0, 0] - Current[1, 0]), (Current[0, 1] - Current[1, 1])])
    return vecCurrent

def calcVector(Topoint , Frompoint):
    Vector = np.array([(Topoint[0] - Frompoint[0]), (Topoint[1] - Frompoint[1])])
    return Vector

def calcAngle(m1 , m2):
    tanOfang = abs(((m2-m1)/(1+(m2*m1))))
    ang = math.degrees(math.atan(tanOfang))
    return ang

def calcVectorAngle(Vector1, Vector2 ):

    dot = np.dot(Vector1,Vector2)
    cross = np.cross(Vector1,Vector2)
    tan_theta = cross / dot
    theta = math.atan(tan_theta)
    theta = math.degrees(theta)
    return theta

vec1 = np.array([[0,0],[1,1]])
vec2 = np.array([[0,0],[0,1]])

vec1 = calcVectorArr(vec1)
vec2 = calcVectorArr(vec2)
print(" vec are ", vec1 , " " , vec2)
ang = calcVectorAngle(vec1,vec2)
print(" angle is ", ang)


def calcShortestPoint( Robot , Obstacle):
    if ((Obstacle[0,1]-Obstacle[1,1]) != 0 and (Obstacle[0,0]-Obstacle[1,0]) != 0):

        m = ((Obstacle[0,1]-Obstacle[1,1])/(Obstacle[0,0]-Obstacle[1,0]))

        c1 = (-m*Obstacle[1,0]) + Obstacle[1,1]
        c2 = (1/m)*Robot[0] + Robot[1]


        ShPointX = ((c2- c1)*m)/(pow(m,2)+1)

        ShPointY = (((c1- c2))/(pow(m,2)+1)) + c2
    else:
        if((Obstacle[0,0]-Obstacle[1,0]) == 0):
            ShPointX = Obstacle[0,0]
            ShPointY = Robot[1]

        if ((Obstacle[0,1]-Obstacle[1,1])== 0):
            ShPointX = Robot[0]
            ShPointY = Obstacle[0,1]

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

Robot = np.array([3,3])
Obs = np.array([[-40,1],[-40,200]])
print(calcShortestPoint(Robot,Obs)) # for checking

def rotateRobot(CurrentPath , theta):
    #Rotation of the robot about center
    Center=[CurrentPath[:,0].mean(),CurrentPath[:,1].mean()]
    #translation
    transCurrentfront = CurrentPath[0,:] - Center
    transCurrentback = CurrentPath[1, :] - Center
    #rotation
    theta = math.radians(theta)
    Rotmat = np.array([[math.cos(theta),- math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    RotCurrentfront = np.matmul(Rotmat, transCurrentfront)
    RotCurrentback = np.matmul(Rotmat,transCurrentback)
    Advfront = RotCurrentfront + Center
    Advback = RotCurrentback + Center
    Rotpos = np.array([Advfront, Advback])
    return Rotpos



def pointAtDistance(dist , Current , Direction):

    vecCurrent = [(Current[0, 0] - Current[1, 0]), (Current[0, 1] - Current[1, 1])]
    magCurrent = math.sqrt(math.pow(vecCurrent[0],2)+ math.pow(vecCurrent[1],2))
    unitvecCurrent = np.divide(vecCurrent,magCurrent)


    if ((Current[0, 0] - Current[1, 0]) != 0):
        m = ((Current[0, 1] - Current[1, 1]) / (Current[0, 0] - Current[1, 0]))
        x3 = math.sqrt((pow(dist,2))/(pow(m,2)+1)) + Current[1, 0]
        x32 = - math.sqrt((pow(dist,2))/(pow(m,2)+1)) + Current[1, 0]
        y3 = m*(x3 - Current[1, 0])+ Current[1, 1]
        y32 = m * (x32 - Current[1, 0]) + Current[1, 1]

        x3dash = math.sqrt((pow(dist, 2)) / (pow(m, 2) + 1)) + Current[0, 0]
        x32dash = - math.sqrt((pow(dist, 2)) / (pow(m, 2) + 1)) + Current[0, 0]
        y3dash = m * (x3dash - Current[1, 0]) + Current[1, 1]
        y32dash = m * (x32dash - Current[1, 0]) + Current[1, 1]

    else:
        x3 = Current[1, 0]
        y3 = Current[1, 1] + dist
        x3dash = Current[0,0]
        y3dash = Current[0,1] + dist

        x32 = Current[1, 0]
        y32 = Current[1, 1] - dist
        x32dash = Current[0, 0]
        y32dash = Current[0, 1] - dist


    vecfront = [(x3 - Current[1, 0]), (y3- Current[1, 1])]
    magfront = math.sqrt(math.pow(vecfront[0], 2) + math.pow(vecfront[1], 2))
    print(" vec ", vecfront , " " , magfront)
    unitvecfront = np.divide(vecfront, magfront)
    vecfront2 = [(x32 - Current[1, 0]), (y32 - Current[1, 1])]
    magfront2 = math.sqrt(math.pow(vecfront2[0], 2) + math.pow(vecfront2[1], 2))
    unitvecfront2 = np.divide(vecfront2, magfront2)
    #print(" unit vec cur", unitvecCurrent , " unit vec fr" , unitvecfront, " ", unitvecfront2)
    #print(" all check " , unitvecfront2 == unitvecCurrent , " check " , np.equal(unitvecfront2,unitvecCurrent) , " diff " , np.all(abs(unitvecfront2 - unitvecCurrent ) < 0.00001) )
    if(np.all(abs(unitvecfront - unitvecCurrent ) < 0.00001)):
        print("1")
        if(Direction == "forward" ):
            Futpos = np.array([[x3dash,y3dash],[x3,y3]])
        else:
            Futpos = np.array([[x32dash, y32dash], [x32, y32]])

    if(np.all(abs(unitvecfront2 - unitvecCurrent ) < 0.00001)):
        print("2")
        if (Direction == "forward"):
            Futpos = np.array([[x32dash, y32dash], [x32, y32]])
        else:
            Futpos = np.array([[x3dash, y3dash], [x3, y3]])

    return Futpos

# Robot = np.array([10,10])
# Obs = np.array([[0,2],[5,7]])
# dist = 10
# print(pointAtDistance(dist,Obs))

def generateEcho(rdist,gbat,angle,ear):
    af = 1
    Csei =0
    si =0
    angle = angle /2

    if (ear=="left"):
        Dsei = pow(math.cos(math.radians(angle)),2)
    else:
        Dsei =  pow(math.sin(math.degrees(angle)),2)

    gi = gbat + 40*math.log((0.1/rdist),10) + 2*(rdist-0.1)*af + Dsei + si + Csei
    return gi

def defineVelocity(rdist,Dist_wall):
    print("dist", rdist)

    if (0.3 < rdist < 10):
        Velocity = rdist
    else:
        if(rdist >= 10 ):
            Velocity = 6
        else:
            Velocity = 0.01
    if(rdist <= Dist_wall):
        Angvelo = 100
    else:
        Angvelo = 30

    return Velocity,Angvelo

def calcDistance(point1 , point2):
    dist = math.sqrt((math.pow((point1[0]-point2[0]),2)+math.pow((point1[1]-point2[1]),2)))
    return dist

def controllerRobot(Robot,Obstacle , dist_Wall):

    time_update=1

    RobotLeft = Robot[0]
    RobotRight = Robot[1]
    RobotCenter = (Robot[0]+Robot[1])/2

    Rleft = calcShortestPoint(RobotLeft[0],Obstacle)
    RLdist = calcDistance(Rleft,RobotLeft[0])
    Rright = calcShortestPoint(RobotRight[0], Obstacle)
    RRdist = calcDistance(Rright, RobotRight[0])
    Rcen = calcShortestPoint(RobotCenter[0],Obstacle)
    Rdist = calcDistance(Rcen, RobotCenter[0])

    VectorRobot = calcVectorArr(RobotCenter)
    VectorObstacle = calcVectorArr(Obstacle)

    if((RobotCenter[0,0]-RobotCenter[0,1])==0):
        SlopeRobot = math.inf
    else:
        SlopeRobot = (RobotCenter[0,1] - RobotCenter[1,1])/(RobotCenter[0,0]-RobotCenter[0,1])
    if ((Obstacle[0,0]-Obstacle[0,1]) == 0):
        SlopeObstacle= math.inf
    else:
        SlopeObstacle = (Obstacle[0,1] - Obstacle[1,1])/(Obstacle[0,0]-Obstacle[0,1])

    sei = calcVectorAngle(VectorRobot,VectorObstacle)

    print(" angle", sei)
    gl = generateEcho(RLdist,120,sei,"left")
    gl=1/gl
    gr = generateEcho(RRdist,120,sei,"right")
    gr = 1/gr
    print( "echo ", gl ,gr )

    if(gl < gr ):
        n=1
    else:
        if(gl==gr):
            n=0
        else:
            n=-1

    v,w = defineVelocity(Rdist,dist_Wall)

    if (Rdist > dist_Wall):
        r = -1
        print(" obs to ")
    else:
        r = 1
        print(" obs avoid")
    turn = n*r*w*time_update *0.1
    if(turn > 0 ):
        print("left")
    else:
        print ("right")
    print(" turn ", turn , " ang vel ", w)
    RotLeft = rotateRobot(RobotLeft, turn)
    RotRight = rotateRobot(RobotRight, turn)
    RotCentre = rotateRobot(RobotCenter, turn)


    dist = v*time_update
    print(" velocity ", v, " dis", dist)
    DistLeft =  pointAtDistance(dist, RotLeft,"forward")
    DistRight = pointAtDistance(dist ,RotRight , "forward" )
    DistCenter = pointAtDistance(dist, RotRight , "forward")

    updatedRobot = np.array([DistLeft,DistRight])

    np.append(Robotpath , RobotCenter)


    return updatedRobot







if __name__ == "__main__":
    steps = 0
    RobotLeft = np.array([[0,10],[0,1]])
    RobotRight = np.array([[10,10],[10,1]])
    Robot = np.array([RobotLeft , RobotRight])
    Obstacle_Hor = np.array([[-30,0],[-40,200]])
    updatedRobot = Robot
    while steps <50:
        print("robot path ", updatedRobot)
        plt.plot(Obstacle_Hor[:, 0], Obstacle_Hor[:, 1])
        plt.plot(RobotLeft[:, 0], RobotLeft[:, 1])
        #plt.plot(RobotRight[:, 0], RobotRight[:, 1])
        #plt.plot(Robotpath[:,0],Robotpath[:,1])
        updatedRobot = controllerRobot(Robot,Obstacle_Hor,20)
        RobotLeft = updatedRobot[0]
        RobotRight = updatedRobot[1]
        RobotCenter = (updatedRobot[0] + updatedRobot[1]) / 2
        Robot = updatedRobot
        steps =steps+1


    plt.show()
