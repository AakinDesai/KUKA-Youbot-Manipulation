import numpy as np
import modern_robotics as mr
import logging

logging.basicConfig(filename = "runNewtask.log", level = logging.INFO, format = '%(asctime)s  %(processName)s %(name)s  %(message)s')
logging.info("Running newTask version")


# Milestone 1

def NextState(currentconfiguration, controls, dt, maximum):
    matrix = np.zeros(12)
    for i in range(5, 9):
        if controls[i] < -maximum:
            controls[i] = -maximum      # Setting Wheel Speed Limit
        elif controls[i] > maximum:
            controls[i] = maximum

    matrix[3:12] = currentconfiguration[3:12] + (dt*controls[0:9])  # Calculating next step joint angle and wheel speed
    A=np.transpose(matrix[8:12]-currentconfiguration[8:12])
    B = np.dot(HI, A)
    v = np.zeros(6)
    v[2:5] = B
    se3mat = mr.VecTose3(v)       # Twist
    T = mr.MatrixExp6(se3mat)     # Tbb+1 Matrix 
    matrix[0] = currentconfiguration[0] + np.arccos(T[0, 0])
    matrix[1] = currentconfiguration[1] + T[0, 3]
    matrix[2] = currentconfiguration[2] + T[1, 3]
    return matrix

# Milestone 2

def TrajectoryGenerator(E,C,F,G,S):
    Xstart1 = np.copy(E)     # Trajectory 1 Starting Point :- Tseinitial
    Xend2 = np.dot(C,G)   # Trajectory 2 Ending Point :- Tsg (Endeffector Poistion during Grasping) 
    Xend1 = np.dot(C,S)   # Trajectory 1 Ending Point :- Tss (Endeffector Poistion during Standoff)  
    Xend6 = np.dot(F,G)   # Tsg at Cube Goal (grasp)
    Xend5 = np.dot(F,S)   # Tss at Cube Goal(Standoff)
    
    N2=200
    t2=N2/100         # Time and Number of Iteration for each Trajectory
    N5=1600
    t5=N5/100
    
    trac1=mr.CartesianTrajectory(Xstart1, Xend1, 6, 600, 5)
    trac2=mr.CartesianTrajectory(Xend1,Xend2,t2,N2,5)
    trac4=mr.CartesianTrajectory(Xend2,Xend1,t2,N2,5)
    trac5=mr.CartesianTrajectory(Xend1, Xend5, t5, N5, 5)       # Finding Trajectory using MR Function
    trac6=mr.CartesianTrajectory(Xend5,Xend6,t2,N2,5)
    trac8=mr.CartesianTrajectory(Xend6,Xend5,t2,N2,5)

    matrix1=np.zeros((600,13))
    matrix2=np.zeros((N2,13))
    matrix3=np.zeros((65,13))
    matrix4=np.zeros((N2,13))          
    matrix5=np.zeros((N5,13))
    matrix6=np.zeros((N2,13))
    matrix7=np.zeros((65,13))
    matrix8=np.zeros((N2,13))
   



    for i in range(600):
        matrix1[i][0]=trac1[i][0][0]
        matrix1[i][1]=trac1[i][0][1]
        matrix1[i][2]=trac1[i][0][2]
        matrix1[i][3]=trac1[i][1][0]
        matrix1[i][4]=trac1[i][1][1]
        matrix1[i][5]=trac1[i][1][2]
        matrix1[i][6]=trac1[i][2][0]
        matrix1[i][7]=trac1[i][2][1]
        matrix1[i][8]=trac1[i][2][2]
        matrix1[i][9]=trac1[i][0][3]
        matrix1[i][10]=trac1[i][1][3]
        matrix1[i][11]=trac1[i][2][3]
        matrix1[i][12]=0

    for i in range(N2):
        matrix2[i][0]=trac2[i][0][0]
        matrix2[i][1]=trac2[i][0][1]
        matrix2[i][2]=trac2[i][0][2]
        matrix2[i][3]=trac2[i][1][0]
        matrix2[i][4]=trac2[i][1][1]
        matrix2[i][5]=trac2[i][1][2]
        matrix2[i][6]=trac2[i][2][0]
        matrix2[i][7]=trac2[i][2][1]
        matrix2[i][8]=trac2[i][2][2]
        matrix2[i][9]=trac2[i][0][3]
        matrix2[i][10]=trac2[i][1][3]
        matrix2[i][11]=trac2[i][2][3]
        matrix2[i][12]=0

    for i in range(65):
        matrix3[i,0:12]=matrix2[N2-1,0:12]
        matrix3[i,12]=1

    for i in range(N2):
        matrix4[i][0]=trac4[i][0][0]
        matrix4[i][1]=trac4[i][0][1]
        matrix4[i][2]=trac4[i][0][2]
        matrix4[i][3]=trac4[i][1][0]
        matrix4[i][4]=trac4[i][1][1]
        matrix4[i][5]=trac4[i][1][2]
        matrix4[i][6]=trac4[i][2][0]
        matrix4[i][7]=trac4[i][2][1]
        matrix4[i][8]=trac4[i][2][2]
        matrix4[i][9]=trac4[i][0][3]
        matrix4[i][10]=trac4[i][1][3]
        matrix4[i][11]=trac4[i][2][3]
        matrix4[i][12]=1

    for i in range(N5):
        matrix5[i][0]=trac5[i][0][0]
        matrix5[i][1]=trac5[i][0][1]
        matrix5[i][2]=trac5[i][0][2]
        matrix5[i][3]=trac5[i][1][0]
        matrix5[i][4]=trac5[i][1][1]
        matrix5[i][5]=trac5[i][1][2]
        matrix5[i][6]=trac5[i][2][0]
        matrix5[i][7]=trac5[i][2][1]
        matrix5[i][8]=trac5[i][2][2]
        matrix5[i][9]=trac5[i][0][3]
        matrix5[i][10]=trac5[i][1][3]
        matrix5[i][11]=trac5[i][2][3]
        matrix5[i][12]=1

    for i in range(N2):
        matrix6[i][0]=trac6[i][0][0]
        matrix6[i][1]=trac6[i][0][1]
        matrix6[i][2]=trac6[i][0][2]
        matrix6[i][3]=trac6[i][1][0]
        matrix6[i][4]=trac6[i][1][1]
        matrix6[i][5]=trac6[i][1][2]
        matrix6[i][6]=trac6[i][2][0]
        matrix6[i][7]=trac6[i][2][1]
        matrix6[i][8]=trac6[i][2][2]
        matrix6[i][9]=trac6[i][0][3]
        matrix6[i][10]=trac6[i][1][3]
        matrix6[i][11]=trac6[i][2][3]
        matrix6[i][12]=1

    for i in range(65):
        matrix7[i,0:12]=matrix6[N2-1,0:12]
        matrix7[i,12]=0

    for i in range(N2):
        matrix8[i][0]=trac8[i][0][0]
        matrix8[i][1]=trac8[i][0][1]
        matrix8[i][2]=trac8[i][0][2]
        matrix8[i][3]=trac8[i][1][0]
        matrix8[i][4]=trac8[i][1][1]
        matrix8[i][5]=trac8[i][1][2]
        matrix8[i][6]=trac8[i][2][0]
        matrix8[i][7]=trac8[i][2][1]
        matrix8[i][8]=trac8[i][2][2]
        matrix8[i][9]=trac8[i][0][3]
        matrix8[i][10]=trac8[i][1][3]
        matrix8[i][11]=trac8[i][2][3]
        matrix8[i][12]=0
    finalmatrix=np.vstack((matrix1,matrix2,matrix3,matrix4,matrix5,matrix6,matrix7,matrix8))  # Storing all 12 configuration and gripper state in Matrix
    return finalmatrix

# Milestone 3

def FeedbackControl(X,XD,XDN,KP,KI,dt):
    
    XDI=np.linalg.inv(XD)
    XDDN=np.dot(XDI,XDN)
    se3VD=(1/dt)*mr.MatrixLog6(XDDN)       # Calculating Feedforward Twist in desired frame
    VD=mr.se3ToVec(se3VD)
    
    XI=np.linalg.inv(X)
    XIXD=np.dot(XI,XD)                 # Calculating Adjoint for converting twist in desired frame to end effector frame   
    A= mr.Adjoint(XIXD) 
    
    se3XE=mr.MatrixLog6(XIXD)
    XE=mr.se3ToVec(se3XE)                # Error twist
    
    I=dt*XE
    V = np.dot(A,VD) + np.dot(KP,XE) + np.dot(KI,I)      # Twist in end effector frame using PID controller
    #V=np.dot(KP,XE) + np.dot(KI,I)                        

    
    T0E = mr.FKinBody(M,Blist,thetalist)
    T0EI=np.linalg.inv(T0E)
    TB0I=np.linalg.inv(TB0)
    TEB=np.dot(T0EI,TB0I)
    C= mr.Adjoint(TEB)

    FM= 0.011875*np.array([[ -2.5974, 2.5974,  2.5974, -2.5974],
                 [ 1, 1,  1, 1],
                 [-1, 1,  -1, 1],
             ])

    F6=np.zeros((6,4))
    F6[2:5,:]=FM

    JB=np.dot(C,F6)                                 # Calculating Base Jacobian
    JA =mr.JacobianBody(Blist,thetalist)               # Calculating Arm Jacobian

    JE=np.zeros((6,9))
    JE[:,0:5]=JA
    JE[:,5:9]=JB

    JEI=np.linalg.pinv(JE)
    Matrix=np.dot(JEI,V)                          # Calculating speed and angular velocity 
    return Matrix

def matrixfromT(T1):
    
    Y=np.zeros((4,4))
    Y[0,0]=T1[0]
    Y[0,1]=T1[1]
    Y[0,2]=T1[2]
    Y[1,0]=T1[3]
    Y[1,1]=T1[4]                            # Convert Trajectory back to Tse Matrix
    Y[1,2]=T1[5]
    Y[2,0]=T1[6]
    Y[2,1]=T1[7]
    Y[2,2]=T1[8]
    Y[0,3]=T1[9]
    Y[1,3]=T1[10]
    Y[2,3]=T1[11]
    Y[3,0:3]=0
    Y[3,3]=1
    
    return Y

HI = 0.011875*np.array([[ -2.5974, 2.5974,  2.5974, -2.5974],
                  [ 1, 1,  1, 1],
                  [-1, 1,  -1, 1],
              ])


M = np.array([[1, 0,  0, 0.033],
                      [ 0, 1,  0, 0],
                      [ 0, 0, 1, 0.6546],
                      [ 0, 0,  0, 1]])

Blist = np.array([[0, 0, 1, 0, 0.033,   0],
                          [0, -1,  0, -0.5076, 0,   0],
                          [0, -1,  0, -0.3526, 0, 0],
                          [0, -1,  0, -0.2176, 0, 0],
                  [0, 0,  1, 0, 0, 0]]).T
#Configuration of Cube
Xcubeinitial=np.array([1,0,0])
Xcubefinal=np.array([0,-1,-np.pi/2])

#Configuration of Cube for new task
#Xcubeinitial=np.array([1,1,np.pi/4])
#Xcubefinal=np.array([0,0,-np.pi/4])

Tsci=np.array([[ np.cos(Xcubeinitial[2]), -np.sin(Xcubeinitial[2]),  0, Xcubeinitial[0]],
                 [ np.sin(Xcubeinitial[2]), np.cos(Xcubeinitial[2]),  0, Xcubeinitial[1]],
                 [0, 0,  1, 0.025],
                 [0, 0,  0, 1]
             ])

Tscf=np.array([[ np.cos(Xcubefinal[2]), -np.sin(Xcubefinal[2]),  0, Xcubefinal[0]],
                 [ np.sin(Xcubefinal[2]), np.cos(Xcubefinal[2]),  0, Xcubefinal[1]],
                 [0, 0,  1, 0.025],
                 [0, 0,  0, 1]
             ])
                                                                                          # Defining Initial Configuartion and Other Required Details

Tceg = np.array([[-np.sqrt(0.5), 0, np.sqrt(0.5), 0],
                  [0, 1, 0, 0],
                  [-np.sqrt(0.5), 0, -np.sqrt(0.5), 0],
                  [0, 0, 0,   1]])

Tces = np.array([[-np.sqrt(0.5), 0, np.sqrt(0.5), 0],
                  [0, 1, 0, 0],
                  [-np.sqrt(0.5), 0, -np.sqrt(0.5), 0.25],
                  [0, 0, 0,   1]])

Tse = np.array([[0, 0, 1, 0],
                  [0, 1, 0, 0],
                  [-1, 0, 0, 0.5],
                  [0, 0, 0,   1]])

TB0 = np.array([[ 1, 0,  0, 0.1662],
                 [ 0, 1,  0, 0],
                 [0, 0,  1, 0.0026],
                 [0, 0,  0, 1]
             ])

Xi=np.array([0,0.1,0,0,0.85,0.2,-1.6,0,0,0,0,0])
thetalist = Xi[3:8]


Tsb=np.array([[ np.cos(Xi[0]), -np.sin(Xi[0]),  0, Xi[1]],
                 [ np.sin(Xi[0]), np.cos(Xi[0]),  0, Xi[2]],
                 [0, 0,  1, 0.0963],
                 [0, 0,  0, 1]
             ])

X=np.dot(Tsb,np.dot(TB0,mr.FKinBody(M,Blist,thetalist)))


Trajectory=TrajectoryGenerator(Tse,Tsci,Tscf,Tceg,Tces)   # Trajectory Generated
logging.info("Trajectory Generated")



length=len(Trajectory)
KP1=7*np.identity(6)
KI1=0.3*np.identity(6)
dt=0.01
FinalMatrix=np.zeros((length,13))
FinalMatrix[0,0:12]=Xi
FinalMatrix[0,12]=0

XE1=np.zeros((6,length-1))


for i in range(length-1):
    
    Xd=matrixfromT(Trajectory[i,:])
    Xdn=matrixfromT(Trajectory[i+1,:])
    
    XI1=np.linalg.inv(X)
    XIXD1=np.dot(XI1,Xd)                                 # Calculating Error 
    se3XE1=mr.MatrixLog6(XIXD1)
    XE1[:,i]=np.transpose(mr.se3ToVec(se3XE1))
    
    speed=FeedbackControl(X,Xd,Xdn,KP1,KI1,dt)
    Xi=NextState(Xi,speed,dt,12.3)
    thetalist=Xi[3:8]
    Tsb=np.array([[ np.cos(Xi[0]), -np.sin(Xi[0]),  0, Xi[1]],
                 [ np.sin(Xi[0]), np.cos(Xi[0]),  0, Xi[2]],
                 [0, 0,  1, 0.0963],
                 [0, 0,  0, 1]
             ])
    X=np.dot(Tsb,np.dot(TB0,mr.FKinBody(M,Blist,thetalist)))
    FinalMatrix[i+1,0:12]=Xi
    FinalMatrix[i+1,12]=Trajectory[i,12]                              # Storing a configuartion value in a Matrix
    

logging.info("Generating animation csv file")  
CSVmatrix=np.array(FinalMatrix)
np.savetxt("FA2",CSVmatrix, delimiter=",")

logging.info("Writing error plot data")
CSVmatrix=np.array(np.transpose(XE1))
np.savetxt("EA1",CSVmatrix, delimiter=",")



    
import matplotlib.pyplot as plt 

x = [i for i in range(length-1)]

y1=XE1[0]
y2=XE1[1]
y3=XE1[2]
y4=XE1[3]
y5=XE1[4]
y6=XE1[5]

plt.plot(x, y1,label = "Error in Wx")
plt.plot(x, y2,label = "Error in Wy") 
plt.plot(x, y3,label = "Error in Wz") 
plt.plot(x, y4,label = "Error in Vx") 
plt.plot(x, y5,label = "Error in Vy") 
plt.plot(x, y6,label = "Error in Vz") 


plt.xlabel('x - axis') 
plt.ylabel('y - axis') 
  
plt.title('Error graph') 
plt.legend() 
  
plt.show() 

logging.info("Done")



        