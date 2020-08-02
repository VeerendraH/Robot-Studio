import pandas as pd
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import Vector as vctr
import Quaternion as qtrn


import plotly
import plotly.offline as py
import plotly.graph_objs as go

##  ROBOT ARM PACKAGE
#   CLASS - VECTORLIST



#   CLASS - COORDINATES

class Coordinates:
    def __init__(self,array,l = 200):

        self.x = array[0]
        self.y = array[1]
        self.z = array[2]
        self.Phi = array[3]
        self.Theta = array[4]
        self.Psy = array[5]
        self.array = array
        self.l = l

    def contents(self):
        print("x = ",self.x , ";    y = ",self.y , ";   z = ",self.z )
        print("Phi = ",self.Phi , ";    Theta = ",self.Theta , ";   Psy = ",self.Psy )

    def OPQ(self):
        O = vctr.array2vec(self.array[0:3])
        OR0 = vctr.array2vec([0,0,np.sqrt(0.75)*self.l])
        OR1 = qtrn.RotOper(self.Phi,vctr.Vector(0,1,0),OR0)
        OR2 = qtrn.RotOper(self.Theta,vctr.Vector(0,0,1),OR1.purify())

        R = vctr.Add(O,OR2)
        OR0.contents()
        OR1.purify().contents()
        OR2.purify().contents()
        R.contents()


        if self.Phi == 0:
            Pr = qtrn.RotOper(self.Theta,vctr.Vector(0,0,1),vctr.Vector(0,1,0))
            PR0 = vctr.ScalarMul(0.5*self.l,Pr.purify().unit())

        else:
            Pr = vctr.Cross(vctr.Vector(0,0,1),OR2.purify())
            PR0 = vctr.ScalarMul(0.5*self.l,Pr.unit())

        PR = qtrn.RotOper(self.Psy,OR2.purify(),PR0)


        #PR0 = vctr.ScalarMul(0.5*self.l,Pr.unit())



        P = vctr.Add(R,PR.purify())
        Q = vctr.Subtract(R,PR.purify())

        p = np.round(P.array().reshape(1,3),8)
        o = np.round(O.array().reshape(1,3),8)
        q = np.round(Q.array().reshape(1,3),8)
        OP_Array = np.append(o,p,axis = 0)
        OPQ_Array = np.append(OP_Array,q,axis = 0)
        return OPQ_Array

    def Angular(self):
        Angular = self.array[3:6]
        return Angular

#   CLASS - OPQ


class OPQ:
    def __init__(self,matrix):

        self.O = vctr.array2vec(matrix[0,:])
        self.P = vctr.array2vec(matrix[1,:])
        self.Q = vctr.array2vec(matrix[2,:])

    def length(self):
        PQ = vctr.Subtract(self.P,self.Q)
        self.l = PQ.mag()

        return self.l

    def OPQList_In(self,OPQ):
        self.O = OPQ[0]
        self.P = OPQ[1]
        self.Q = OPQ[2]


    def OPQ_Coordinate(self):
        #Spatial
        array = np.zeros((1,6))
        array[0,0:3] = self.O.array()
        #Angular
        self.R = vctr.ScalarMul(0.5,vctr.Add(self.P,self.Q))
        OR_unit = vctr.Subtract(self.R,self.O).unit()
        ORarr = OR_unit.array()
        Phi = np.rad2deg(np.arccos(ORarr[2]))

        Thetar = np.arccos( ORarr[0]/np.sin(np.arccos(ORarr[2])) )
        Theta = np.rad2deg(np.arccos( ORarr[0]/np.sin(np.arccos(ORarr[2]))))

        pq = vctr.Subtract(self.P,self.Q).unit()
        pqarr = pq.array()
        pq0 = vctr.Vector(-np.sin(Thetar),np.cos(Thetar),0).unit()
        cosPsy = vctr.Dot(pq0,pq)

        if vctr.Distance(pq,pq0) > 0.0001:
            sinPsy = vctr.Dot(vctr.Cross(pq0,pq),OR_unit)

            if sinPsy >= 0:

                Psy = np.rad2deg(np.arccos(cosPsy))
            elif sinPsy < 0:
                Psy = 360 - np.rad2deg(np.arccos(cosPsy))

        else:
            Psy = 0


        array[0,3] = Phi
        array[0,4] = Theta
        array[0,5] = Psy
        arrayo = np.append(self.O.array(),[Phi,Theta,Psy])

        self.Coordinate = Coordinates(arrayo,self.length())

        return self.Coordinate

    def O_Coord(self):

        return self.O

    def P_Coord(self):

        return self.P

    def Q_Coord(self):

        return self.Q

    def List_Out(self):
        OPQ = []
        OPQ.append(self.O)
        OPQ.append(self.P)
        OPQ.append(self.Q)

        return OPQ

    def OPQ_Flat_Arr(self):
        return np.hstack((self.O.array(),self.P.array(),self.Q.array()))

    def Angles(self):
        return self.OPQ_Coordinate().Angular()


#       DISPLAY LIST OF VECTORS

def DisplayList(lister):
    for i in lister:
        i.contents()

#       CLASS - ROBOTARM - REPRESENTS CURRENT CONFIGURATION OF ARM

class RobotArm:
    # Initialize the arguments.
    def __init__(self, DOF):

        self.DOF = DOF

        self.Position = []
        self.Axes = []
        self.OPQ = []
        self.Motor_Theta = np.zeros((6,))

    def Array_Input(self,Robot_Initiator):
        DataIn = Robot_Initiator.Array_Output()
        self.Position = DataIn[0]
        self.Axes = DataIn[1]
        self.OPQ = DataIn[2]
        self.Ref = DataIn[3]
        self.Act = DataIn[4]

    # Update the List Arrays with position and ARRAY
    def Change_Position_Vector(self,i,Vector):
        self.Position[i] = Vector

    def Change_Axes_Vector(self,i,Vector):
        self.Axes[i] = Vector

    def Change_OPQ_Vector(self,i,Vector):
        self.OPQ[i] = Vector

    def Change_Ref_Vector(self,i,Vector):
        self.Ref[i] = Vector

    def Change_Act_Vector(self,i,Vector):
        self.Act[i] = Vector

    # Perform an Actuation from a Theta Array

    def True_Actuate(self,Theta,OPQ = []):
        OPQ = self.OPQ

#       Loop over each joint.
        for i in range(0,self.DOF):

#           Get Axis, Position Vectors and Reference Lines as Input
            Axi = self.Axes[i]
            Ri = self.Position[i]

#           Loop over the downstream joints.
            for j in range(i+1,self.DOF):

#               Obtain Joint j Axes,Position Vectors and Reference Vectors
                Axj = self.Axes[j]
                Rj = self.Position[j]
                Refj = self.Ref[j]
                Actj = self.Act[j]

#               Perform Rotations

                Rjnew = Ri + vctr.TransformVector(Theta[i],(Rj - Ri),Axi)
                Axnew = vctr.TransformVector(Theta[i],Axj,Axi)
                Refjnew =  vctr.TransformVector(Theta[i],Refj,Axi)
                Actjnew =  vctr.TransformVector(Theta[i],Actj,Axi)

#               Update the new Vectors
                self.Change_Axes_Vector(j,Axnew)
                self.Change_Position_Vector(j,Rjnew)
                self.Change_Ref_Vector(j,Refjnew)
                self.Change_Act_Vector(j,Actjnew)
#           Actuate the Act Vectors
            Acti = self.Act[i]
            Actinew = vctr.TransformVector(Theta[i],Acti,Axi)
            self.Change_Act_Vector(i,Actinew)
#           Actuate OPQ Vectors
            Oi = OPQ[0]
            Pi = OPQ[1]
            Qi = OPQ[2]

            Oinew = Ri + vctr.TransformVector(Theta[i],(Oi-Ri),Axi)
            Pinew = Ri + vctr.TransformVector(Theta[i],(Pi-Ri),Axi)
            Qinew = Ri + vctr.TransformVector(Theta[i],(Qi-Ri),Axi)

            self.Change_OPQ_Vector(0,Oinew)
            self.Change_OPQ_Vector(1,Pinew)
            self.Change_OPQ_Vector(2,Qinew)

        return

    def Motor_Angle(self):

#       Loop over all joints
        for i in range(0,self.DOF):

            Refi = self.Ref[i].unit()
            Acti = self.Act[i].unit()
            Axi = self.Axes[i].unit()
            self.Motor_Theta[i] = vctr.AnglewrtAxis(Axi,Refi,Acti)

        return self.Motor_Theta

    def Actuate(self,Theta,OPQ = []):
        OPQstore = OPQ
        OPQ = self.OPQ

#       Loop over each Joint
        for i in range(0,self.DOF):

#           Get Axis and Position Vector as Input
            Axi = self.Axes[i]
            Ri = self.Position[i]

#           Loop over the remaining Joints.
            for j in range(i+1,self.DOF):

#               Obtain 2nd loop Joint Axis and Position Vector.
                Axj = self.Axes[j]
                Rj = self.Position[j]


#               Obtain inputs for Rotation

#               Perform Rotations

                Rjnew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Rj,Ri),Axi))
                Axnew = vctr.TransformVector(Theta[i],Axj,Axi)

#               Update the new Axes
                self.Change_Axes_Vector(j,Axnew)
                self.Change_Position_Vector(j,Rjnew)

            Oi = OPQ[0]
            Pi = OPQ[1]
            Qi = OPQ[2]

            Oinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Oi,Ri),Axi))
            Pinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Pi,Ri),Axi))
            Qinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Qi,Ri),Axi))

            self.Change_OPQ_Vector(0,Oinew)
            self.Change_OPQ_Vector(1,Pinew)
            self.Change_OPQ_Vector(2,Qinew)

        return

    def Actuate_Sim(self,Theta):
        AxesSim = self.Axes
        PosiSim = self.Position
        OPQ = self.OPQ

        O1 = OPQ[0]
        P1 = OPQ[1]
        Q1 = OPQ[2]

        #       Loop over each Joint
        for i in range(0,self.DOF):

            #Get Axis and Position Vector as Input
            Axi = AxesSim[i]
            Ri = PosiSim[i]

#           Loop over the remaining Joints.
            for j in range(i+1,self.DOF):

#               Obtain 2nd loop Joint Axis and Position Vector.
                Axj = AxesSim[j]
                Rj = PosiSim[j]


#               Obtain inputs for Rotation

#               Perform Rotations

                Rjnew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Rj,Ri),Axi))
                Axnew = vctr.TransformVector(Theta[i],Axj,Axi)

#               Update the new Axes
                AxesSim[j] = Axnew
                PosiSim[j] = Rjnew

            Oi = OPQ[0]
            Pi = OPQ[1]
            Qi = OPQ[2]

            Oinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Oi,Ri),Axi))
            Pinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Pi,Ri),Axi))
            Qinew = vctr.Add(Ri , vctr.TransformVector(Theta[i],vctr.Subtract(Qi,Ri),Axi))

            OPQ[0] = Oinew
            OPQ[1] = Pinew
            OPQ[2] = Qinew


        fO = vctr.Subtract(Oinew , O1).unit()
        fP = vctr.Subtract(Pinew , P1).unit()
        fQ = vctr.Subtract(Qinew , Q1).unit()

        f = [fO,fP,fQ]


        return OPQ,f

    def Cost_Calc(self,Theta,i,OPQ2):

        O1 = self.OPQ[0]
        O2 = OPQ2.O
        P1 = self.OPQ[1]
        P2 = OPQ2.P
        Q1 = self.OPQ[2]
        Q2 = OPQ2.Q

        OPQ2lis = [O2,P2,Q2]

        Theta_inc = np.zeros((1,6))
        inc = (0.0001)
        Theta_inc[0,i] = 0.0001

        Sim_Res = self.Actuate_Sim(Theta.flatten())

        Sim_Res_inc = self.Actuate_Sim((Theta+Theta_inc).flatten())

        Jprime = 0
        J = 0
        for j in range(0,3):
            V1 = vctr.Subtract(Sim_Res_inc[0][j],Sim_Res[0][j])
            V2 = vctr.Subtract(Sim_Res[0][j],OPQ2lis[j])
            Jprime = Jprime + vctr.Dot(V1,V2)
            delO = vctr.Subtract(Sim_Res[0][j],OPQ2lis[j])
            J = J + vctr.Dot(delO,delO)

        return J,Jprime


    # Give input for Visualisation
    def Return_Point_Array(self):

        Pos_Arr = np.zeros((self.DOF,3))
        for i in range(0,self.DOF):
            Pos_Arr[i,:] = self.Position[i].array()
        Pos_Arr.reshape(6,3)

        OPQ_Arr = np.zeros((3,3))
        for i in range(0,3):
            OPQ_Arr[i,:] = self.OPQ[i].array()
        OPQ_Arr.reshape(3,3)
        OPQ_Arr = np.append(OPQ_Arr,self.OPQ[0].array().reshape(1,-1),axis = 0)

        Point_Arr = np.append(Pos_Arr,OPQ_Arr,axis = 0)

        return Point_Arr

    # Give Input for OPQ Output
    def Return_OPQ_Array(self):

        OPQ_Arr = np.zeros((3,3))
        for i in range(0,3):
            OPQ_Arr[i,:] = self.OPQ[i].array()
        OPQ_Arr.reshape(3,3)

        return OPQ_Arr

    # Visualisation of current Configuration
    def Visualise(self):

        Pos_Array = self.Return_Point_Array()
        print(Pos_Array,'##########################')

        trace = go.Scatter3d(
    x=Pos_Array[:,0], y=Pos_Array[:,1], z=Pos_Array[:,2],
    marker=dict(
        size=4,
        color=1,
        colorscale='Viridis',
    ),
    line=dict(
        color='#1f77b4',
        width=1
    )
        )

        data = [trace]

        layout = dict(
            width=800,
            height=800,
            autosize=True,
            title='The Arm',
            scene=dict(
                xaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                yaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                zaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                camera=dict(
                    up=dict(
                        x=0,
                        y=0,
                        z=1
                    ),
                    eye=dict(
                        x=1,
                        y=-1,
                        z=1,
                    )
                ),
                aspectratio = dict( x=1, y=1, z=1 ),
                aspectmode = 'manual'
            ),
        )

        fig = dict(data=data, layout=layout)

        py.iplot(fig, filename='pandas-brownian-motion-3d')

#   CLASS - ROBOT INITIATOR

class Robot_Initiator:
    def __init__(self,DOF):
        self.DOF = DOF

        self.Position = []

        for i in range(0,self.DOF):
            self.Position.append(vctr.Vector(0,0,0))

        self.Axes = []

        for i in range(0,self.DOF):
            self.Axes.append(vctr.Vector(0,0,0))

        self.OPQ = []

        for i in range(0,3):
            self.OPQ.append(vctr.Vector(0,0,0))

        self.Ref = []

        for i in range(0,self.DOF):
            self.Ref.append(vctr.Vector(0,0,0))

        self.Act = []

        for i in range(0,self.DOF):
            self.Act.append(vctr.Vector(0,0,0))
#   Input from Arrays
    def Array_Input(self,Pos_Arr,Axs_Arr,OPQ_Arr,Ref_Arr,Act_Arr):

        for i in range(0,self.DOF):
            self.Position[i] = vctr.array2vec(Pos_Arr[i,:])

        for i in range(0,self.DOF):
            self.Axes[i] = vctr.array2vec(Axs_Arr[i,:])

        for i in range(0,3):
            self.OPQ[i] = vctr.array2vec(OPQ_Arr[i,:])

        for i in range(0,self.DOF):
            self.Ref[i] = vctr.array2vec(Ref_Arr[i,:])

        for i in range(0,self.DOF):
            self.Act[i] = vctr.array2vec(Act_Arr[i,:])

        return

#   Output the Data as a List
    def Array_Output(self):
        self.Data = []
        self.Data.append(self.Position)
        self.Data.append(self.Axes)
        self.Data.append(self.OPQ)
        self.Data.append(self.Ref)
        self.Data.append(self.Act)

        return self.Data

#   Plot The End Effector OPQ Triangles
def Triangle_Plot(a,fig):

    b1 = [0,3,6,0]
    b2 = [1,4,7,1]
    b3 = [2,5,8,2]
    a1 = a[:,b1]
    a2 = a[:,b2]
    a3 = a[:,b3]

    for i in range(0,a.shape[0]):

        fig.add_trace(go.Scatter3d(
        x=a1[i,:], y= a2[i,:], z=a3[i,:],
        marker=dict(
            size=2,
            color=1,
            colorscale='Viridis',
        ),
        line=dict(
            color='#f0465c',
            width=1
        )
            ))

#   CLASS - ACTUATOR MATRIX

class Actuator_Matrix:
    def __init__(self):
        self.Matrix = []

    def Initialise(self,Array):
        for i in range(0,np.size(Array,0)):
            self.Matrix.append(Array[i])
#   Give out Actuator Sequence no. i
    def Theta_Unroll(self,i):
        arr = self.Matrix[i]
        return arr

    def Length(self):
        return len(self.Matrix)

#   CLASS - ROBOT ARM CHART
class Robot_Arm_Chart:
    def __init__(self,Actuator_Matrix,Robot_Initiator):

        self.Actuator_Matrix = Actuator_Matrix
        self.Robot_Initiator = Robot_Initiator
        self.Length = Actuator_Matrix.Length()
        self.Track_Position = []
        self.Track_Axes = []
        self.Track_OPQ = []

    def Initialise(self):
        self.RO = RobotArm(6)
        self.RO.Array_Input(self.Robot_Initiator)
        self.Track_Position.append(self.RO.Position)
        self.Track_Axes.append(self.RO.Axes)
        self.Track_OPQ.append(self.RO.Return_OPQ_Array())

    def Run_Samples(self):
#       Loop over the Actuator Matrix contents
        for i in range(0,self.Length):

            Theta = self.Actuator_Matrix.Theta_Unroll(i)
            self.RO.True_Actuate(Theta)
            Out = self.RO.Return_OPQ_Array()
            self.Track_Position.append(self.RO.Position)
            self.Track_Axes.append(self.RO.Axes)
            self.Track_OPQ.append(Out)

        self.Data_Out = []
        self.Data_Out.append(self.Track_Position)
        self.Data_Out.append(self.Track_Axes)
        self.Data_Out.append(self.Track_OPQ)

        return self.Data_Out

    def Array_Output(self):
        O_Out = np.zeros((len(self.Track_OPQ),9))
        count = 0

        for i in range(0,10) :

            O_Out[i][0:3] = self.Track_OPQ[i][0]
            O_Out[i][3:6] = self.Track_OPQ[i][1]
            O_Out[i][6:9] = self.Track_OPQ[i][2]
            count = count + 1

        return O_Out



    def Visualise(self):
        #O
        Pos_Array = self.Array_Output()

        fig = go.Figure()

        fig.add_trace(go.Scatter3d(
    x=Pos_Array[:,3], y=Pos_Array[:,4], z=Pos_Array[:,5],
    marker=dict(
        size=4,
        color=1,
        colorscale='Viridis',
    ),
    line=dict(
        color='#1fb49b',
        width=1
    )
        ))

        fig.add_trace(go.Scatter3d(
    x=Pos_Array[:,0], y=Pos_Array[:,1], z=Pos_Array[:,2],
    marker=dict(
        size=4,
        color=1,
        colorscale='Viridis',
    ),

    line=dict(
        color='#1f77b4',
        width=1
    )
        ))

        fig.add_trace(go.Scatter3d(
    x=Pos_Array[:,6], y=Pos_Array[:,7], z=Pos_Array[:,8],
    marker=dict(
        size=4,
        color=1,
        colorscale='Viridis',
    ),
    line=dict(
        color='#801fb4',
        width=1
    )
        ))

        #data = [trace]

        layout = dict(
            width=800,
            height=800,
            autosize=True,
            title='The Arm',
            scene=dict(
                xaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                yaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                zaxis=dict(
                    range = [-1500,1500],
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                camera=dict(
                    up=dict(
                        x=0,
                        y=1,
                        z=0
                    ),
                    eye=dict(
                        x=-1,
                        y=1,
                        z=-1,
                    )
                ),
                aspectratio = dict( x=1, y=1, z=1 ),
                aspectmode = 'manual'
            ),
        )

        #fig = dict(data=data, layout=layout)

        #py.iplot(fig, filename='pandas-brownian-motion-3d')
        Triangle_Plot(Pos_Array,fig)
        fig.show()

        return fig


#       VISUALISATION FROM POS ARRAY
def Visualise(Pos_Array):
    #O


    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
x=Pos_Array[:,3], y=Pos_Array[:,4], z=Pos_Array[:,5],
marker=dict(
    size=4,
    color=1,
    colorscale='Viridis',
),
line=dict(
    color='#1fb49b',
    width=1
)
    ))

    fig.add_trace(go.Scatter3d(
x=Pos_Array[:,0], y=Pos_Array[:,1], z=Pos_Array[:,2],
marker=dict(
    size=4,
    color=1,
    colorscale='Viridis',
),

line=dict(
    color='#1f77b4',
    width=1
)
    ))

    fig.add_trace(go.Scatter3d(
x=Pos_Array[:,6], y=Pos_Array[:,7], z=Pos_Array[:,8],
marker=dict(
    size=4,
    color=1,
    colorscale='Viridis',
),
line=dict(
    color='#801fb4',
    width=1
)
    ))

    #data = [trace]

    layout = dict(
        width=800,
        height=800,
        autosize=True,
        title='The Arm',
        scene=dict(
            xaxis=dict(
                range = [-1500,1500],
                gridcolor='rgb(255, 255, 255)',
                zerolinecolor='rgb(255, 255, 255)',
                showbackground=True,
                backgroundcolor='rgb(230, 230,230)'
            ),
            yaxis=dict(
                range = [-1500,1500],
                gridcolor='rgb(255, 255, 255)',
                zerolinecolor='rgb(255, 255, 255)',
                showbackground=True,
                backgroundcolor='rgb(230, 230,230)'
            ),
            zaxis=dict(
                range = [-1500,1500],
                gridcolor='rgb(255, 255, 255)',
                zerolinecolor='rgb(255, 255, 255)',
                showbackground=True,
                backgroundcolor='rgb(230, 230,230)'
            ),
            camera=dict(
                up=dict(
                    x=0,
                    y=1,
                    z=0
                ),
                eye=dict(
                    x=-1,
                    y=1,
                    z=-1,
                )
            ),
            aspectratio = dict( x=1, y=1, z=1 ),
            aspectmode = 'manual'
        ),
    )

    Triangle_Plot(Pos_Array,fig)
    fig.show()
'''
def MatPlotPosarray(Pos):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for i in range(0,Pos.shape[0]):
        xi = Pos[i,0]
        yi = Pos[i,1]
        zi = Pos[i,2]
        ax.scatter(xi,yi,zi,marker = 'o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ea.set_axes_equal(ax)
    plt.show()
'''
