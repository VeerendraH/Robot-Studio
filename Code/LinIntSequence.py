import pandas as pd
import numpy as np
import Robot_Arm_Library as ral
import Vector as vctr
import Rotation_Library as rotl


import plotly
import plotly.offline as py
import plotly.graph_objs as go
#   FOR ANY TWO ORIENTATIONS GIVEN AS OPQ, WE GENERATE THE INTERMEDIATE OPQ AS A FUNCTION OF NUMBER OF TRIALS.

#   OPQ  TO COORDINATES

#   ONE - DIMENSIONAL MOVEMENT IN COORDINATES

filename = "OPQTrialData.csv"
Points = pd.read_csv(filename)


OPQ1_mat = Points[['X1','Y1','Z1']].to_numpy()
OPQ2_mat = Points[['X2','Y2','Z2']].to_numpy()

N = 50 #Number of Steps

OPQ1 = ral.OPQ(OPQ1_mat)
OPQ2 = ral.OPQ(OPQ2_mat)

#Spatial Reconstruction
Coordinate1 = OPQ1.O_Coord()
Coordinate2 = OPQ2.O_Coord()

l = OPQ1.length()
CoordinatePath = []
Coord_Matrix = np.zeros((N+1,3))
Coord_Matrix[0,:] = Coordinate1.array()

for i in range(0,N+1):
    t = (i)/(N)
    Vector_i = vctr.Add(vctr.ScalarMul((1-t),Coordinate1),vctr.ScalarMul((t),Coordinate2))
    CoordinatePath.append(Vector_i)
    Coord_Matrix[i,:] = Vector_i.array()

Pos_Array = Coord_Matrix
'''
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

fig = dict(data=data, layout=layout)

py.iplot(fig, filename='pandas-brownian-motion-3d')
'''



#Angular Reconstruction


Angles1 = OPQ1.Angles()
Angles2 = OPQ2.Angles()
print(OPQ1.Angles())
print(OPQ2.Angles())

va1 = vctr.Subtract(OPQ1.P_Coord() , OPQ1.O_Coord()).unit()
va2 = vctr.Subtract(OPQ2.P_Coord() , OPQ2.O_Coord()).unit()
va1.contents()
va2.contents()


vb1 = vctr.Subtract(OPQ1.Q_Coord() , OPQ1.O_Coord()).unit()
vb2 = vctr.Subtract(OPQ2.Q_Coord() , OPQ2.O_Coord()).unit()
vb1.contents()
vb2.contents()
#   CONVERT COORDINATES TO OPQ
print(rotl.Extreme_Rotater(va1,vb1,va2,vb2))

Euler = rotl.Extreme_Rotater(va1,vb1,va2,vb2)
print('here',N)
deltheta = Euler[1]/N
print('here',N,deltheta)
u1list = [va1]
v1list = [vb1]
u1 = va1
v1 = vb1

for i in range(0,N):

    uv = rotl.RotateInSteps(Euler[0],deltheta,u1,v1)
    u1 = uv[0]
    v1 = uv[1]
    u1list.append(u1)
    v1list.append(v1)


u1list[-1].contents()
v1list[-1].contents()
print(len(u1list))

#Put it all Together

OPQchart = [OPQ1]
for i in range(0,N):
    Oi = CoordinatePath[i+1]
    Pi = vctr.Add(Oi , vctr.ScalarMul(l,u1list[i+1]))
    Qi = vctr.Add(Oi , vctr.ScalarMul(l,v1list[i+1]))
    OPQ_matrix = np.zeros((3,3))
    OPQ_matrix[0,:] = Oi.array()
    OPQ_matrix[1,:] = Pi.array()
    OPQ_matrix[2,:] = Qi.array()
    OPQi = ral.OPQ(OPQ_matrix)
    OPQchart.append(OPQi)

OPQ2.OPQ_Coordinate().contents()
OPQchart[-1].OPQ_Coordinate().contents()
CoordinatePath[-1].contents()
print(Coord_Matrix)


OPQ_array = np.zeros((1,9))
for i in OPQchart:
    print('Yolanda',type(i))
    print(OPQ_array.shape)
    OPQ_array = np.vstack((OPQ_array,i.OPQ_Flat_Arr()))

OPQ_array = np.delete(OPQ_array,0,0)
print(OPQ_array)
Pos_Array = OPQ_array

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
fig.update_layout(
    title="OPQ Path Planner - Linear Interpolation",
    font=dict(
        family="Courier New, monospace",
        size=18,
        color="#7f7f7f"
    )
)

fig.layout.update(showlegend=False)
fig.show()
fig.write_html("OPQ_Path_Planning.html")

np.savetxt(".Data/DataOut/OPQ Path.csv",OPQ_array,delimiter = ',')
