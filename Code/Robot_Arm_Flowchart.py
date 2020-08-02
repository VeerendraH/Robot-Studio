#   IMPORT LIBRARIES AND MODULES

import pandas as pd
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

import Vector as vctr
import Quaternion as qtrn
import Robot_Arm_Library as ral

import plotly
import plotly.offline as py
import plotly.graph_objs as go

#   ROBOT CONFIGURATION INPUT

# Take input from files
Links_filename = 'Robot Arm Input File.csv'
OPQ_filename = 'OPQ Data Input.csv'
Robot_Hand = pd.read_csv(OPQ_filename)
Robot_Arm = pd.read_csv(Links_filename)
print(Robot_Arm)

# Load onto the arrays
Position_Arm = Robot_Arm[['X','Y','Z']]
Position_array = Position_Arm.to_numpy()

Axes_Arm = Robot_Arm[['ax_l','ax_m','ax_n']]
Axes_array = Axes_Arm.to_numpy()

OPQ_Arm = Robot_Hand[['X','Y','Z']]
OPQ_array = OPQ_Arm.to_numpy()

Ref_Arm = Robot_Arm[['ref_l','ref_m','ref_n']]
Ref_array = Ref_Arm.to_numpy()

Act_Arm = Robot_Arm[['act_l','act_m','act_n']]
Act_array = Act_Arm.to_numpy()


#   PREPARE "Robot_Initiator" AND "Actuator_Matrix"

Data_Trial = ral.RobotArm(6)

# Robot_initiator
Data_RO = ral.Robot_Initiator(6)
Data_RO.Array_Input(Position_array,Axes_array,OPQ_array,Ref_array,Act_array)
Data_Trial.Array_Input(Data_RO)

#Data_Trial.Visualise()
print(Data_Trial.Motor_Angle())
Data_Trial.True_Actuate([90,0,0,0,0,0])
#Data_Trial.Visualise()
print(Data_Trial.Motor_Angle())


# Actuator_Matrix
Act = ral.Actuator_Matrix()
Act_Mat = pd.read_csv('Actuator.csv')
Act_Matric = Act_Mat.to_numpy()

Act.Initialise(Act_Matric)

#   ROBOT ACTUATION

def Run_Act(Act_Matrix,Robot_Init):
    NominalRobotArmChart = ral.Robot_Arm_Chart(Act_Matrix,Robot_Init)
    NominalRobotArmChart.Initialise()
    NominalRobotArmChart.Run_Samples()
    Output_Arr = NominalRobotArmChart.Array_Output()
    fig = NominalRobotArmChart.Visualise()
    return Output_Arr,fig

Product = Run_Act(Act,Data_RO)
Output_Array = Product[0]
Figure = Product[1]
Figure.layout.update(showlegend=False)
Figure.show()
Figure.write_html("file.html")

print(Output_Array)
'''

np.savetxt('Out.csv',Output_Array,header = 'Ox,Oy,Oz,Px,Py,Pz,Qx,Qy,Qz',comments='',delimiter = ',')
'''
