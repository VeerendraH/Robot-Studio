import pandas as pd
import numpy as np
import Robot_Arm_Library as ral
import Vector as vctr

import plotly
import plotly.offline as py
import plotly.graph_objs as go
import Quaternion as qtrn

#   EXTRACT ANGLE OF ROTATION

def Delta_Theta(v1,v2,n):
    if np.abs(Parallel(v1,n)) + np.abs(Parallel(v2,n)) == 0:
        vproj1 = vctr.Subtract(v1,vctr.ScalarMul((vctr.Dot(v1,n)),n))
        vproj2 = vctr.Subtract(v2,vctr.ScalarMul((vctr.Dot(v2,n)),n))

        deltheta = vctr.Angle(vproj1,vproj2)
    else:
        deltheta = 0
    return deltheta

#   CHECK PARALLELISM OF TWO VECTORS

def Parallel(v1,v2):
    if (vctr.Subtract(v1.unit(),v2.unit())).mag() <0.0001:
        Val = 1
    elif (vctr.Add(v1.unit(),v2.unit())).mag() < 0.0001:
        Val = -1
    else:
        Val = 0

    return Val

#   SUFFICIENCY CONDITION TO OBTAIN UNIQUE AXIS
def Sufficiency(va1,vb1,va2,vb2):

    na = Generate_Normal_For_Locus_Plane(va1,va2)[0]
    nb = Generate_Normal_For_Locus_Plane(vb1,vb2)[0]

    return Parallel(na,nb)

#   GENERATE NORMAL FOR LOCUS PLANE
def Generate_Normal_For_Locus_Plane(v1,v2):
    prop = 0
    if Parallel(v1,v2) == -1:
        Normal = v1
    elif Parallel(v1,v2) == 0:
        Ang_bis = vctr.Add(v1.unit(),v2.unit())
        Ang_bis = Ang_bis.unit()
        para = vctr.ScalarMul(vctr.Dot(v1,Ang_bis),Ang_bis)
        Perp = vctr.Subtract(v1,para)
        Normal = Perp

    elif Parallel(v1,v2) == 1:
        Normal = v1
        prop = 1

    return Normal,prop

#   GENERATE AXIS FROM LOCII NORMALS
def AxisFromNormals(na,nb):
    no = vctr.Cross(na,nb)
    n = no.unit()
    #nmin = vctr.Subtract(vctr.Vector(0,0,0),n)

    return n

#   OBSERVE PARALLELISM AMONGST ALL VECTORS
def Parallel_Matrix(u1,v1,w1,u2,v2,w2):
    Mat = [[0,1,2],[0,0,0]]
    Mat[1][0] = Parallel(u1,u2)
    Mat[1][1] = Parallel(v1,v2)
    Mat[1][2] = Parallel(w1,w2)

    return np.array(Mat)

#   OBSERVE SUFFICIENCY AMONGST ALL VECTORS
def Sufficiency_Matrix(u1,v1,w1,u2,v2,w2):

    Mat = [0,0,0]
    Mat[0] = Sufficiency(u1,v1,u2,v2)
    Mat[1] = Sufficiency(w1,v1,w2,v2)
    Mat[2] = Sufficiency(u1,w1,u2,w2)

    return Mat

#   OBTAIN OPTIMUM ANGLE OF ROTATION REQUIRED
def Optimum(ThetaA,ThetaB):

    if np.abs(ThetaA - ThetaB) < 0.001:
        Theta = 0.5*ThetaA + 0.5*ThetaB
    else:
        Theta = 0

    return Theta

#   FLOWCHART BASED DESIGN OF OBTAINING UNIQUE AXIS AND ANGLE

#START

#CHECK LOCII OF A AND B
def Extreme_Rotater(u1,v1,u2,v2):
    va = vctr.Vector(0,0,0)
    na = vctr.Vector(0,0,0)
    vb = vctr.Vector(0,0,0)
    nb = vctr.Vector(0,0,0)
    Axis = vctr.Vector(0,0,0)

    #   A
    LocusA = Generate_Normal_For_Locus_Plane(u1,u2)
    if LocusA[1] == 1:
        va = u1
        casea = 1
    else:
        na = LocusA[0]
        casea = 0

    #   B
    LocusB = Generate_Normal_For_Locus_Plane(v1,v2)
    if LocusB[1] == 1:
        vb = v1
        caseb = 1
    else:
        nb = LocusB[0]
        caseb = 0

        #CATEGORISE
    Case = casea + caseb


    #PROCEED TO INTERSECTION OF LOCII
        #BOTH LOCII ARE UNIQUE VECTORS
    if Case == 2:
        Axis = vctr.Vector(0,0,1)
        Theta = 0
        #ONE OF THEM HAS VECTOR LOCUS
    elif Case == 1:
        v = vctr.Add(va,vb)
        n = vctr.Add(va,vb)
            #
        if vctr.Dot(v,n) == 0:
            Axis = v
        else:
            Axis = vctr.Vector(0,0,0)
        #BOTH LOCII ARE PLANES
    elif Case == 0:
        if np.abs(Parallel(na,nb)) == 1:
            w1 = vctr.Cross(u1.unit(),v1.unit()).unit()
            w2 = vctr.Cross(u2.unit(),v2.unit()).unit()
            SM = Sufficiency_Matrix(u1,v1,w1,u2,v2,w2)
            if SM[1] == 0:
                Axis = Extreme_Rotater(v1,w1,v2,w2)[0]
            elif SM[2] == 0:
                Axis = Extreme_Rotater(u1,w1,u2,w2)[0]
            else:
                Axis = vctr.Vector(0,0,0)
        else:
            Axis = AxisFromNormals(na,nb)






    #OUTLIER REPROACH

    #PROCEED TO GENERATE THETA
    if Axis == vctr.Vector(0,0,0):
        Theta = 0

    else:
        ThetaA = Delta_Theta(u1,u2,Axis)
        ThetaB = Delta_Theta(v1,v2,Axis)

        Theta = Optimum(ThetaA,ThetaB)

        return Axis,Theta




def RotateInSteps(Axis,delTheta,u1,v1):
    u2 = qtrn.RotOper(delTheta,Axis,u1).purify()
    v2 = qtrn.RotOper(delTheta,Axis,v1).purify()

    return u2,v2
