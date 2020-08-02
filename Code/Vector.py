import pandas as pd
import numpy as np

import matplotlib

import matplotlib.pyplot as plt


import cufflinks as cf

import plotly
import plotly.offline as py
import plotly.graph_objs as go

## VECTOR MODULE

class Vector:
    def __init__(self, x , y , z):
        self.i = x
        self.j = y
        self.k = z

    def __str__(self):
        return "x = {}  ;    y = {} ;   z = {}".format(self.i,self.j,self.k)

    def __add__(self,other):

        return Vector(self.i + other.i , self.j + other.j , self.k + other.k)

    def __radd__(self, other):

        return self.__add__(other)

    def __sub__(self,other):

        return Vector(self.i - other.i , self.j - other.j , self.k - other.k)

    def __mul__(self,other):

        return Vector(self.i * other , self.j * other , self.k * other)

    def __rmul__(self,other):

        return self.__mul__(other)

    def __matmul__(self,other):

        return self.i * other.i + self.j * other.j + self.k * other.k

    def __neg__(self):

        return(Vector(-self.i,-self.j,-self.k))



    def contents(self):
        print("x = ",self.i , ";    y = ",self.j , ";   z = ",self.k )

    def array(self):
        array = np.array([self.i , self.j , self.k])
        return array

    def unit(self):
        d = np.sqrt((self.i)**2 + (self.j)**2 + (self.k)**2)
        if d > 0:
            uniter = Vector(self.i/d,self.j/d,self.k/d)
        else:
            uniter = Vector(self.i,self.j,self.k)

        return uniter

    def mag(self):
        d = np.sqrt((self.i)**2 + (self.j)**2 + (self.k)**2)

        return d

## FUNCTIONS

# CONVERT NP ARRAY TO VECTOR
def array2vec(A):

    A = np.array(A)
    Avec = Vector(A[0],A[1],A[2])

    return Avec

# FIND EUCLIDIAN DISTANCE BETWEEN TWO (POSITION) VECTORS

def Distance(A , B):
    d = np.sqrt((A.i-B.i)**2 + (A.j-B.j)**2 + (A.k-B.k)**2)

    return d

# DOT PRODUCT OF TWO VECTORS

def Dot(A,B):
    d = A.i*B.i + A.j*B.j + A.k*B.k

    return d

# CROSS PRODUCT OF TWO QUARTERNIONS

def Cross(A,B):
    a = A.array()
    b = B.array()

    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    C = array2vec(c)

    return C

# RETURN ANGLE BETWEEN TWO VECTORS

def Angle(A,B):
    costheta = Dot(A.unit(),B.unit())
    Theta = np.arccos(costheta)
    Theta = np.rad2deg(Theta)

    return Theta

def AnglewrtAxis(Ax,v0,v):
    AngleMag = Angle(v0,v)
    cosAng = v0.unit()@v.unit()
    #print(v0,v,cosAng,'cosine')
    if cosAng**2 < 0.9999:
        AngleSign = np.sign(Cross(v0,v).unit()@Ax)
        Anglet = AngleMag*AngleSign
    elif cosAng > 0:
        Anglet = 0
    elif cosAng < 0:
        Anglet = 180

    return Anglet

# CONVERT VECTOR TO UNIT VECTOR

def Unit(A):
    d = Dot(A,A)
    A = array2vec(((A.array)/d))

    return A

# ADD TWO VECTORS

def Add(A,B):
    C = Vector(0,0,0)
    C.i = A.i + B.i
    C.j = A.j + B.j
    C.k = A.k + B.k

    return C

# SUBTRACT TWO VECTORS

def Subtract(A,B):
    C = Vector(0,0,0)
    C.i = A.i - B.i
    C.j = A.j - B.j
    C.k = A.k - B.k

    return C

# SCALAR MULTIPLICATION

def ScalarMul(A,B):
    C = Vector(0,0,0)
    C.i = A*B.i
    C.j = A*B.j
    C.k = A*B.k

    return C

# PROJECT POSITION VECTOR ON A PLANE DEFINED BY NORMAL AND CONSTITUENT POINT

def Project(DC,A,B):
    DC = DC.unit()

    C1 = B
    C2mul = Dot(A,DC) - Dot(B,DC)

    C2 = ScalarMul(C2mul,DC)
    print(type(C2),type(C1))

    Cone = Add(C1,C2)
    Ctwo = Subtract(C1,C2)

    coned = Distance(Cone,A)
    ctwod = Distance(Ctwo,A)

    if coned < ctwod:
        C = Cone
    elif coned > ctwod:
        C = Ctwo
    elif coned == ctwod:
        C = Cone

    return C

# TRANSFORM A VECTOR BY THETA ALONG AXIS

def TransformVector(Theta, Vector, AxisVector):
    Vector = Vector.array()
    Axis = AxisVector.array()

    Theta = np.radians(Theta)
    Vectormag = np.linalg.norm(Vector)
    Vector = Vector / Vectormag
    Axis = Axis / np.linalg.norm(Axis)

    l = Axis[0]
    m = Axis[1]
    n = Axis[2]

    CrossMat = np.array([[0 , - n , m ] , [n,0 , -l] , [-m , l , 0]])

    Matrix1 = np.cos(Theta)*np.eye(3) + np.sin(Theta)*CrossMat
    Matrix2 = (1 - np.cos(Theta))*np.outer(Axis,Axis)
    Matrix = Matrix1 + Matrix2

    VectorOut = np.around(Vectormag*np.matmul(Matrix,np.array(Vector)), decimals = 8)

    VectorOut = array2vec(VectorOut)

    return VectorOut
