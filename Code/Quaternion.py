
import numpy as np
import pandas as pd

import Vector as vctr

## Quaternion MODULE

class Quaternion:
    def __init__(self, w,i,j,k):
        self.w = w
        self.i = i
        self.j = j
        self.k = k

    def contents(self):
        print("w = ",self.w ,";    x = ",self.i , ";    y = ",self.j , ";   z = ",self.k )

    def array(self):
        array = np.array([self.w , self.i , self.j , self.k])
        return array

    def unit(self):
        d = np.sqrt((self.w)**2 + (self.i)**2 + (self.j)**2 + (self.k)**2)
        uniter = Quaternion(self.i/d,self.i/d,self.j/d,self.k/d)

        return uniter

    def mag(self):
        d = np.sqrt((self.w)**2 + (self.i)**2 + (self.j)**2 + (self.k)**2)

        return d

    def purify(self):
        Vector = vctr.array2vec([self.i,self.j,self.k])
        return Vector

## FUNCTIONS

#   CONVERT NP ARRAY TO Quaternion
def array2quar(A):

    A = np.array(A)
    Aquar = Quaternion(A[0],A[1],A[2],A[3])

    return Aquar

#   CONVERT QUATERNION TO VECTOR


#   ROUND A QUATERNION
def roundquar(A):

    Anew = array2quar(np.round(A.array(),8))

    return Anew

#   SCALAR MULTIPLICATION

def ScalarMul(A,B):
    B.w = A*B.w
    B.i = A*B.i
    B.j = A*B.j
    B.k = A*B.k

    return B

#   ADDITION OF TWO QUATERNIONS

def Add(A,B):
    C = Quaternion(0,0,0,0)
    C.w = A.w + B.w
    C.i = A.i + B.i
    C.j = A.j + B.j
    C.k = A.k + B.k

    return C

#   SUBTRACTION OF TWO QUATERNIONS

def Subtract(A,B):
    C = Quaternion(0,0,0,0)
    C.w = A.w - B.w
    C.i = A.i - B.i
    C.j = A.j - B.j
    C.k = A.k - B.k

    return C

#   PRODUCT OF TWO QUATERNIONS
def Product(p,q):
    p0 = p.array()[0]
    q0 = q.array()[0]

    pbar = vctr.array2vec((p.array()[1:4]))
    qbar = vctr.array2vec((q.array()[1:4]))

    cros = vctr.Cross(pbar,qbar)
    #Scalar Part
    pq0 = p0*q0 - vctr.Dot(pbar,qbar)

    #Vector Part
    pqbar1 = vctr.Add(vctr.ScalarMul(q0,pbar) , vctr.ScalarMul(p0,qbar))
    pqbar = vctr.Add(pqbar1,cros)


    pq = array2quar(np.append(pq0 , pqbar.array()))
    return pq

#   COMPLEMENT OF QUATERNION
def Complement(A):
    A0 = A.array()[0]
    Abar = vctr.array2vec((-A.array()[1:4]))

    Acomp = array2quar(np.append(A0 , Abar.array()))
    return Acomp

#   MULTIPLICATIVE INVERSE
def Inverse(A):
    Ainv = ScalarMul((1/(A.mag())**2),Complement(A))

    return Ainv

#   VECTOR TO PURE QUATERNIONS
def Vctr2Qtrn(A):
    Aq = array2quar(np.append(0,A.array()))

    return Aq

#   ANGLE BETWEEN PURE QUATERNIONS
def Angle(ubar,vbar):
    u = Vctr2Qtrn(ubar)
    v = Vctr2Qtrn(vbar)

    w = Product(u.unit(),v.unit())
    cosTheta = w.w
    Theta = np.arccos(cosTheta)
    return Theta

#   DISTANCE BETWEEN PURE QUATERNIONS
def Distance(Ribar,Rjbar):
    Ri = Vctr2Qtrn(Ribar)
    Rj = Vctr2Qtrn(Rjbar)

    DelR = Subtract(Ri,Rj)
    Dist = DelR.mag()

    return Dist

#   ROTATION OPERATOR
def RotOper(Theta,u,v):
    u_unit = u.unit()
    c = np.cos(np.deg2rad(Theta/2))
    s = np.sin(np.deg2rad(Theta/2))

    q = array2quar(np.append(c , vctr.ScalarMul(s,u_unit).array()))
    v = array2quar(np.append(0,v.array()))

    Lv = Product(Product(q,v),Complement(q))

    Lvrounded = roundquar(Lv)

    return Lvrounded
#   ANGLE CHANGE BETWEEN TWO VECTORS WRT AN AXIS
def Delta_Theta(v1,v2,n):

    vproj1 = vctr.Subtract(v1,vctr.ScalarMul((vctr.Dot(v1,n)),n))
    vproj2 = vctr.Subtract(v2,vctr.ScalarMul((vctr.Dot(v2,n)),n))

    deltheta = vctr.Angle(vproj1,vproj2)
    sign = np.sign(vctr.Dot(n,vctr.Cross(va1,va2)))

    nopt = vctr.ScalarMul(sign,n)
    

    return sign*deltheta

#   1D ROTATION BETWEEN TWO ANGULAR COORDINATES

def OneD_Rotate(va1,vb1,va2,vb2):

    na = Plane_Normal(va1,va2)
    nb = Plane_Normal(vb1,vb2)

    no = vctr.Cross(na,nb)
    n = no.unit()
    nmin = vctr.Subtract(vctr.Vector(0,0,0),n)

    Delta_Theta_A = Delta_Theta(va1,va2,n)
    Delta_Theta_B = Delta_Theta(vb1,vb2,n)

    return n,
