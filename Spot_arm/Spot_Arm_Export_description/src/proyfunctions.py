import numpy as np
import rbdl
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi

## Robot

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    
    return T

## Cinematica directa  

def fkine_Spot(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)
    h1 = 0.140137
    h2 = 0.09734 
    d1 = 0.3385
    d2 = 0.3833
    d3 = 0.275027
    
    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(h1       ,q[0]+pi  ,0  ,pi/2)
    T2 = dh(0        ,-q[1]+pi  ,d1 ,   0)
    T3 = dh(0        ,q[2]+pi/2,-h2,pi/2)
    T4 = dh(0        ,q[3]     ,0  ,   0)
    T5 = dh(q[4] + d2,pi       ,0  ,pi/2)
    T6 = dh(0        ,q[5]+pi  ,0  ,pi/2)
    T7 = dh(d3       ,q[6]+pi  ,-0.05025  ,   0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T

## Dinamica

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.g = np.zeros(ndof)
        self.zero = np.zeros(ndof)
        self.robot = rbdl.loadModel('../urdf/Spot_Arm_Export.xacro')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

    def read_gravity(self):
        rbdl.InverseDynamics(self.robot, self.q, self.zero, self.zero, self.g)
        return self.g
