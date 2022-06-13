import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


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

def fkine_ur5(q):
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
    T2 = dh(0        ,q[1]+pi  ,d1 ,   0)
    T3 = dh(0        ,q[2]+pi/2,-h2,pi/2)
    T4 = dh(0        ,q[3]     ,0  ,   0)
    T5 = dh(q[4] + d2,pi       ,0  ,pi/2)
    T6 = dh(0        ,q[4]+pi  ,0  ,pi/2)
    T7 = dh(d3       ,q[5]+pi  ,0  ,   0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T
