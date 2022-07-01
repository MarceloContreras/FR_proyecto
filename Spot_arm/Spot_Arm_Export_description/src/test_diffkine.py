#!/usr/bin/env python
 
from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
 
from markers import *
from proyfunctions import *
from roslib import packages

 
# Inicializacion del nodo
rospy.init_node("testKineControlPosition")
print('starting motion ... ')
# Publicacion del joint_states
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

# Archivos para almacenar datos
fxcurrent = open("/home/user/proy_ws/src/xcurrent.txt", "w")                
fxdesired = open("/home/user/proy_ws/src/xdesired.txt", "w")
fq = open("/home/user/proy_ws/src/q.txt", "w")
fqd = open("/home/user/proy_ws/src/qd.txt", "w")

# Markers para las 
bmarker_current  = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])
 
# Joint names
jnames = ['Rev7', 'Rev8', 'Rev9','Rev10','Slider11', 'Rev14', 'Rev15']

# Posicion deseada

# Caso 1
qd = np.array([2.18, 2.86, 1.93, -2.77, 0.04, 0.8, 0.38])
k = 12

# Caso 2
#qd = np.array([0.36, 2.89, 2.40, -1.96, 0.03, -0.04, -2.05])
#k = 5

# Caso 3
#qd = np.array([1.85, 2.93, 1.76, -1.55, 0.04, -0.35, 0.33])
#k = 10

# Se define la posicion deseada en base a qd usando la cinematica 
# directa dependiendo del caso

T = fkine_Spot(qd)
xd = T[0:3,3]

# Initial configuration
q0  = np.array([0, 0, 0, 0, 0.000001, 0, 0])

# Resulting initial position (end effector with respect to the base link)
T = fkine_Spot(q0)
x0 = T[0:3,3]
 
epsilon = 0.0001
count = 0

u_lim = np.array([3.14159,   3.14159,  2.61799, 1.22173, 0.075, 1.047198,  1.30899])
l_lim = np.array([-2.61799, -0.523599, 0,      -4.3633,  0,    -1.047198, -4.45059])

# Red marker shows the achieved position
bmarker_current.xyz(x0)
# Green marker shows the desired position
bmarker_desired.xyz(xd)
 
# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0
 
# Frequency (in Hz) and control period 
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)
t = 0
 
# Initial joint configuration
q = copy(q0)
# Main loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for position (complete here)
    # -----------------------------
 
    # Calculo del jacobiano y posicion actual
    J = jacobian_Spot(q, 0.0001)
    x = fkine_Spot(q)
    x = x[0:3, 3]
    
    # Calculo del error
    e = x - xd
 
    # Condicion para detener el algoritmo
    if(np.linalg.norm(e) < epsilon):
        print('Desired point reached')
        print(e)
        print(x)
        print('iteraciones', count)
        break
 
    # Ley de control
    de = -k*e

    # Comprobacion del rango del Jacobiano
    rank_J = np.linalg.matrix_rank(J)

    # Si el rango es menor a 3 se usa la pseudo-inversa
    # amortiguada
    if (rank_J < 3):
        dq = J.T.dot(np.linalg.inv(J.dot(J.T)+0.01*np.eye((3)))).dot(de)
        print(rank_J)
        print(J)
    else:
        dq = np.linalg.pinv(J).dot(de)

    # Se actualiza el q potencialmente
    q_pot = q + dt*dq

    # Si una articulacion sale de sus limites no se actualiza
    for i in range(7):
        if ( (q_pot[i] < u_lim[i]) and (q_pot[i] > l_lim[i]) ):
            q[i] = q_pot[i]

    # Se detiene el algoritmo despues de 10000 iteraciones si 
    # no se llego al punto deseado
    count = count + 1 
    if(count > 10000):
        print('Max number of iterations reached')
        break
 
    # -----------------------------

    t = t + dt 
    
    # Log values                                                      
    fxcurrent.write(str(t)+" "+str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
    fxdesired.write(str(t)+" "+str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.write(str(t)+" "+str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
             str(q[4])+" "+str(q[5])+" "+str(q[6])+"\n")
    fqd.write(str(t)+" "+str(qd[0])+" "+str(qd[1])+" "+str(qd[2])+" "+str(qd[3])+" "+
             str(qd[4])+" "+str(qd[5])+" "+str(qd[6])+"\n")
    
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    # Wait for the next iteration
    rate.sleep()
 
print('ending motion ...')
fxcurrent.close()
fxdesired.close()
fq.close()
fqd.close()
