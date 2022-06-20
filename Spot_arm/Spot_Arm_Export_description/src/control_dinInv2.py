#!/usr/bin/env python
import rbdl
import rospy
from sensor_msgs.msg import JointState
from markers import *
from proyfunctions import *
from roslib import packages


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("qactual.dat", "w")
fqdes = open("qdeseado.dat", "w")
fxact = open("xactual.dat", "w")
fxdes = open("xdeseado.dat", "w")

# Nombres de las articulaciones
jnames = ['Rev7', 'Rev8', 'Rev9', 'Rev10', 'Slider11', 'Rev14', 'Rev15']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# #======================Grupo 1================================
# Configuracion articular inicial (en radianes)
# q0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# Velocidad inicial
# dq0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
# qdes = np.array([2.34, 2.33, 2.26, -0.05, 0.07, 0.37, 1.03])
# dqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# ddqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# #=============================================================

# # ======================Grupo 2================================
# # Configuracion articular inicial (en radianes)
# q0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# # Velocidad inicial
# dq0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# # Configuracion articular deseada
# qdes = np.array([0.36, 2.89, 2.40, -1.96, 0.03, -0.04, -2.05])
# dqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# ddqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# # =============================================================

# # ======================Grupo 3================================
# # Configuracion articular inicial (en radianes)
# q0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# # Velocidad inicial
# dq0 = np.array([0., 0., 0., 0., 0., 0., 0.])
# # Configuracion articular deseada
# qdes = np.array([1.85, 2.93, 1.76, -1.55, 0.04, -0.35, 0.33])
# dqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# ddqdes = np.array([0., 0., 0., 0., 0., 0., 0.])
# # =============================================================


# Posicion resultante de la configuracion articular deseada
xdes = fkine_Spot(qdes)[0:3, 3]

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q0
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/Spot.urdf')
ndof = modelo.q_size     # Grados de libertad

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Se definen las ganancias del controlador
valores = 0.5*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Bucle de ejecucion continua
t = 0.0

# Variables articulares
q = copy(q0)
dq = copy(dq0)

# Arrays para dinamica
M = np.zeros([ndof, ndof])
b = np.zeros(ndof)

# Limites
q_max = [3.14159, 3.14159, 2.61799, 1.22173, 0.075, 1.047198, 1.30899]
q_min = [-2.61799, -0.523599, 0.0, -4.3633, 0.0, -1.047198, -4.45059]

while not rospy.is_shutdown():

    # Leer valores del simulado

    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
    rbdl.NonlinearEffects(modelo, q, dq, b)

    x = fkine_Spot(q)[0:3, 3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' ' + str(q[2]) +
                ' ' + str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' ' + str(qdes[2])+' ' + str(
        qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n')

    # Error
    e = qdes - q
    de = dqdes - dq

    # Ley de control
    u = M.dot(ddqdes + Kd.dot(de) + Kp.dot(e)) + b

    # Actualizacion
    ddq = np.linalg.inv(M).dot(u-b)
    qprev = q + dt*dq
    dq = dq + dt*ddq

    # Comprobacion de limites articulares
    for j in range(ndof):
        if(qprev[j] > q_min[j] and qprev[j] < q_max[j]):
            q[j] = qprev[j]

    print(np.round(q, 4))

    # Finalizacion del control
    if(np.linalg.norm(qdes - q) < 1e-4):
        break

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
