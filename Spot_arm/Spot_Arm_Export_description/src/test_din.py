import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/Spot_Arm_Export.xacro')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6 , 0])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0, 0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad
m_temp = np.zeros(ndof)

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Calcula vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
rbdl.InverseDynamics(modelo, q, dq, zeros, c) 
c = c - g

for i in range(ndof):
   rbdl.InverseDynamics(modelo, q, zeros, e[:,i], m_temp)
   M[:,i] = m_temp - g


# Verificacion de expresion dinamica 
tau_sim = M.dot(ddq) + c + g

if( np.linalg.norm(tau_sim-tau) < 1e-5):
    print("Parametros dinamicos coinciden con simulación")
else:
    print("Parametros dinamicos NO coinciden con simulación")


