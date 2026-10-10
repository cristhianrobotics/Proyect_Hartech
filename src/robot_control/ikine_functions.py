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
    

def fkine_hartech(q):
  """
  Calcular la cinematica directa del robot HARTECH dados sus valores articulares.
  q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
  """
  # Longitudes (en metros)
  # Matrices DH según la tabla del robot HARTECH
  T1 = dh(0.273, q[0] + np.pi,   0.0,   np.pi/2)
  T2 = dh(0.0,   q[1] + np.pi,   0.160, 0.0)
  T3 = dh(0.0,   q[2] + np.pi/2, 0.0,   np.pi/2)
  T4 = dh(0.175, q[3] + np.pi,   0.0,   np.pi/2)
  T5 = dh(0.0,   q[4] + np.pi,   0.0,   np.pi/2)
  T6 = dh(0.113, q[5],           0.0,   0.0)

  # Efector final respecto a la base
  T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
  return T


def jacobian_position(q, delta=0.001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad.
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular
 q=[q1, q2, q3, ..., qn],
 
 lo que se implementa realmente es una aproximacion numerica del Jacobiano de posicion usando diferencias finitas.
 """
 # Determinar la cantidad de articulaciones
 n = len(q)
 # Crear una matriz 3xn
 J = np.zeros((3, n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine_hartech(q)
 x = T[0:3, 3]          # posición actual del efector
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
      dq = np.copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta,
      dq[i] = dq[i] + delta
  # usar la copia de configuración inicial
  # Transformacion homogenea luego del incremento (q+delta)
      T_delta = fkine_hartech(dq)
      dx = T_delta[0:3, 3]

      columna_i = 1/delta * (dx - x)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
      J[:,i] = columna_i
 return J



def ikine(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo de newton.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001
 ee = []
 q  = copy(q0)
 for i in range(max_iter):
   # Jacobiano de posición en q
   J = jacobian_position(q, delta)   # (3 x n)
   
   # Posición actual del efector
   T = fkine_hartech(q)
   f = T[0:3, 3]                     # (3,n)

   # Error de posición (x_des - x_act)
   e = xdes - f                      # (3,)
   enorm = np.linalg.norm(e)
   ee.append(enorm)

   # Condición de término
   if enorm < epsilon:
    print("Cinemática inversa: solución obtenida en iteración", i)
    break
   
   # PASO DE NEWTON: Δq = J^+ e
   dq = np.linalg.pinv(J) @ e 
   # Actualizar q
   q = q + dq

   if i == max_iter - 1:
     print("No se llegó a la solución deseada (revisa q0 o el modelo)")
    
 return q, ee


def ik_gradient(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo gradiente.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001
 alpha    = 0.1   # ganancia del metodo gradiente

 ee = []
 q = copy(q0)

 for i in range(max_iter):
  # Jacobiano de posicion en q
  J = jacobian_position(q, delta)   # (3 x n)

  # Posicion actual del efector
  T = fkine_hartech(q)
  f = T[0:3, 3]                     # (3,)

  # Error de posicion
  e = xdes - f
  enorm = np.linalg.norm(e)
  ee.append(enorm)

  # Condicion de termino
  if enorm < epsilon:
   print("Cinemática inversa por gradiente: solución obtenida en iteración", i)
   break

  # Metodo gradiente: Δq = alpha * J^T * e
  dq = alpha * J.T @ e

  # Actualizar q
  q = q + dq

  if i == max_iter - 1:
   print("No se llegó a la solución deseada (revisa q0 o el modelo)")

 return q, ee




def rot2quat(R):
  """
  Convertir una matriz de rotacion en un cuaternion

  Entrada:
   R -- Matriz de rotacion
  Salida:
   Q -- Cuaternion [ew, ex, ey, ez]

  """
  dEpsilon = 1e-6
  quat = 4*[0.,]

  quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
  if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
    quat[1] = 0.0
  else:
    quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
  if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
    quat[2] = 0.0
  else:
    quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
  if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
    quat[3] = 0.0
  else:
    quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

  return np.array(quat)


def TF2xyzquat(T):
  """
  Convert a homogeneous transformation matrix into the a vector containing the
  pose of the robot.
  
  Input:
   T -- A homogeneous transformation
  Output:
   X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
        is Cartesian coordinates and the last part is a quaternion
  """
  quat = rot2quat(T[0:3,0:3])
  res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
  return np.array(res)