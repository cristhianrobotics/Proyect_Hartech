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
  sth = sin(theta)
  cth = cos(theta)
  sa  = sin(alpha)
  ca  = cos(alpha)
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
  T1 = dh(0.273, q[0] + np.pi,   0.0,  np.pi/2)
  T2 = dh(0.0,   q[1] + np.pi,   0.160, 0.0)
  T3 = dh(0.0,   q[2] + np.pi/2, 0.0,  np.pi/2)
  T4 = dh(0.175, q[3] + np.pi,   0.0,  np.pi/2)
  T5 = dh(0.0,   q[4] + np.pi,   0.0,  np.pi/2)
  T6 = dh(0.113, q[5],           0.0,  0.0)

  # Efector final respecto a la base
  T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
  return T


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