import numpy as np
from scipy import linalg


def ellipsoid_fit(s):
  ''' Estimate ellipsoid parameters from a set of points.

      Parameters
      ----------
      s : array_like => np.array([mx, my, mz])
        The samples (M,N) where M=3 (x,y,z) and N=number of samples.

      Returns
      -------
      M, n, d : array_like, array_like, float
        The ellipsoid parameters M, n, d.

      References
      ----------
      .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
          fitting," in Geometric Modeling and Processing, 2004.
          Proceedings, vol., no., pp.335-340, 2004
  '''

  # D (samples)
  D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

  # S, S_11, S_12, S_21, S_22 (eq. 11)
  S = np.dot(D, D.T)
  S_11 = S[:6,:6]
  S_12 = S[:6,6:]
  S_21 = S[6:,:6]
  S_22 = S[6:,6:]

  # C (Eq. 8, k=4)
  C = np.array([[-1,  1,  1,  0,  0,  0],
                [ 1, -1,  1,  0,  0,  0],
                [ 1,  1, -1,  0,  0,  0],
                [ 0,  0,  0, -4,  0,  0],
                [ 0,  0,  0,  0, -4,  0],
                [ 0,  0,  0,  0,  0, -4]])

  # v_1 (eq. 15, solution)
  E = np.dot(linalg.inv(C),
              S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

  E_w, E_v = np.linalg.eig(E)

  v_1 = E_v[:, np.argmax(E_w)]
  if v_1[0] < 0: v_1 = -v_1

  # v_2 (eq. 13, solution)
  v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

  # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
  M = np.array([[v_1[0], v_1[5], v_1[4]],
                [v_1[5], v_1[1], v_1[3]],
                [v_1[4], v_1[3], v_1[2]]])
  n = np.array([[v_2[0]],
                [v_2[1]],
                [v_2[2]]])
  d = v_2[3]

  return M, n, d

def calculate_params_calibration_mag(M, n, d):
  # Calcular parámetros de calibración
  M_1 = linalg.inv(M)
  b = -M_1 @ n  # Vector columna 3x1
  
  # Calcular matriz de corrección A_1
  scale_factor = 2803 / np.sqrt(n.T @ M_1 @ n - d)
  A_1 = np.real(scale_factor * linalg.sqrtm(M))
  
  return A_1, b

def estimate_new_magnetometer_with_array_data(A_1, b, mx, my, mz):
  xm_cal, ym_cal, zm_cal = [], [], []
  for x, y, z in zip(mx, my, mz):
      # Restar hard iron offset
      x_off = x - b[0, 0]
      y_off = y - b[1, 0]
      z_off = z - b[2, 0]
      
      # Aplicar corrección soft iron
      x_cal = A_1[0,0] * x_off + A_1[0,1] * y_off + A_1[0,2] * z_off
      y_cal = A_1[1,0] * x_off + A_1[1,1] * y_off + A_1[1,2] * z_off
      z_cal = A_1[2,0] * x_off + A_1[2,1] * y_off + A_1[2,2] * z_off
      
      xm_cal.append(x_cal)
      ym_cal.append(y_cal)
      zm_cal.append(z_cal)
      
  return xm_cal, ym_cal, zm_cal

def estimate_new_magnetometer_value(A_1, b, x, y, z):
  # Restar hard iron offset
  x_off = x - b[0]
  y_off = y - b[1]
  z_off = z - b[2]
  
  # Aplicar corrección soft iron
  x_cal = A_1[0,0] * x_off + A_1[0,1] * y_off + A_1[0,2] * z_off
  y_cal = A_1[1,0] * x_off + A_1[1,1] * y_off + A_1[1,2] * z_off
  z_cal = A_1[2,0] * x_off + A_1[2,1] * y_off + A_1[2,2] * z_off
  
  return x_cal, y_cal, z_cal

# fig = plt.figure()
#   a1 = fig.add_subplot(121, projection='3d')
#   a1.scatter(xm_cal, ym_cal, zm_cal, marker='o', color='g')
#   a1.title.set_text('Calibrated Data')
#   a2 = fig.add_subplot(122, projection='3d')
#   a2.scatter(mx, my, mz, marker='o', color='r')
#   a2.title.set_text('Original Data')
  
#   # Crear figura con subplots
#   fig2 = plt.figure(figsize=(15, 5))
  
#   # Configurar los 3 subplots
#   planes = [
#       ('XY', mx, my, xm_cal, ym_cal),
#       ('XZ', mx, mz, xm_cal, zm_cal),
#       ('YZ', my, mz, ym_cal, zm_cal)
#   ]
  
#   for i, (title, orig_x, orig_y, cal_x, cal_y) in enumerate(planes):
#       ax = fig2.add_subplot(1, 3, i+1)
#       ax.scatter(orig_x, orig_y, c='r', marker='o', alpha=0.3, label='Original')
#       ax.scatter(cal_x, cal_y, c='g', marker='o', alpha=0.3, label='Calibrado')
#       ax.set_title(f'Plano {title}')
#       ax.set_xlabel(f'Eje {title[0]}' if i != 2 else 'Eje Y')
#       ax.set_ylabel(f'Eje {title[1]}')
#       ax.grid(True)
#       ax.legend()
#       ax.axis('equal')  # Mantener relación de aspecto 1:1
  
#   plt.tight_layout()
#   plt.show()