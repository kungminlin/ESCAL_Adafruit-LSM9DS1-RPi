import numpy as np

dt = 0.02												# Change in Time (sec)
A = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],				# State Transition Matrix
			  [0, 1, 0, 0, 0, 0, 0, 0, 0],
			  [0, 0, 1, 0, 0, 0, 0, 0, 0],
			  [dt, 0, 0, 1, 0, 0, 0, 0, 0],
			  [0, dt, 0, 0, 1, 0, 0, 0, 0],
			  [0, 0, dt, 0, 0, 1, 0, 0, 0],
			  [0.5*dt*dt, 0, 0, dt, 0, 0, 1, 0, 0],
			  [0, 0.5*dt*dt, 0, 0, dt, 0, 0, 1, 0],
			  [0, 0, 0.5*dt*dt, 0, 0, dt, 0, 0, 1]])

Q = np.array([[0.01, 0, 0, 0, 0, 0, 0, 0, 0],			# Process Noise Co-Variance Matrix
			 [0, 0.01, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0.01, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0.01, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0.01, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0.01, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0.01, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0.01, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0.01]])

x = np.array([pos_x, pos_y, pos_z, 						# State Model
			  vel_x, vel_y, vel_z, 
			  accel_x, accel_y, accel_z])

P = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],				# Co-Variance Matrix
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0],
			 [0, 0, 0, 0, 0, 0, 0, 0, 0]])

H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],				# Observation Matrix
			  [0, 0, 0, 0, 0, 0, 0, 1, 0],
			  [0, 0, 0, 0, 0, 0, 0, 0, 1]])

R = np.array([[5, 0, 0],								# Measurement Noise Covariance Matrix
			  [0, 5, 0],
			  [0, 0, 5]])

def update(new_state):
	global x
	global P
	x = A.dot(x)
	P = A.dot(P).dot(A.T) + Q
	y = new_state - H.dot(x) 							# Innovation Factor
	S = H.dot(P).dot(H.T) + R 							# Innovation Co-Variance
	K = P.dot(H.T).dot(np.linalg.pinv(S))				# Kalman Gain
	x = x + K.dot(y)
	P = (np.identity(9) - (K.dot(H))).dot(P)

def get_state():
	return x
