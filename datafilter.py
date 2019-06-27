import abc
import numpy as np
import math

class Filter(abc.ABC):
	@abc.abstractmethod
	def update(self):
		pass

	@abc.abstractmethod
	def get_state(self):
		pass

class SensorFusion(Filter):
	def __init__(self, sensor, dt=0.01):
		self.sensor = sensor
		self.dt = dt
		self.ang_vel_x, self.roll = 0.0, 0.0
		self.ang_vel_y, self.pitch = 0.0, 0.0

		self.A = np.array([[1, -dt],
							 [0, 1]])

		self.Q = np.identity(2)*0.01

		self.x_roll = np.array([self.ang_vel_x, self.roll])
		self.x_pitch = np.array([self.ang_vel_y, self.pitch])

		self.P_roll = np.zeros((2, 2))
		self.P_pitch = np.zeros((2, 2))

		self.H = np.array([[1], [0]])

		self.R = np.array([5])

	def update(self):
		accel_x, accel_y, accel_z = self.sensor.acceleration
		gyro_x, gyro_y, gyro_z = self.sensor.gyro
		new_state_roll = np.array([gyro_x, math.atan2(acceL_y/(accel_y**2 + accel_z**2))])
		new_state_pitch = np.array([gyro_y, math.atan2(accel_x/(accel_x**2 + accel_z**2))])
		self.x_roll = self.A.dot(self.x_roll)
		self.x_pitch = self.A.dot(self.x_pitch)
		self.P_roll = self.A.dot(self.P_roll).dot(self.A.T) + self.Q
		self.P_pitch = self.A.dot(self.P_pitch).dot(self.A.T) + self.Q
		y_roll = new_state_roll - self.H.dot(self.x_roll)
		y_pitch = new_state_pitch - self.H.dot(self.x_pitch)
		S_roll = self.H.dot(self.P_roll).dot(self.H.T) + self.R
		S_pitch = self.H.dot(self.P_pitch).dot(self.H.T) + self.R
		K_roll = self.P_roll.dot(self.H.T).dot(np.linalg.pinv(S_roll))
		K_pitch = self.P_pitch.dot(self.H.T).dot(np.linalg.pinv(S_pitch))
		self.x_roll = self.x_roll + K_roll.dot(y_roll)
		self.x_pitch = self.x_pitch + K_pitch.dot(y_pitch)
		self.P_roll = (np.identity(2) - (K_roll.dot(self.H))).dot(self.P_roll)
		self.P_pitch = (np.identity(2) - (K_pitch.dot(self.H))).dot(self.P_pitch)

	def get_state(self):
		return self.x_roll[1], self.x_pitch[1]

class KalmanFilter(Filter):
	def __init__(self, sensor, dt=0.01):
		self.sensor = sensor
		self.dt = dt
		self.pos_x, self.pos_y, self.pos_z = 0.0, 0.0, 0.0
		self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
		self.accel_x, self.accel_y, self.accel_z = sensor.acceleration

		self.A = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],				# State Transition Matrix
						   [0, 1, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 1, 0, 0, 0, 0, 0, 0],
						   [dt, 0, 0, 1, 0, 0, 0, 0, 0],
						   [0, dt, 0, 0, 1, 0, 0, 0, 0],
						   [0, 0, dt, 0, 0, 1, 0, 0, 0],
						   [0.5*dt*dt, 0, 0, dt, 0, 0, 1, 0, 0],
						   [0, 0.5*dt*dt, 0, 0, dt, 0, 0, 1, 0],
						   [0, 0, 0.5*dt*dt, 0, 0, dt, 0, 0, 1]])

		self.Q = np.array([[0.01, 0, 0, 0, 0, 0, 0, 0, 0],			# Process Noise Co-Variance Matrix
						  [0, 0.01, 0, 0, 0, 0, 0, 0, 0],
						  [0, 0, 0.01, 0, 0, 0, 0, 0, 0],
						  [0, 0, 0, 0.01, 0, 0, 0, 0, 0],
						  [0, 0, 0, 0, 0.01, 0, 0, 0, 0],
						  [0, 0, 0, 0, 0, 0.01, 0, 0, 0],
						  [0, 0, 0, 0, 0, 0, 0.01, 0, 0],
						  [0, 0, 0, 0, 0, 0, 0, 0.01, 0],
						  [0, 0, 0, 0, 0, 0, 0, 0, 0.01]])

		self.x = np.array([self.pos_x, self.pos_y, self.pos_z, 		# State Model
					  	   self.vel_x, self.vel_y, self.vel_z, 
					  	   self.accel_x, self.accel_y, self.accel_z])

		self.P = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],				# Co-Variance Matrix
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0],
						   [0, 0, 0, 0, 0, 0, 0, 0, 0]])

		self.H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],				# Observation Matrix
						  [0, 0, 0, 0, 0, 0, 0, 1, 0],
						  [0, 0, 0, 0, 0, 0, 0, 0, 1]])

		self.R = np.array([[5, 0, 0],								# Measurement Noise Covariance Matrix
						   [0, 5, 0],
						   [0, 0, 5]])

	def update(self):
		accel_x, accel_y, accel_z = self.sensor.acceleration
		new_state = np.array([accel_x, accel_y, accel_z])
		self.x = self.A.dot(self.x)
		self.P = self.A.dot(self.P).dot(self.A.T) + self.Q
		y = new_state - self.H.dot(self.x)							# Innovation Factor
		S = self.H.dot(self.P).dot(self.H.T) + self.R 				# Innovation Covariance
		K = self.P.dot(self.H.T).dot(np.linalg.pinv(S))				# Kalman Gain
		self.x = self.x + K.dot(y)
		self.P = (np.identity(9) - (K.dot(self.H))).dot(self.P)

	def get_state(self):
		return self.x[0], self.x[1], self.x[2], self.x[3], self.x[4], self.x[5], self.x[6], self.x[7], self.x[8]

class ComplFilter(Filter):
	def __init__(self, sensor, dt=0.01, alpha=0.02, g_gain=0.6):
		self.sensor = sensor
		self.dt = dt
		self.alpha = alpha
		self.g_gain = g_gain
		self.pitch, self.roll, self.yaw = 0.0, 0.0, 0.0

	def update(self):
		accel_x, accel_y, accel_z = self.sensor.acceleration
		gyro_x, gyro_y, gyro_z = self.sensor.acceleration
		mag_x, mag_y, mag_z = self.sensor.acceleration
		unit_accel_x, unit_accel_y, unit_accel_z = self.sensor.acceleration
		accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
		if accel_magnitude is not 0:
			unit_accel_x = accel_x/accel_magnitude
			unit_accel_y = accel_y/accel_magnitude
			unit_accel_z = accel_z/accel_magnitude

		pitch_accel = math.atan2(unit_accel_y, unit_accel_z) * 180/math.pi
		roll_accel = math.atan2(unit_accel_x, unit_accel_z) * 180/math.pi
		yaw_mag = math.atan2(mag_y, mag_x) * 180/math.pi

		self.pitch += gyro_x * self.g_gain * self.dt
		self.roll += gyro_y * self.g_gain * self.dt
		self.yaw += gyro_z * self.g_gain * self.dt

		self.pitch = self.pitch * (1-self.alpha) + pitch_accel * self.alpha
		self.roll = self.roll * (1-self.alpha) + roll_accel * self.alpha
		self.yaw = self.yaw * (1-self.alpha) + yaw_mag * self.alpha

	def get_state(self):
		return self.pitch, self.roll, self.yaw