# Data Logging
import time
import board
import busio
import adafruit_lsm9ds1

# Data Plotting
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import dataplot
import math

# 3D Visualization
import visualization
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np

i2c = busio.I2C(board.SCL, board.SDA)		# Connect sensors via I2C
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)	# Identify sensor as Adafruit LSM9DS1

# Initial State
pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
accel_x, accel_y, accel_z = 0.0, 0.0, 0.0

roll, pitch = 0.0, 0.0						# Roll = Rotation about X-Axis, Pitch = Rotation about Y-Axis
elapsed = 0.0

start_time = time.time()
gyro_rotation = {'x': 0, 'y': 0, 'z': 0}

# Initialize 3D Animation
pygame.init()
display = (800,600)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
pygame.display.set_caption("Orientation Simulation")
gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
glTranslatef(0.0,0.0,-5)
prev_rot_x, prev_rot_y, prev_rot_z = 0.0, 0.0, 0.0

# Kalman Filter
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

def kalman_update(new_state):
	global x
	global P
	x = A.dot(x)
	P = A.dot(P).dot(A.T) + Q
	y = new_state - H.dot(x) 							# Innovation Factor
	S = H.dot(P).dot(H.T) + R 							# Innovation Co-Variance
	K = P.dot(H.T).dot(np.linalg.pinv(S))				# Kalman Gain
	x = x + K.dot(y)
	P = (np.identity(9) - (K.dot(H))).dot(H)

while True:
	# Get Sensor Input
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	# Normalize Acceleration
	accel_magnitude = math.sqrt(math.pow(accel_x, 2) + math.pow(accel_y, 2) + math.pow(accel_z, 2))
	unit_accel_x = accel_x/accel_magnitude
	unit_accel_y = accel_y/accel_magnitude
	unit_accel_z = accel_z/accel_magnitude

	print("\033[2J")	
	print('\033[H{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Acceleration:', accel_x, accel_y, accel_z+9.8))	# Acceleration (Accounting for Acceleration due to Gravity)
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Magnetometer:', mag_x, mag_y, mag_z))					# Magnetometer
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Gyroscope:', gyro_x, gyro_y, gyro_z))					# Gyroscope
	print('{0:15s} {1:8.3f}C'.format('Temperature:', temp))															# Temperature
	print('{0:15s} {1:8.2f}s'.format('Time Elapsed:', time.time()-start_time))										# Elapsed Time
	print('\n')
	print('{0:15s}  {1:>8s}'.format('Motion:', 'Up' if accel_z+9.8 > 0 else 'Down'))								# Acceleration Direction (Vertical)
	print('{0:15s}  {1:>8s}'.format('Turn (Local):', 'Left' if gyro_z < 0 else 'Right'))							# Local Turn (Only under correct orientation)
	# print('{0:15s}  {1:>8s}'.format('Turn (Global):', 'Left' if ))
	print('{0:15s} {1:8.3f}N'.format('Heading:', math.atan2(mag_y, mag_x) * 180 / math.pi))							# Compass Heading (Not Accounting for Magnetic Declination)
	print('\n')
	
	# Rotations with Gyroscope
	gyro_rotation['x'] += gyro_x*20/1000
	gyro_rotation['y'] += gyro_y*20/1000
	gyro_rotation['z'] += gyro_z*20/1000
	
	print('Rotations with Gyroscope')
	print('{0:15s} {1:8.3f}'.format('X Rotation:', gyro_rotation['x']))
	print('{0:15s} {1:8.3f}'.format('Y Rotation:', gyro_rotation['y']))
	print('{0:15s} {1:8.3f}'.format('Z Rotation:', gyro_rotation['z']))
	print('\n')

	# Rotations with Accelerometer
	roll = math.atan2(unit_accel_y, unit_accel_z) * 180/math.pi
	pitch = math.atan2((-unit_accel_x), math.sqrt(unit_accel_y*unit_accel_y+unit_accel_z*unit_accel_z)) * 180/math.pi
	
	print('Rotations with Accelerometer')
	print('{0:15s} {1:8.3f}'.format('Roll:', roll))
	print('{0:15s} {1:8.3f}'.format('Pitch:', pitch))


	# Quit Window Event
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			quit()

	# 3D Simulation
	# Gyroscope Rotation
	# glRotatef(gyro_x*20/1000, 1, 0, 0)
	# glRotatef(gyro_y*20/1000, 0, 1, 0)
	# glRotatef(gyro_z*20/1000, 0, 0, 1)
	# Accelerometer Rotation
	glRotatef(roll-prev_rot_x, 1, 0, 0)
	glRotatef(pitch-prev_rot_y, 0, 1, 0)
	prev_rot_x = roll
	prev_rot_y = pitch
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	visualization.Cube()
	pygame.display.flip()
	pygame.time.wait(10)

	kalman_update(np.array([accel_x, accel_y, accel_z]))
	print(x)

	time.sleep(0.02)


# Realtime Sensor Data Plotting
# fig = plt.figure()									# Define Plot (For Data Visualization)
# dataplot = dataplot.Dataplot(fig)
# dataplot.add_subplot("accel_x", "Accelerometer X")
# dataplot.add_subplot("accel_y", "Accelerometer Y")
# dataplot.add_subplot("accel_z", "Accelerometer Z")
# dataplot.add_subplot("gyro_x", "Gyroscope X")
# dataplot.add_subplot("gyro_y", "Gyroscope Y")
# dataplot.add_subplot("gyro_z", "Gyroscope Z")
# dataplot.add_subplot("mag_x", "Magnetometer X")
# dataplot.add_subplot("mag_y", "Magnetometer Y")
# dataplot.add_subplot("mag_z", "Magnetometer Z")
# dataplot.add_subplot("temp", "Temperature")

# def animate(i):
# 	accel_x, accel_y, accel_z = sensor.acceleration
# 	mag_x, mag_y, mag_z = sensor.magnetic
# 	gyro_x, gyro_y, gyro_z = sensor.gyro
# 	temp = sensor.temperature

# 	dataplot.update_data("accel_x", accel_x)
# 	dataplot.update_data("accel_y", accel_y)
# 	dataplot.update_data("accel_z", accel_z)

# 	dataplot.update_data("gyro_x", gyro_x)
# 	dataplot.update_data("gyro_y", gyro_y)
# 	dataplot.update_data("gyro_z", gyro_z)

# 	dataplot.update_data("mag_x", mag_x)
# 	dataplot.update_data("mag_y", mag_y)
# 	dataplot.update_data("mag_z", mag_z)

# 	dataplot.update_data("temp", temp)

# 	plt.xticks(rotation=45, ha='right')
# 	plt.subplots_adjust(bottom=0.30)
# 	plt.subplots_adjust(hspace=0.6)
	
# 	print("\033[2J")	
# 	print('\033[HAcceleration: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
# 	print('Magnetometer: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
# 	print('Gyroscope: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
# 	print('Temperature: {0:0.3f}C'.format(temp))

# ani = animation.FuncAnimation(fig, animate, interval=20)
# plt.show()