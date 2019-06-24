import sys
import math

# Data Logging
import time
import board
import busio
import adafruit_lsm9ds1

# Data Plotting
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime
import dataplot 							# Local Module


# 3D Visualization
import visualization 						# Local Module
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np
import datafilter	 						# Local Module

i2c = busio.I2C(board.SCL, board.SDA)		# Connect sensors via I2C
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)	# Identify sensor as Adafruit LSM9DS1

roll_accel, pitch_accel = 0.0, 0.0			# Roll = Rotation about X-Axis, Pitch = Rotation about Y-Axis, Yaw = Rotation about Z-Axis
roll, pitch = 0.0, 0.0
elapsed = 0.0

# Initialization
start_time = time.time()
gyro_rotation = {'x': 0, 'y': 0, 'z': 0}
prev_rot_x, prev_rot_y, prev_rot_z = 0.0, 0.0, 0.0

dt = 0.01
kf = datafilter.KalmanFilter(sensor, dt)
cf = datafilter.ComplFilter(sensor, dt)

# Initialize 3D Animation
pygame.init()
display = (800,600)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
pygame.display.set_caption("Orientation Simulation")
gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
glTranslatef(0.0,0.0,-5)

# Data Plots
fig = plt.figure()
if len(sys.argv) > 1 and sys.argv[1] == "dataplot":
	plot = dataplot.Dataplot()
	plot.add_subplot("accel_x")
	# plot.add_subplot("accel_y")
	# plot.add_subplot("accel_z")
	# plot.add_subplot("gyro_x")
	# plot.add_subplot("gyro_y")
	# plot.add_subplot("gyro_z")
	# plot.add_subplot("mag_x")
	# plot.add_subplot("mag_y")
	# plot.add_subplot("mag_z")

	def realtime_dataplot(i):
		accel_x, accel_y, accel_z = sensor.acceleration
		mag_x, mag_y, mag_z = sensor.magnetic
		gyro_x, gyro_y, gyro_z = sensor.gyro
		temp = sensor.temperature
		plot.update_data("accel_x", str(datetime.datetime.now()), accel_x)
		# plot.update_data("accel_y", str(datetime.datetime.now()), accel_y)
		# plot.update_data("accel_z", str(datetime.datetime.now()), accel_z)
		# plot.update_data("gyro_x", str(datetime.datetime.now()), gyro_x)
		# plot.update_data("gyro_y", str(datetime.datetime.now()), gyro_y)
		# plot.update_data("gyro_z", str(datetime.datetime.now()), gyro_z)
		# plot.update_data("mag_x", str(datetime.datetime.now()), mag_x)
		# plot.update_data("mag_y", str(datetime.datetime.now()), mag_y)
		# plot.update_data("mag_z", str(datetime.datetime.now()), mag_z)

	anim = animation.FuncAnimation(fig, realtime_dataplot, interval=20)
	plt.show()

# Euler Angles to Quaternions Conversion
def euler_to_quaternion(yaw, pitch, roll):
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)

	q = {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
	q['w'] = cy * cp * cr + sy * sp * sr
	q['x'] = cy * cp * sr - sy * sp * cr
	q['y'] = sy * cp * sr + cy * sp * cr
	q['z'] = sy * cp * cr - cy * sp * sr

	return q

# Main Loop
while True:
	# Get Sensor Input
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	# Rotations with Accelerometer and Magnetometer (Default)
	pitch = math.atan2(accel_y, accel_z) * 180/math.pi
	roll = math.atan2(accel_x, accel_z) * 180/math.pi
	yaw = math.atan2(mag_y, mag_x) * 180/math.pi


	# Kalman Filter
	kf.update()
	pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, kf_accel_x, kf_accel_y, kf_accel_z = kf.get_state()

	# Complementary Filter
	cf.update()
	cf_pitch, cf_roll = cf.get_state()

	if len(sys.argv) > 1 and sys.argv[1] == "kalman_filter":
		accel_x, accel_y, accel_z = kf_accel_x, kf_accel_y, kf_accel_z

	if len(sys.argv) > 1 and sys.argv[1] == "compl_filter":
		pitch, roll = cf_pitch, cf_roll

	print("\033[2J")	
	print('\033[H{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Acceleration:', accel_x, accel_y, accel_z))	# Acceleration (Accounting for Acceleration due to Gravity)
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Magnetometer:', mag_x, mag_y, mag_z))				# Magnetometer
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Gyroscope:', gyro_x, gyro_y, gyro_z))				# Gyroscope
	print('{0:15s} {1:8.3f}C'.format('Temperature:', temp))														# Temperature
	print('{0:15s} {1:8.2f}s'.format('Time Elapsed:', time.time()-start_time))									# Elapsed Time
	print('\n')
	print('{0:15s}  {1:>8s}'.format('Motion:', 'Up' if accel_z+9.8 > 0 else 'Down'))							# Acceleration Direction (Vertical)
	print('{0:15s}  {1:>8s}'.format('Turn (Local):', 'Left' if gyro_z < 0 else 'Right'))						# Local Turn (Only under correct orientation)
	# print('{0:15s}  {1:>8s}'.format('Turn (Global):', 'Left' if ))
	print('{0:15s} {1:8.3f}N'.format('Heading:', yaw))															# Compass Heading (Not Accounting for Magnetic Declination)
	print('\n')
	
	# Rotations with Gyroscope
	gyro_rotation['x'] += gyro_x*dt
	gyro_rotation['y'] += gyro_y*dt
	gyro_rotation['z'] += gyro_z*dt
	
	print('Rotations with Gyroscope')
	print('{0:15s} {1:8.3f}'.format('X Rotation:', gyro_rotation['x']))
	print('{0:15s} {1:8.3f}'.format('Y Rotation:', gyro_rotation['y']))
	print('{0:15s} {1:8.3f}'.format('Z Rotation:', gyro_rotation['z']))
	print('\n')
	
	print('Rotations with Accelerometer and Magnetometer')
	print('{0:15s} {1:8.3f}'.format('Roll (x):', roll))
	print('{0:15s} {1:8.3f}'.format('Pitch (y):', pitch))
	print('{0:15s} {1:8.3f}'.format('Yaw (z):', yaw))
	print('\n')

	print('Complementary Filter Predictions')
	print('{0:15s} {1:8.3f}'.format('Roll:', cf_roll))
	print('{0:15s} {1:8.3f}'.format('Pitch:', cf_pitch))
	print('\n')

	print('Kalman Filter Predictions')
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Position:', pos_x, pos_y, pos_z))
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Velocity:', vel_x, vel_y, vel_z))
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Acceleration:', kf_accel_x, kf_accel_y, kf_accel_z))
	print('\n')

	print('Quaternion: ' + str(euler_to_quaternion(yaw, pitch, roll)))
	print('\n')

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
	glRotatef(yaw-prev_rot_z, 0, 0, 1)



	prev_rot_x = roll
	prev_rot_y = pitch
	prev_rot_z = yaw
	
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	visualization.Cube()
	pygame.display.flip()
	pygame.time.wait(10)

	time.sleep(dt)