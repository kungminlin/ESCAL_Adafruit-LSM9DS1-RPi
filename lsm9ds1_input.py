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


i2c = busio.I2C(board.SCL, board.SDA)		# Connect sensors via I2C
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)	# Identify sensor as Adafruit LSM9DS1

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0
elapsed = 0.0

start_time = time.time()
gyro_rotation = {'x': 0, 'y': 0, 'z': 0}

# 3D Simulation Window
pygame.init()
display = (800,600)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
glTranslatef(5,5,0.0)
        

while True:
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	print("\033[2J")	
	print('\033[H{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Acceleration:', accel_x, accel_y, accel_z+9.8)) # Accounting for Acceleration due to Gravity
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Magnetometer:', mag_x, mag_y, mag_z))
	print('{0:15s} ({1:8.3f}, {2:8.3f}, {3:8.3f})'.format('Gyroscope:', gyro_x, gyro_y, gyro_z))
	print('{0:15s} {1:8.3f}C'.format('Temperature:', temp))
	print('{0:15s} {1:8.2f}s'.format('Time Elapsed:', time.time()-start_time))
	print('\n')
	print('{0:15s}  {1:>8s}'.format('Motion:', 'Up' if accel_z+9.8 > 0 else 'Down'))
	print('{0:15s}  {1:>8s}'.format('Turn:', 'Left' if gyro_z < 0 else 'Right'))
	print('{0:15s} {1:8.3f}N'.format('Heading:', math.atan2(mag_y, mag_x) * 180 / math.pi))
	print('\n')
	
	gyro_rotation['x'] += gyro_x*20/1000
	gyro_rotation['y'] += gyro_y*20/1000
	gyro_rotation['z'] += gyro_z*20/1000

	print('Rotations with Gyroscope')
	print('{0:15s} {1:8.3f}'.format('X Rotation:', gyro_rotation['x']))
	print('{0:15s} {1:8.3f}'.format('Y Rotation:', gyro_rotation['y']))
	print('{0:15s} {1:8.3f}'.format('Z Rotation:', gyro_rotation['z']))

	print('\n')
	print('Rotations with Accelerometer')
	print('{0:15s} {1:8.3f}'.format('Roll:', atan2(accel_y, accel_z) * 57.3))
	print('{0:15s} {1:8.3f}'.format('Pitch:', atan2((-accel_x), math.sqrt(accel_y*accel_y+accel_z*accel+z)) * 57.3))

	# Quit Window Event
	# for event in pygame.event.get():
	# 	if event.type == pygame.QUIT:
	# 		pygame.quit()
	# 		quit()

	# 3D Simulation
	glRotatef(gyro_x*20/1000, 1, 0, 0)
	glRotatef(gyro_y*20/1000, 0, 1, 0)
	glRotatef(gyro_z*20/1000, 0, 0, 1)
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	visualization.Cube()
	pygame.display.flip()
	pygame.time.wait(10)

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