import time
import board
import busio
import adafruit_lsm9ds1
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import dataplot
import math

fig = plt.figure()									# Define Plot (For Data Visualization)
dataplot = dataplot.Dataplot(fig)
dataplot.add_subplot("accel_x", "Accelerometer X")
dataplot.add_subplot("accel_y", "Accelerometer Y")
dataplot.add_subplot("accel_z", "Accelerometer Z")
dataplot.add_subplot("gyro_x", "Gyroscope X")
dataplot.add_subplot("gyro_y", "Gyroscope Y")
dataplot.add_subplot("gyro_z", "Gyroscope Z")
dataplot.add_subplot("mag_x", "Magnetometer X")
dataplot.add_subplot("mag_y", "Magnetometer Y")
dataplot.add_subplot("mag_z", "Magnetometer Z")
dataplot.add_subplot("temp", "Temperature")

i2c = busio.I2C(board.SCL, board.SDA)		# Connect sensors via I2C
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)	# Identify sensor as Adafruit LSM9DS1

# while True:
	# accel_x, accel_y, accel_z = sensor.acceleration
	# mag_x, mag_y, mag_z = sensor.magnetic
	# gyro_x, gyro_y, gyro_z = sensor.gyro
	# temp = sensor.temperature

	# print("\033[2J")	
	# print('\033[HAcceleration: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
	# print('Magnetometer: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
	# print('Direction: {0:0.3f}'.format())
	# print('Gyroscope: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	# print('Temperature: {0:0.3f}C'.format(temp))

accel_x, accel_y, accel_z = sensor.acceleration
mag_x, mag_y, mag_z = sensor.magnetic
gyro_x, gyro_y, gyro_z = sensor.gyro
temp = sensor.temperature

dataplot.plot("accel_x", accel_x)
dataplot.plot("accel_y", accel_y)
dataplot.plot("accel_z", accel_z)

dataplot.plot("gyro_x", gyro_x)
dataplot.plot("gyro_y", gyro_y)
dataplot.plot("gyro_z", gyro_z)

dataplot.plot("mag_x", mag_x)
dataplot.plot("mag_y", mag_y)
dataplot.plot("mag_z", mag_z)

dataplot.plot("temp", temp)

# Realtime Sensor Data Plotting
def animate(i):
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	dataplot.update_data("accel_x", accel_x)
	dataplot.update_data("accel_y", accel_y)
	dataplot.update_data("accel_z", accel_z)

	dataplot.update_data("gyro_x", gyro_x)
	dataplot.update_data("gyro_y", gyro_y)
	dataplot.update_data("gyro_z", gyro_z)

	dataplot.update_data("mag_x", mag_x)
	dataplot.update_data("mag_y", mag_y)
	dataplot.update_data("mag_z", mag_z)

	dataplot.update_data("temp", temp)

	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.subplots_adjust(hspace=0.6)
	
	print("\033[2J")	
	print('\033[HAcceleration: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
	print('Magnetometer: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
	print('Gyroscope: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	print('Temperature: {0:0.3f}C'.format(temp))

ani = animation.FuncAnimation(self.fig, animate, interval=20)
# dataplot.animate(animate)
plt.show()