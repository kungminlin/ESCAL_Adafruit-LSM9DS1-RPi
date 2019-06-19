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

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0
elapsed = 0.0

start_time = time.time()

while True:
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	print("\033[2J")	
	print('\033[H{0:15s} ({1:7.3f}, {2:7.3f}, {3:7.3f})'.format('Acceleration:', accel_x, accel_y, accel_z))
	print('{0:15s} ({1:7.3f}, {2:7.3f}, {3:7.3f})'.format('Magnetometer:', mag_x, mag_y, mag_z))
	# print('Direction: {0:0.3f}'.format())
	print('{0:15s} ({1:7.3f}, {2:7.3f}, {3:7.3f})'.format('Gyroscope:', gyro_x, gyro_y, gyro_z))
	print('{0:15s} {1:7.3f}C'.format('Temperature:', temp))
	print('{0:15s} {1:0.2f}s'.format('Time Elapsed:', time.time()-start_time))
	print('\n\n')
	print('Direction: {0:>6s}'.format('Left' if gyro_z < 0 else 'Right'))
	time.sleep(0.02)

# Realtime Sensor Data Plotting
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