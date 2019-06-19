import time
import board
import busio
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

while True:
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	print("\033[2J")	
	print('\033[HAcceleration: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
	print('Magnetometer: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
	print('Gyroscope: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
	print('Temperature: {0:0.3f}C'.format(temp))


