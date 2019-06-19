import time
import board
import busio
import adafruit_lsm9ds1

import socket

HOST = socket.gethostname('127.0.0.1')
PORT = 5000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(5)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

while True:
	connection, address = s.accept()
	print('Client connection accepted; connected at: ' , address)
	while True:
		try:
			accel_x, accel_y, accel_z = sensor.acceleration
			mag_x, mag_y, mag_z = sensor.magnetic
			gyro_x, gyro_y, gyro_z = sensor.gyro
			temp = sensor.temperature
			
			data = str(accel_x)
			connection.send(data)
			
			print("\033[2J")	
			print('\033[HAcceleration: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
			print('Magnetometer: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
			print('Gyroscope: ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
			print('Temperature: {0:0.3f}C'.format(temp))
			time.sleep(1)
		except socket.error, msg:
			print('Client connection closed.')
			break

connection.close()


