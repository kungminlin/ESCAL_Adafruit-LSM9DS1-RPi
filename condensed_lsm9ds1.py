import time
import board
import busio
import adafruit_lsm9ds1
import math

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

while True:
	accel_x, accel_y, accel_z = sensor.acceleration
	mag_x, mag_y, mag_z = sensor.magnetic
	gyro_x, gyro_y, gyro_z = sensor.gyro
	temp = sensor.temperature

	unit_accel_x, unit_accel_y, unit_accel_z = 0, 0, 0
	accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
	if accel_magnitude is not 0:
		unit_accel_x = accel_x/accel_magnitude
		unit_accel_y = accel_y/accel_magnitude
		unit_accel_z = accel_z/accel_magnitude

	roll = math.atan2(unit_accel_y, unit_accel_z) * 180/math.pi
	pitch = math.atan2((-unit_accel_x), math.sqrt(unit_accel_y**2 + unit_accel_z**2)) * 180/math.pi
