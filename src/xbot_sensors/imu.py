import smbus
import math
import time
from mybot_sdk.robot_setup import get_robot_cfg

class MPU6050:
	def __init__(self):
		# Class / object / constructor setup
		self.gx = None; self.gy = None; self.gz = None;
		self.ax = None; self.ay = None; self.az = None;
		
		self.MPU6050 = get_robot_cfg()["Sensors"]["IMU"]["MPU6050"]
		GYRO = 250 # 250, 500, 1000, 2000 [deg/s]
		ACCELERATION = 2 # 2, 4, 7, 16 [g]
		TAU = 0.98

		CALIBRATION_POINTS = 500
		self.ACCURACY = 1
		self.RAW_ACCURACY = 3

		self.gyroXcal = 0
		self.gyroYcal = 0
		self.gyroZcal = 0

		self.gyroRoll = 0
		self.gyroPitch = 0
		self.gyroYaw = 0

		self.roll = 0
		self.pitch = 0
		self.yaw = 0

		self.dtTimer = 0
		self.tau = TAU

		self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(GYRO)
		self.accScaleFactor, self.accHex = self.accelerometerSensitivity(ACCELERATION)

		self.bus = smbus.SMBus(1)
		self.address = self.MPU6050["ADDRESS"]

		self.setUp()
		self.calibrateGyro(CALIBRATION_POINTS)

	def gyroSensitivity(self, x):
		# Create dictionary with standard value of 500 deg/s
		return {
			250:  [131.0, 0x00],
			500:  [65.5,  0x08],
			1000: [32.8,  0x10],
			2000: [16.4,  0x18]
		}.get(x,  [65.5,  0x08])

	def accelerometerSensitivity(self, x):
		# Create dictionary with standard value of 4 g
		return {
			2:  [16384.0, 0x00],
			4:  [8192.0,  0x08],
			8:  [4096.0,  0x10],
			16: [2048.0,  0x18]
		}.get(x,[8192.0,  0x08])

	def setUp(self):
		# Activate the MPU-6050
		self.bus.write_byte_data(self.address, self.MPU6050["PWR_MGMT_1"], 0x00)

		# Configure the accelerometer
		self.bus.write_byte_data(self.address, 0x1C, self.accHex)

		# Configure the gyro
		self.bus.write_byte_data(self.address, self.MPU6050["GYRO_CONFIG"], self.gyroHex)

		# Display message to user
		print("MPU set up:")
		print('\tAccelerometer: ' + str(self.accHex) + ' ' + str(self.accScaleFactor))
		print('\tGyro: ' + str(self.gyroHex) + ' ' + str(self.gyroScaleFactor) + "\n")
		time.sleep(2)

	def eightBit2sixteenBit(self, reg):
		# Reads high and low 8 bit values and shifts them into 16 bit
		h = self.bus.read_byte_data(self.address, reg)
		l = self.bus.read_byte_data(self.address, reg+1)
		val = (h << 8) + l

		# Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val

	def getRawData(self):
		self.gx = self.eightBit2sixteenBit(self.MPU6050["GYRO_XOUT_H"])
		self.gy = self.eightBit2sixteenBit(self.MPU6050["GYRO_YOUT_H"])
		self.gz = self.eightBit2sixteenBit(self.MPU6050["GYRO_ZOUT_H"])

		self.ax = self.eightBit2sixteenBit(self.MPU6050["INT_ENABLE"])
		self.ay = self.eightBit2sixteenBit(self.MPU6050["ACCEL_YOUT_H"])
		self.az = self.eightBit2sixteenBit(self.MPU6050["ACCEL_ZOUT_H"])

	def calibrateGyro(self, N):
		# Display message
		print("Calibrating gyro with " + str(N) + " points. Do not move!")

		# Take N readings for each coordinate and add to itself
		for ii in range(N):
			self.getRawData()
			self.gyroXcal += self.gx
			self.gyroYcal += self.gy
			self.gyroZcal += self.gz

		# Find average offset value
		self.gyroXcal /= N
		self.gyroYcal /= N
		self.gyroZcal /= N

		# Display message and restart timer for comp filter
		print("Calibration complete")
		print("\tX axis offset: " + str(round(self.gyroXcal,1)))
		print("\tY axis offset: " + str(round(self.gyroYcal,1)))
		print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")
		time.sleep(2)
		self.dtTimer = time.time()

	def processIMUvalues(self):
		# Update the raw data
		self.getRawData()

		# Subtract the offset calibration values
		self.gx -= self.gyroXcal
		self.gy -= self.gyroYcal
		self.gz -= self.gyroZcal

		# Convert to instantaneous degrees per second
		self.gx /= self.gyroScaleFactor
		self.gy /= self.gyroScaleFactor
		self.gz /= self.gyroScaleFactor

		# Convert to g force
		self.ax /= self.accScaleFactor
		self.ay /= self.accScaleFactor
		self.az /= self.accScaleFactor

		gx = self.gx 
		gy = self.gy 
		gz = self.gz 

		ax = self.ax 
		ay = self.ay 
		az = self.az 

		return gx,gy,gz,ax,ay,az

	def compFilter(self):
		# Get the processed values from IMU
		gx,gy,gz,ax,ay,az = self.processIMUvalues()

		# Get delta time and record time for next call
		dt = time.time() - self.dtTimer
		self.dtTimer = time.time()

		# Acceleration vector angle
		accPitch = math.degrees(math.atan2(ay, az))
		accRoll = math.degrees(math.atan2(ax, az))

		# Gyro integration angle
		self.gyroRoll -= gy * dt
		self.gyroPitch += gx * dt
		self.gyroYaw += gz * dt
		self.yaw = self.gyroYaw

		# Comp filter
		self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
		self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

		# Print data
		# print(" R: " + str(round(self.roll,1)) \
		#     + " P: " + str(round(self.pitch,1)) \
		#     + " Y: " + str(round(self.yaw,1)))
		
		return self.roll,self.pitch,self.yaw

	def get_roll(self):
		roll = self.compFilter()[0]
		return round(roll,self.ACCURACY)

	def get_pitch(self):
		pitch = self.compFilter()[1]
		return round(pitch,self.ACCURACY)

	def get_yaw(self):
		yaw = self.compFilter()[2]
		return round(yaw,self.ACCURACY)

	def get_gx(self):
		gx = self.processIMUvalues()[0]
		return round(gx,self.RAW_ACCURACY)

	def get_gy(self):
		gy = self.processIMUvalues()[1]
		return round(gy,self.RAW_ACCURACY)

	def get_gz(self):
		gz = self.processIMUvalues()[2]
		return round(gz,self.RAW_ACCURACY)

	def get_ax(self):
		ax = self.processIMUvalues()[3]
		return round(ax,self.RAW_ACCURACY)

	def get_ay(self):
		ay = self.processIMUvalues()[4]
		return round(ay,self.RAW_ACCURACY)

	def get_az(self):
		az = self.processIMUvalues()[5]
		return round(az,self.RAW_ACCURACY)

# def main():
#     mpu = MPU6050()
#     while(1):
# 		print("Gx: "+str(mpu.get_gx()), str(mpu.get_gz()))
# 		time.sleep(0.033333333)

# # Main loop
# if __name__ == '__main__':
# 	main()