import time
import board
import busio
import digitalio
import adafruit_rfm9x
import adafruit_lsm9ds1
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_gps
import math

import serial
import time

class Events:
    def __init__(self) -> None:
        self.preLiftoff = True
        self.inFlight = False
        self.postFlight = False
        self.liftoff = False
        self.boosterBurnout = False
        self.apogee = False
        self.drogueDeploy = False
        self.mainDeploy = False
        self.touchdown = False
        self.timeOut = False

events = Events()

def inititalizeSensors():
    global gps
    global rfm9x
    global bme280
    global lsm9ds1
    
    i2c = board.I2C()
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    cs = digitalio.DigitalInOut(board.CE1)
    reset = digitalio.DigitalInOut(board.D25)

    # GPS:
    uart = serial.Serial("/dev/ttyS0",baudrate=9600,timeout=10)
    gps = adafruit_gps.GPS(uart, debug=False)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    gps.send_command(b"PMTK220, 1000")
    
    rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 433.0)
    rfm9x.tx_power = 17
    
    lsm9ds1 = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    
def calculate_tilt_angles(accel_x, accel_y, accel_z):
    # Calculate roll and pitch from accelerometer data
    roll = math.atan2(accel_y, accel_z)
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    # Convert to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    return roll, pitch

def calculate_yaw(mag_x, mag_y, mag_z, roll, pitch):
    # Calculate yaw from magnetometer data
    mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
    mag_y_comp = mag_x * math.sin(roll) * math.sin(pitch) + mag_y * math.cos(roll) - mag_z * math.sin(roll) * math.cos(pitch)
    yaw = math.atan2(-mag_y_comp, mag_x_comp)
    # Convert to degrees
    yaw = math.degrees(yaw)
    return yaw

lastCollected = time.time()
last_gps_update = time.monotonic()
startTime = time.time()


inititalizeSensors()

initial_altitude = bme280.altitude
init_accel_x,init_accel_y, init_accel_z = lsm9ds1.acceleration
init_gyro_x,init_gyro_y,init_gyro_z = lsm9ds1.gyro

accel_sumX0 = accel_sumY0 = accel_sumZ0 = accel_samps = accel_lastSamp = 0
accel_timeBtwnSamp = 0.001

gyro_sumX0 = gyro_sumY0 = gyro_sumZ0 = gyro_samps = gyro_lastSamp = 0
gyro_timeBtwnSamp = .001

print("Calibrating Sensors...")
sampleTime = 4 # sec
calibrationStart = time.time()
while time.time() - calibrationStart < sampleTime:
    if time.time() - gyro_lastSamp > gyro_timeBtwnSamp:
        print(time.time() - gyro_lastSamp)
        gyro_x,gyro_y,gyro_z = lsm9ds1.gyro
        gyro_lastSamp = time.time()

        gyro_sumX0 += gyro_x
        gyro_sumY0 += gyro_y
        gyro_sumZ0 += gyro_z
        gyro_samps += 1

    if time.time() - accel_lastSamp > accel_timeBtwnSamp:
        accel_x,accel_y,accel_z = lsm9ds1.acceleration
        accel_lastSamp = time.time()

        accel_sumX0 += accel_x
        accel_sumY0 += accel_y
        accel_sumZ0 += accel_z
        accel_samps += 1
        
i = 0
while i > 300:
    mag_rawX, mag_rawY, mag_rawZ = lsm9ds1.magnetic
    maxMagX = minMagX = mag_rawX
    maxMagY = minMagY = mag_rawY
    maxMagZ = minMagZ = mag_rawZ
    
    if mag_rawX > maxMagX: maxMagX = mag_rawX
    if mag_rawY > maxMagY: maxMagY = mag_rawY 
    if mag_rawZ > maxMagZ: maxMagZ = mag_rawZ 
    if mag_rawX < minMagX: minMagX = mag_rawX 
    if mag_rawY < minMagY: minMagY = mag_rawY 
    if mag_rawZ < minMagZ: minMagZ = mag_rawZ 
    time.sleep(.01)

mag_biasX = minMagX + (maxMagX - minMagX)/2
mag_biasY = minMagY + (maxMagY - minMagY)/2
mag_biasZ = minMagZ + (maxMagZ - minMagZ)/2

gyro_x0 = (gyro_sumX0/gyro_samps)
gyro_y0 = (gyro_sumY0/gyro_samps)
gyro_z0 = (gyro_sumZ0/gyro_samps)

accel_x0 = (accel_sumX0/accel_samps)
accel_y0 = (accel_sumY0/accel_samps)
accel_z0 = (accel_sumZ0/accel_samps)
print("Accelerometer error: ", accel_x0,accel_y0,accel_z0)
print("Gyro error: ", gyro_x0,gyro_y0,gyro_z0)

mag_x, mag_y, mag_z = lsm9ds1.magnetic
mag_x = mag_x - mag_biasX
mag_y = mag_y - mag_biasY
mag_z = mag_z - mag_biasZ

roll, pitch = calculate_tilt_angles(accel_x, accel_y, accel_z)
yaw = calculate_yaw(mag_x, mag_y, mag_z, math.radians(roll), math.radians(pitch))
print(f"Roll: {roll:.2f} degrees")
print(f"Pitch: {pitch:.2f} degrees")
print(f"Yaw: {yaw:.2f} degrees")

#currentEvent = 0 # 0: Preflight, 1: Liftoff, 2: Burnout, 3: Apogee, 4: Drogue Deploy, 5: Main Deploy, 6: Touchdown

baroApogee = False
event_bool = "0000000"
print("Transmitting!!")
while True:
    # get current event
    if events.preLiftoff:
        gps_sample_time = 10.0
        timeBtwnSamples = 3.0
    elif not events.preLiftoff and not events.touchdown:
        gps_sample_time = 1.0
        timeBtwnSamples = 0.001 
                
        if time.monotonic() - last_gps_update >= gps_sample_time:
            gps.update()
            last_gps_update - time.monotonic()
            
            if gps.altitude_m is None:
                gps.altitude_m = 0
            if gps.speed_knots is None:
                    gps.speed_knots = 0
                    
            if not gps.has_fix:
                gps.latitude = 0
                gps.longitude = 0
                gps.satellites = 0
        
        deltaT = time.time()-lastCollected
        if deltaT >= timeBtwnSamples:
            lastCollected = time.time()
            
            # Read acceleration, magnetometer, gyroscope, temperature.
            accel_x,accel_y, accel_z = lsm9ds1.acceleration
            mag_x, mag_y, mag_z = lsm9ds1.magnetic
            gyro_x, gyro_y, gyro_z = lsm9ds1.gyro # pitch, roll, yaw
            
            accel_x = accel_x-accel_x0
            accel_y = -1*(accel_y-accel_y0) # Keep value 'positive'
            accel_z = accel_z-accel_z0
            
            accel_vel = accel_vel + (accel_y * deltaT)
            
            gyro_x = gyro_x-gyro_x0
            gyro_y = gyro_y-gyro_y0
            gyro_z = gyro_z-gyro_z0
            
            temp = bme280.temperature * (9/5) + 32
            humidity = bme280.humidity
            pressure = bme280.pressure
            altitude = (bme280.altitude - initial_altitude) * 3.28084
            
            baro_vel = altitude / deltaT
        
            # Check liftoff
            if events.preLiftoff and accel_y > 2.5*32.2:
                events.liftoff =  True # Liftoff detected
                event_bool = "100000"
                
            # Check burnout 
            if not events.boosterBurnout and events.liftoff and accel_y <= 0:
                events.boosterBurnout = True # Detect burnout
                event_bool = "110000"
                
            # Check apogee
            if baro_vel < -10 and accel_vel < 70 and altitude < 13000:
                baroApogee = True
            if not events.apogee and events.boosterBurnout and baroApogee:
                events.apogee = True
                event_bool = "111000"
                apogeeTime = time.time()
            
            # Check drogue deploy
            if time.time() > apogeeTime + 10 and not events.drogueDeploy and baro_vel <  0 and baro_vel > -100:
                events.drogueDeploy = True
                event_bool = "111100"
                drogue_vel = baro_vel
                    
            # Check main deploy
            if not events.mainDeploy and baro_vel > drogue_vel:
                events.mainDeploy = True
                event_bool = "111110"
                
            # Check touchdown
            if not events.touchdown and baro_vel < 10 and baro_vel > -5:
                events.touchdown = True
                event_bool = "111111"
                
        data = "{0:0.3f},{1:0.3f},{2:0.3f},{3:0.3f},{4:0.3f},{5:0.3f},{6:0.3f},{7:0.3f},{8:0.3f},{9:0.3f},{10:0.3f},{11:0.3f},{12:0.3f},{13:0.3f},{14:0.3f},{15:0.3f},{16:0.3f},{17:0.3f},{18:0.3f},{19:0.3f},{20:0.3f},{21:0.3f},{22:0.3f},{23:0.3f},{24:0.3f},\n".format(
		time.time()-startTime ,accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, temp, humidity, pressure, altitude,
                gps.fix_quality, gps.satellites, gps.latitude, gps.longitude, gps.altitude_m*3.28084, gps.speed_knots*1.688, event_bool, roll, pitch, yaw)
        
        rfm9x.send(data.encode('utf-8'))
        
        

