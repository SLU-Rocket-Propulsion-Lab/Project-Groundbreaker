import board
import busio
import digitalio
import adafruit_rfm9x
import time
import csv

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

cs = digitalio.DigitalInOut(board.CE1)
reset = digitalio.DigitalInOut(board.D25)

rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 433.0)

lastReceive = 0
f = open("TELEMETRY.csv", "w+")
csv.writer(f).writerow(["time", "accel_x", "accel_y", "accel_z", "mag_x", "mag_y", "mag_z", "gyro_x", "gyro_y", "gyro_z", "temp", "humidity", "pressure", "altitude", "fix_q", "sat", "lat", "long", "gps_alt", "gps_speed", "telem_signal"])
f.close()
while True:
    with open('TELEMETRY.csv','a') as f1:
        if time.time()-lastReceive >= 0.05:
            packet = rfm9x.receive()  # Wait for a packet to be received (up to 0.5 seconds)
            if packet is not None:
                variables=[]
                signal_strength = rfm9x.last_rssi
                
                packet_text = str(packet, 'ascii')
                for index, var in enumerate(packet_text.split(',')):
                    if var == '\n':
                        continue
                    variables.append(float(var))
                variables.append(signal_strength)
                
                writer = csv.writer(f1) 
                writer.writerow(variables)
                lastReceive = time.time()
            
