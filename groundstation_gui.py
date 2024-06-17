import pygame
import time
import pandas as pd
import numpy as np
import os
import pygame.camera
import RPi_I2C_driver
import math
import pygame_chart as pyc


# Colors:
r=20
g=10
b=20
background_color = (40,69,108)
panel_color = (200,201,199)
text_color = (0,0,0)

mylcd = RPi_I2C_driver.lcd()

# pygame setup
pygame.init()

screen_width = 1920
screen_height = 1080
screen = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN | pygame.SCALED | pygame.HWSURFACE | pygame.DOUBLEBUF, 16)
clock = pygame.time.Clock()


Title_Font = pygame.font.SysFont("calibri",45)
Header_Font = pygame.font.SysFont("calibri", 50,True)
Subheader_Font = pygame.font.SysFont("calibri", 30,True)
Data_Font = pygame.font.SysFont("calibri", 50)
Small_Data_Font = pygame.font.SysFont("calibri", 30)
Chute_Font = pygame.font.SysFont("calibri", 35)
Clock_Font = pygame.font.SysFont("calibri", 40)
title_text = Title_Font.render("SAINT LOUIS UNIVERSITY ROCKET PROPULSION LAB: PROJECT GROUNDBREAKER", True, (255, 255, 255))
gndbkr_text = Header_Font.render("Groundbreaker", True, (255, 255, 255))
#gndstn_text = Header_Font.render("Gyro Plot", True, (255, 255, 255))

parachute_text = Subheader_Font.render("Parachutes", True, text_color)


rotation_header = Subheader_Font.render("Rotation", True, text_color)
acceleration_header = Subheader_Font.render("Accelerometer", True, text_color)
gyro_header = Subheader_Font.render("Gyro", True, text_color)
mag_header = Subheader_Font.render("Magnometer", True, text_color)


Groundbreaker_Panel = pygame.Rect(0, 760, 1125, screen_height/3)

Ground_Station_Panel = pygame.Rect(screen_width*(3/4) + 12, 760, screen_width/4, screen_height/3)



#print((screen_width-(camera_width*2))//3, (800-camera_height)/2, camera_width, camera_height)
#print(screen_width - (screen_width-(camera_width*2))//3 - camera_width, (800-camera_height)/2, camera_width, camera_height)
line_width = 8
column1_rect = pygame.Rect(0, Groundbreaker_Panel.top, 800+line_width, screen_height-Groundbreaker_Panel.top)
column2_rect = pygame.Rect(800, Groundbreaker_Panel.top, 325, screen_height-Groundbreaker_Panel.top)
#column3_rect = pygame.Rect(column2_rect.right-line_width, Groundbreaker_Panel.top, 1428-column2_rect.right+line_width, screen_height-Groundbreaker_Panel.top)

camera_width = 720
camera_height = 480


camera2 = pygame.Rect((column2_rect.right)+(screen_width-column2_rect.right-camera_width)/2, screen_height-camera_height-15, camera_width, camera_height)
camera2_text = Header_Font.render("Camera 2", True, (0,0,0))
camera1 = pygame.Rect((column2_rect.right)+(screen_width-column2_rect.right-camera_width)/2, camera2.top-camera_height-15, camera_width, camera_height)
camera1_text = Header_Font.render("Camera 1", True, (0,0,0))
"""camera1 = pygame.Rect((screen_width-(camera_width*2))//3, (800-camera_height)/2, camera_width, camera_height)
camera1_text = Header_Font.render("Camera 1", True, (0,0,0))
camera2 = pygame.Rect(screen_width - (screen_width-(camera_width*2))//3 - camera_width, (800-camera_height)/2, camera_width, camera_height)
camera2_text = Header_Font.render("Camera 2", True, (0,0,0))"""

pygame.camera.init()

altitude_plot = pyc.Figure(screen, 0, camera1.top, 1125//2, 315,bg_color=panel_color)
#altitude_plot_Rect = pygame.Rect(column2_rect.right, Groundbreaker_Panel.top, screen_width-column2_rect.right, screen_height-Groundbreaker_Panel.top)
temp_plot = pyc.Figure(screen, 0, camera1.top+315, 1125//2, 315,bg_color=panel_color)
#gps_plot = pyc.Figure(screen, 0, camera1.top+315, 1125//2, 315,bg_color=panel_color)
#gps_vel_plot = pyc.Figure(screen, 1125/2, camera1.top+315, 1125//2, 315,bg_color=panel_color)

gyro_plot = pyc.Figure(screen, column2_rect.right, Groundbreaker_Panel.top, screen_width-column2_rect.right, screen_height-Groundbreaker_Panel.top,bg_color=panel_color)
gyro_plot_Rect = pygame.Rect(column2_rect.right, Groundbreaker_Panel.top, screen_width-column2_rect.right, screen_height-Groundbreaker_Panel.top)

#telemetry = open("TELEMETRY.txt","r")
#telemetry = pd.read_csv("TELEMETRY.csv")

last_5_packets = []
edit_layout_mode = False
average_values = False

#Gyro Variables A0
launched = False
gyroRotation = np.array([0.0,90.0]) #rotation in spherical coordinates
gyroVectorY = np.array([0,1,0]) #rotation vector y cartesian
gyroVectorX = np.array([1,0,0]) #rotation vector x cartesian
gyroDeltaLast = [0,0,0]

signalLossCount = 0
class Data:
    def __init__(self, value):
        self.value = value
        self.text = ""

class Cam:
    def __init__(self, source):
        self.cam_list = pygame.camera.list_cameras()
        self.exists = True
        
        if os.path.exists(source):
            print(source, " does not exist!")
            self.exists = False
            return
        self.cam = pygame.camera.Camera(source, (camera_width,camera_height))
        self.cam.start()
        #self.cam_frame = pygame.surface.Surface((camera_width,camera_height), 0, screen)

def addGenericValues():
    altitude.value = 10000.00
    velocity.value = 400.00
    velocity_vector.value = [0.00, 400.00, 0.00]
    temperature.value = 69
    acceleration.value = [0.0, -32.2, 0.0]
    rotation_vector.value = [0,0,0]
    gyro.value = [0.0, 0.0, 0.0]
    mag.value = [0.036, 0.048, 0.061]
    signal_strength.value = -90
    gps_coords.value = [38, -90]
    gps_fix_quality.value = 1
    gps_satellites.value = 10

def updateDataText():
    altitude.text = Data_Font.render("Altitude: " + str(altitude.value) + " ft" + " ("+str(round(altitude.value * 0.3048, 2))+" m) AGL", True, text_color)
    velocity.text = Data_Font.render("Speed: " + str(velocity.value) + " ft/s" + " ("+str(round(velocity.value*0.681818,2))+" mph)", True, text_color)
    temperature.text = Data_Font.render("Tempurature: " + str(temperature.value) + "°F" + " ("+str(round((temperature.value-32)*(5/9), 2))+"°C)", True, text_color)

    acceleration.text = Small_Data_Font.render("("+str(acceleration.value[0])+", " + str(acceleration.value[1])+ ", " + str(acceleration.value[2])+ ")", True, text_color)
    rotation_vector.text = Small_Data_Font.render("("+str(rotation_vector.value[0])+", " + str(rotation_vector.value[1])+ ", " + str(rotation_vector.value[2])+ ")", True, text_color)
    gyro.text = Small_Data_Font.render("("+str(gyro.value[0])+", " + str(gyro.value[1])+ ", " + str(gyro.value[2])+ ")", True, text_color)
    mag.text = Small_Data_Font.render("("+str(mag.value[0])+", " + str(mag.value[1])+ ", " + str(mag.value[2])+ ")", True, text_color)

    signal_strength.text = Small_Data_Font.render("Telemetry Signal Strength: "+str(signal_strength.value)+" dBm", True, text_color)
    drogue.text = Chute_Font.render("Drogue: "+ drogue.value, True, text_color)
    main.text = Chute_Font.render("Main: "+ main.value, True, text_color)
    
    gps_coords.text = Small_Data_Font.render("Latitude: " + str(gps_coords.value[0]) + " Longitude: " + str(gps_coords.value[1]), True, text_color)
    gps_fix_quality.text = Small_Data_Font.render("GPS Fix: " + str(gps_fix_quality.value), True, text_color)
    gps_satellites.text = Small_Data_Font.render("# Satellites: " + str(gps_satellites.value), True, text_color)

def displayHeaderText():
    screen.blit(title_text,(screen_width/2 - title_text.get_width()//2, 25))
    screen.blit(camera1_text,(camera1.center[0]-camera1_text.get_width()//2,camera1.center[1]-camera1_text.get_height()//2))
    screen.blit(camera2_text,(camera2.center[0]-camera2_text.get_width()//2,camera2.center[1]-camera2_text.get_height()//2))

    # Column 2: (Center Indented)
    screen.blit(acceleration_header,(column2_rect.center[0] - (acceleration_header.get_width()//2), Groundbreaker_Panel.top + line_width))
    screen.blit(rotation_header,(column2_rect.center[0] - (rotation_header.get_width()//2), Groundbreaker_Panel.top + (column2_rect.height*0.25 +line_width)))
    screen.blit(gyro_header,(column2_rect.center[0] - (gyro_header.get_width()//2), Groundbreaker_Panel.top + (column2_rect.height*0.5 +line_width)))
    screen.blit(mag_header,(column2_rect.center[0] - (mag_header.get_width()//2), Groundbreaker_Panel.top + (column2_rect.height*0.75 +line_width)))

def displayDataText():
    screen.blit(altitude.text,(20, Groundbreaker_Panel.top+line_width))
    screen.blit(velocity.text,(20, Groundbreaker_Panel.top+60+line_width))
    screen.blit(temperature.text,(20, Groundbreaker_Panel.top+120+line_width))
    screen.blit(signal_strength.text,(20, Groundbreaker_Panel.top+190+line_width))
    screen.blit(gps_fix_quality.text,(column1_rect.right-gps_fix_quality.text.get_width()-10, Groundbreaker_Panel.top+190+line_width))
    screen.blit(gps_satellites.text,(column1_rect.right-gps_satellites.text.get_width()-10, Groundbreaker_Panel.top+225+line_width))
    screen.blit(gps_coords.text,(20, Groundbreaker_Panel.top+225+line_width))

    
    screen.blit(acceleration.text,(column2_rect.center[0] - acceleration.text.get_width()//2, Groundbreaker_Panel.top + column2_rect.height*(1/8)))
    screen.blit(rotation_vector.text,(column2_rect.center[0] - rotation_vector.text.get_width()//2, Groundbreaker_Panel.top + column2_rect.height*(3/8)))
    screen.blit(gyro.text,(column2_rect.center[0] - gyro.text.get_width()//2, Groundbreaker_Panel.top + column2_rect.height*(5/8)))
    screen.blit(mag.text,(column2_rect.center[0] - mag.text.get_width()//2, Groundbreaker_Panel.top + column2_rect.height*(7/8)))

    # Clock:
    hour, minute, second = time.strftime('%H'), time.strftime('%M'), time.strftime('%S')
    timeString = str(int(hour))+":"+str(minute)+":"+str(second) +" CDT"
    screen.blit(Data_Font.render(timeString, True, (255, 255, 255)),(5,Groundbreaker_Panel.top-40))

    #screen.blit(drogue.text,(column3_rect.center[0] - drogue.text.get_width()//2, Groundbreaker_Panel.top+35))
    #screen.blit(main.text,(column3_rect.center[0] - main.text.get_width()//2, Groundbreaker_Panel.top+75))
    
def makeRectBorder(rect=pygame.Rect):
    pygame.draw.rect(screen, panel_color, pygame.Rect(rect.left+line_width, rect.top+line_width, rect.width-line_width*2, rect.height-line_width*2))

def displayRects():
    pygame.draw.rect(screen, panel_color, Groundbreaker_Panel)
    pygame.draw.rect(screen, (255,255,255), camera1)
    pygame.draw.rect(screen, (255,255,255), camera2)

    pygame.draw.rect(screen, (0,61,165), column1_rect)
    makeRectBorder(column1_rect)
    pygame.draw.rect(screen, (0,61,165), column2_rect)
    makeRectBorder(column2_rect)

    pygame.draw.line(screen, (0,61,165), (0,Groundbreaker_Panel.top+180+line_width),(column1_rect.right-line_width/2,Groundbreaker_Panel.top+180+line_width),line_width)
    pygame.draw.line(screen, (0,61,165), (column2_rect.left,Groundbreaker_Panel.top),(column2_rect.right-line_width/2,Groundbreaker_Panel.top),line_width)
    pygame.draw.line(screen, (0,61,165), (column2_rect.left,Groundbreaker_Panel.top + column2_rect.height*1/4),(column2_rect.right-line_width/2,Groundbreaker_Panel.top + column2_rect.height*1/4),line_width)
    pygame.draw.line(screen, (0,61,165), (column2_rect.left,Groundbreaker_Panel.top + column2_rect.height*1/2),(column2_rect.right-line_width/2,Groundbreaker_Panel.top + column2_rect.height*1/2),line_width)
    pygame.draw.line(screen, (0,61,165), (column2_rect.left,Groundbreaker_Panel.top + column2_rect.height*3/4),(column2_rect.right-line_width/2,Groundbreaker_Panel.top + column2_rect.height*3/4),line_width)
    pygame.draw.line(screen, (0,61,165), (column2_rect.left,Groundbreaker_Panel.top + column2_rect.height),(column2_rect.right-line_width/2,Groundbreaker_Panel.top + column2_rect.height),line_width)

def getInitialVariables(packet):
    variables = []
    for i, var in enumerate(packet.split(',')):
        if var == '\n':
            continue
        variables.append(round(float(var),2),2)

    #alt0.value = round(variables[13]*3.28084,2)

def getTelemetry(packet, lastReceive): 
    global signalLossCount
    if packet['time'] - time_av.value < .4:
        signalLossCount = signalLossCount + 1
        if signalLossCount > 10:
            signal_strength.value = "No Signal"
        return
    else:
        signalLossCount = 0
        signal_strength.value = packet['telem_signal']
    time_av.value = packet['time']
    acceleration.value = [packet['accel_x'],
                          packet['accel_y'],
                          packet['accel_z']]
    
    mag.value = [packet['mag_x'],
                 packet['mag_y'],
                 packet['mag_z']]
    
    gyro.value = [packet['gyro_x'],
                  packet['gyro_y'],
                  packet['gyro_z'],]

    temperature.value = packet['temp']
    humidity.value = packet['humidity']
    pressure.value = packet['pressure']
    altitude.value = packet['altitude']

    if abs(gyro.value[0]) > .05 or abs(gyro.value[1]) > .05 or abs(gyro.value[2]) > .05:
        rotation_vector.value = [round((rotation_vector.value[0]+((gyro.value[0])*(time.time()-lastReceive))),2), 
                                round((rotation_vector.value[1]+((gyro.value[1])*(time.time()-lastReceive))),2),  
                                round((rotation_vector.value[2]+((gyro.value[2])*(time.time()-lastReceive))),2)]
    
    velocity.value = packet['gps_speed']#round(math.sqrt((velocity_vector.value[0]*velocity_vector.value[0]) + (velocity_vector.value[1]*velocity_vector.value[1]) + (velocity_vector.value[2]*velocity_vector.value[2])),2)
    
    #fix_q,sat,lat,long,gps_alt,gps_speed,telem_signal
    gps_fix_quality.value = packet['fix_q']
    gps_satellites.value = packet['sat']
    gps_coords.value = [packet['lat'], packet['long']]  
    gps_altitude.value = packet['gps_alt']
    gps_velocity.value = packet['gps_speed']
    
    event_bool.value = packet['event_bool']
    roll.value = packet['roll']
    pitch.value = packet['pitch']
    yaw.value = packet['yaw']

def showPlots(telemetry):
    if len(telemetry) > 2:
        altitude_plot.set_ylim((min(telemetry['telem_signal'])-.1,max(telemetry['telem_signal'])+.1))
        altitude_plot.add_gridlines()
        altitude_plot.add_title('Signal Strength')
        altitude_plot.line('Altitude',list(telemetry['time']),list(telemetry['telem_signal']))
        altitude_plot.draw()

        temp_plot.set_ylim((min(telemetry['temp'])-.1,max(telemetry['temp'])+.1))
        temp_plot.add_gridlines()
        temp_plot.add_title('Temperature')
        temp_plot.line('Temperature',list(telemetry['time']),list(telemetry['temp']))
        temp_plot.draw()

        """if telemetry.loc[len(telemetry)-1]['lat'] != 0 or  telemetry.loc[len(telemetry)-1]['long'] != 0:
            #print((round(telemetry.loc[len(telemetry)-1]['lat']-4),round(telemetry.loc[len(telemetry)-1]['lat']+4)))
            #print((round(telemetry.loc[len(telemetry)-1]['long']+4),round(telemetry.loc[len(telemetry)-1]['long']-4)))
            gps_plot.set_xlim((round(telemetry.loc[len(telemetry)-1]['lat']-1),round(telemetry.loc[len(telemetry)-1]['lat']+1)))
            gps_plot.set_ylim((round(telemetry.loc[len(telemetry)-1]['long']-1),round(telemetry.loc[len(telemetry)-1]['long']+1)))
            gps_plot.add_gridlines()
            gps_plot.add_title('GPS Coordinates')
            gps_plot.scatter('GPS',list(telemetry['lat']),list(telemetry['long']))
            gps_plot.draw()

            gps_vel_plot.set_ylim((min(telemetry['gps_speed'])-.1,max(telemetry['gps_speed'])+.1))
            gps_vel_plot.add_gridlines()
            gps_vel_plot.add_title('GPS Speed')
            gps_vel_plot.line('GPS Speed',list(telemetry['time']),list(telemetry['gps_speed']))
            gps_vel_plot.draw()"""
        """if len(telemetry) > 15:
            gyro_plot.add_legend()
            gyro_plot.add_gridlines()
            gyro_plot.add_title("Raw Gyro")
            gyro_plot.line('Gyro_x', list(telemetry['time'][len(telemetry)-10:len(telemetry)]),list(telemetry['gyro_x'][len(telemetry)-10:len(telemetry)]))
            #gyro_plot.line('Gyro_y', telemetry['time'],telemetry['gyro_y'],200)
            #gyro_plot.line('Gyro_z', telemetry['time'],telemetry['gyro_z'],200)
            gyro_plot.draw()"""
            

def guiloop():
    lastReceive = time.time()
    lastTextUpdate = 0

    launched = True
    gyroRotation = np.array([0.0,90.0]) #rotation in spherical coordinates
    gyroVectorY = np.array([0,1,0]) #rotation vector y cartesian
    gyroVectorX = np.array([1,0,0]) #rotation vector x cartesian
  
    lastRotate = 0
    # Load the image
    rotated_image = pygame.image.load("/home/groundstation/Desktop/Gyro_Models/0000.png")
    new_rect = rotated_image.get_rect(center=rotated_image.get_rect(center=(250,250)).center)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return
            
        #detects launch to set gyro 0 AO
        if((acceleration.value[1] < -20) or (altitude.value-0 > 20)) and not launched:
            launched = True


        telemetry = pd.read_csv("TELEMETRY.csv")
        if time.time()- lastReceive >= 0.1:
            if len(telemetry) > 0 and not edit_layout_mode:
                packet = telemetry.loc[len(telemetry)-1]
                getTelemetry(packet, lastReceive) 
            else:
                addGenericValues()

            lastReceive = time.time()    

        #if launch is detected start integrating AO
        if launched and time.time()- lastRotate >= 0.5:
            deltaT = time.time()- lastRotate
            
            #convert degree changes to vector changes
            #trapezoidY = deltaT * (gyroDeltaLast[1] + gyro.value[1]) / 2 #trapezoid integration approximation
            trapezoidY = deltaT * gyro.value[1]
            gyroVectorX = (gyroVectorX * np.cos(trapezoidY)) + (np.cross(gyroVectorX,gyroVectorY) * np.sin(trapezoidY)) #Roll

                                    
            #trapezoidZ = deltaT * (gyroDeltaLast[2] + gyro.value[2]) / 2 #trapezoid integration approximation
            trapezoidZ = deltaT * gyro.value[2]
            newVectorY = (gyroVectorX * np.sin(-trapezoidZ)) + (gyroVectorY * np.cos(trapezoidZ)) #Yaw
            newVectorX = (gyroVectorX * np.cos(trapezoidZ)) + (gyroVectorY * np.sin(trapezoidZ))
            gyroVectorY = newVectorY
            gyroVectorX = newVectorX

            #trapezoidX = deltaT * (gyroDeltaLast[0] + gyro.value[0]) / 2 #trapezoid integration approximation
            trapezoidX = deltaT * gyro.value[0]
            gyroVectorY = (gyroVectorY * np.cos(trapezoidX)) + (np.cross(gyroVectorX,gyroVectorY) * np.sin(trapezoidX)) #Pitch
            
            #convert rotation vector to spherical coordinates
            gyroRotation[0] = np.arctan2(gyroVectorY[1],gyroVectorY[0]) * 180 / np.pi
            gyroRotation[1] = np.arctan2(gyroVectorY[2],np.sqrt(gyroVectorY[1]**2 + gyroVectorY[0]**2)) * 180 / np.pi
                                    
            temp=gyroRotation
            #ensures temp values are positive
            if temp[0] < 0:
                temp[0] = 360 + temp[0]
            if temp[1] < 0:
                temp[1] = 360 + temp[1]

            # Load the image
            image = pygame.image.load("/home/groundstation/Desktop/Gyro_Models/"+'{:04d}'.format((2*temp[1]).astype(int))+".png")

            # Rotate the image
            rotated_image = pygame.transform.rotozoom(image, temp[0]-90, 1)
            rotated_image = pygame.transform.scale2x(rotated_image)
            new_rect = rotated_image.get_rect(center=image.get_rect(center=(1000,500)).center)
            lastRotate = time.time()

        screen.fill(background_color)
        #screen.blit(rotated_image, ((column2_rect.right - rotated_image.get_width()) + new_rect[0], 500 + new_rect[1]))
        screen.blit(rotated_image, new_rect)
        showPlots(telemetry)
        displayRects()
        displayHeaderText()
        if time.time() - lastTextUpdate > .05:
            updateDataText() 
            lastTextUpdate = time.time()
        displayDataText()
        
        
        if  cam1.exists and cam1.cam.query_image:
            screen.blit(cam1.cam.get_image().convert(), (camera1.left,camera1.top))
        """if cam2.cam.query_image and cam2.exists:
            screen.blit(cam2.cam.get_image().convert(), (camera2.left,camera2.top))"""
        
        pygame.display.update()
        clock.tick(60)  # limits FPS to 60
        

        # Display on LCD:
        mylcd.lcd_display_string_pos("A:{}".format(altitude.value), 1, 0)
        mylcd.lcd_display_string_pos("S:{}".format(signal_strength.value), 1, 11)
        mylcd.lcd_display_string_pos("T:{}".format(temperature.value), 2, 0)
        mylcd.lcd_display_string_pos("V:{}".format(velocity.value), 2, 8)


time_av = Data(0)
altitude = Data(0)
velocity = Data(0)
velocity_vector = Data([0,0,0])
rotation_vector = Data([0,0,0])
mach = Data(0)
temperature = Data(0)

acceleration = Data([0,0,0])
gyro = Data([0,0,0])
mag = Data([0,0,0])
humidity = Data(0)
pressure = Data(0)
signal_strength = Data(0)

gps_coords = Data([0,0])
gps_fix_quality = Data(0)
gps_altitude = Data(0)
gps_velocity = Data(0)
gps_satellites = Data(0)

event_bool = Data(0)
roll = Data(0)
pitch = Data(0)
yaw = Data(0)

drogue = Data("Stowed")
main = Data("Stowed")

v0 = Data([0,0,0])
a0 = Data([0,0,0])
alt0 = Data(0)
cam1 = Cam("/dev/video0")
#cam2 = Cam("/dev/video2")
guiloop()
pygame.quit()
