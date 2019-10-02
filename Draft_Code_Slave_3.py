#This is the Slave device, that will be controlled by the network coordinator by a bluetooth connection running in the background.

#The code to use the PiCamera to count passengers was adapted from the work of Pedro Henrique Fonseca Bertoleti, found on:
#https://www.hackster.io/phfbertoleti/counting-objects-in-movement-using-raspberry-pi-opencv-015ba5
#Accessed first on July 20, 2019

#The methods to connect to Bluetooth are based on the PyBluez library found on:
#https://people.csail.mit.edu/albert/bluez-intro/x232.html
#Accessed first on July 25, 2019

#The methods to monitor the accelometer was adapted from the work of Saddam, found on:
#https://circuitdigest.com/microcontroller-projects/mpu6050-gyro-sensor-interfacing-with-raspberry-pi
#Accessed first on July 27, 2019

#All the cited code has been adapted, debugged and mixed by the student(JCFG) to meet the specific requirements of this project.

#Library to run several concurrent threads
import threading

#Library to send bluetooth messages
import bluetooth

#Libraries to operate the camera and passenger counter
import math
import cv2
import numpy as np
import imutils
from imutils.video import pivideostream
from imutils.video import VideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera

#Libraries to connect the RP0 to the sensors
import RPi.GPIO as GPIO
import serial
import random, time, datetime
import unicodedata

#Library to link the accelerometer
import smbus

#PC (Passenger counter): Declare the global variables for the video frame and passenger counter:
width = 0
height = 0
EntranceCounter = 0;
ExitCounter = 0;
#MinCountourArea = 1000  #Adjust ths value according to the size of the monitored space.
#BinarizationThreshold =65  #Adjust ths value according to to lightning conditions
#OffsetRefLines = 80  #Adjust ths value according to the camera characteristics

#PC: We initialize the PiCamera and discard some frames while it adapts to the lightning:
#camera = VideoStream(usePiCamera= True).start()
#time.sleep(2.0)
#ReferenceFrame = None

#for i in range(0,30):
#    (grabbed, Frame) = camera.read(), camera.read()

#Bluetooth: Declare the MAC addresses of all devices in the network:
MACAddress_Master = "B8:27:EB:E0:8A:EF"
MACAddress_Slave ="B8:27:EB:BB:85:7B"

#Accel: We declare the address of the sensor on the RP0 and the initial variables:

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1)
address = 0x68   # device address
 
AxCal=0
AyCal=0
AzCal=0
GxCal=0
GyCal=0
GzCal=0


#The global variable SPEED will start or stop the main sensor thread. 
#We declare the maximum acceleration variable that considers all three  axis.
speed = 0;
max_accel = 0;
max_magnitude = 0;

#Accel: We declare the methods to read the ports (bytes):
def read_byte(reg):
    return bus.read_byte_data(address, reg)

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

#Bluetooth: Method to receive messages from the BT network:
def receiveMessages():
  server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
  
  port = 1
  server_sock.bind(("",port))
  server_sock.listen(1)
  
  client_sock,address = server_sock.accept()
  print ("Accepted connection from " + str(address))
  
  data = client_sock.recv(1024)
  print ("received [%s]" % data)
  
  client_sock.close()
  server_sock.close()
  return data

#Bluetooth: Method to send messages to the BT network:  
def sendMessageTo(targetBluetoothMacAddress, message):
  port = 1
  sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
  sock.connect((targetBluetoothMacAddress, port))
  sock.send(message)
  sock.close()

#Bluetooth: Method to check available devices on the network. Only used on initial discovery.  
def lookUpNearbyBluetoothDevices():
  nearby_devices = bluetooth.discover_devices()
  for bdaddr in nearby_devices:
    print (str(bluetooth.lookup_name( bdaddr )) + " [" + str(bdaddr) + "]")

#Accel: This method reads the data on the specified device address
def readMPU(addr):
     high = bus.read_byte_data(Device_Address, addr)
     low = bus.read_byte_data(Device_Address, addr+1)
     value = ((high << 8) | low)
     if(value > 32768):
           value = value - 65536
     return value

#Accel: These methods read the acceleration, temperature and rotation in three axis, and normalizes the data using the calibration values.
def accel():
     x = readMPU(ACCEL_X)
     y = readMPU(ACCEL_Y)
     z = readMPU(ACCEL_Z)
     Ax = (x/16384.0-AxCal)
     Ay = (y/16384.0-AyCal)
     Az = (z/16384.0-AzCal)
#     print("X="+str(Ax))
#     print("Y="+str(Ay))
#     print("Z="+str(Az))
     time.sleep(.01)
     return Ax, Ay, Az

def gyro():
      x = readMPU(GYRO_X)
      y = readMPU(GYRO_Y)
      z = readMPU(GYRO_Z)
      Gx = x/131.0 - GxCal
      Gy = y/131.0 - GyCal
      Gz = z/131.0 - GzCal
      print ("X="+str(Gx))
      time.sleep(.01)
 
def temp():
  tempRow=readMPU(TEMP)
  tempC=(tempRow / 340.0) + 36.53
  tempC="%.2f" %tempC
  print (tempC)
  print("Temp: ")
  print(str(tempC))
  time.sleep(.2)
  
#Accel: This method calibrates the MPU6050 a nd defines the calibration values. The device should be static when this method runs.
def calibrate():
  print("Calibrating....")
  global AxCal
  global AyCal
  global AzCal
  x=0
  y=0
  z=0
  for i in range(50):
      x = x + readMPU(ACCEL_X)
      y = y + readMPU(ACCEL_Y)
      z = z + readMPU(ACCEL_Z)
  x= x/50
  y= y/50
  z= z/50
  AxCal = x/16384.0
  AyCal = y/16384.0
  AzCal = z/16384.0
  
  print (AxCal)
  print (AyCal)
  print (AzCal)
 
  global GxCal
  global GyCal
  global GzCal

  x=0
  y=0
  z=0

  for i in range(50):
    x = x + readMPU(GYRO_X)
    y = y + readMPU(GYRO_Y)
    z = z + readMPU(GYRO_Z)
  x= x/50
  y= y/50
  z= z/50
  GxCal = x/131.0
  GyCal = y/131.0
  GzCal = z/131.0
 
  print (GxCal)
  print (GyCal)
  print (GzCal)  

def get_accelerometer(accelerometer_xout,accelerometer_yout,accelerometer_zout):
  accelerometer_xout = read_word_2c(0x3b)
  accelerometer_yout = read_word_2c(0x3d)
  accelerometer_zout = read_word_2c(0x3f)

  Ax = accelerometer_xout / 16384.0
  Ay = accelerometer_yout / 16384.0
  Az = accelerometer_zout / 16384.0
  return(Ax,Ay,Az)
  
#PC: Check if an object in entering in monitored zone.
def CheckEntranceLineCrossing(y, CoorYEntranceLine, CoorYExitLine):
  AbsDistance = abs(y - CoorYEntranceLine)

  if ((AbsDistance <= 2) and (y < CoorYExitLine)):
    return 1
  else:
    return 0

#PC: Check if an object is exiting the monitored zone
def CheckExitLineCrossing(y, CoorYEntranceLine, CoorYExitLine):
  AbsDistance = abs(y - CoorYExitLine)

  if ((AbsDistance <= 2) and (y > CoorYEntranceLine)):
    return 1
  else:
    return 0

#PC: This method reads the Video Stream from the PiCamera and returns the number of entrances and exits.
#def PassengerCounter(): 

#We declare the person counter as global variables       
#    global EntranceCounter
#    global ExitCounter
#    global ReferenceFrame
    
#PC: Read the video stream from the PiCamera:
#    (grabbed, Frame) = camera.read(),camera.read()
#    height = np.size(Frame,0)
#    width = np.size(Frame,1)

#PC: This functions are used to apply gray-scale convertion and Gaussian blur filter
#    GrayFrame = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
#    GrayFrame = cv2.GaussianBlur(GrayFrame, (21, 21), 0)

#    if ReferenceFrame is None:
#        ReferenceFrame = GrayFrame
#        continue

#PC: Background subtraction and image binarization, compared to the Reference Frame
#    FrameDelta = cv2.absdiff(ReferenceFrame, GrayFrame)
#    FrameThresh = cv2.threshold(FrameDelta, BinarizationThreshold, 255, cv2.THRESH_BINARY)[1]

#PC: Dilate image and find all the contours
#    FrameThresh = cv2.dilate(FrameThresh, None, iterations=2)
#    _, cnts, _ = cv2.findContours(FrameThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#    QttyOfContours = 0

#PC: We define the reference lines (entrance and exit) and convert them to Integers for line function
#    CoorYEntranceLine = (height / 2)-OffsetRefLines
#    CoorYExitLine = (height / 2)+OffsetRefLines
#    CoorYEntranceLine = int(CoorYEntranceLine)
#    CoorYExitLine = int(CoorYExitLine)

#    cv2.line(Frame, (0,CoorYEntranceLine), (width,CoorYEntranceLine), (255, 0, 0), 2)
#    cv2.line(Frame, (0,CoorYExitLine), (width,CoorYExitLine), (0, 0, 255), 2)


    #check all found countours
#    for c in cnts:
        #if a contour has small area, it'll be ignored
#        if cv2.contourArea(c) < MinCountourArea:
#            continue

#        QttyOfContours = QttyOfContours+1

        #draw an rectangle "around" the object
#        (x, y, w, h) = cv2.boundingRect(c)
#        cv2.rectangle(Frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #find object's centroid
#        CoordXCentroid = (x+x+w)/2
#        CoordYCentroid = (y+y+h)/2
#        CoordXCentroid = int(CoordXCentroid)
#        CoordYCentroid = int(CoordYCentroid)

#        ObjectCentroid = (CoordXCentroid,CoordYCentroid)
#        cv2.circle(Frame, ObjectCentroid, 1, (0, 0, 0), 5)
#        if (CheckEntranceLineCrossing(CoordYCentroid,CoorYEntranceLine,CoorYExitLine)):
#          EntranceCounter += 1

#        if (CheckExitLineCrossing(CoordYCentroid,CoorYEntranceLine,CoorYExitLine)):
#          ExitCounter += 1

#    print( "Total countours found: " + str(QttyOfContours))

    #PC: ONLY FOR TESTING: Write entrance and exit counter values on frame and shows it
    #   cv2.putText(Frame, "Entrances: {}".format(str(EntranceCounter)), (10, 50),
    #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 0, 1), 2)
    #    cv2.putText(Frame, "Exits: {}".format(str(ExitCounter)), (10, 70),
    #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    #    cv2.imshow("Original Frame", Frame)
    #    cv2.waitKey(1);

#    print("Fin del programa")
    # cleanup the camera and close any open windows
#    camera.release()
#    cv2.destroyAllWindows()




#The core program begins here, once the variables have been initialized and the methods declared:

#Initialize the accelerometer
bus.write_byte_data(address, power_mgmt_1, 0)

#Background method: This method will be receiving the SPEED variable on an infinite loop, 
# waiting for instructions from the Network coordinator:
#When it receives the key code "11111", it will stop listening and respond with the requested data:
def background():
    
    while True:
        try:
            global speed
            global max_accel
#            global EntranceCounter
#            global ExitCounter
            global max_magnitude
            
            server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            port = 1
            server_sock.bind(("",port))
            server_sock.listen(1)
            client_sock,address = server_sock.accept()
            value = client_sock.recv(1024)
            extra = str(value, 'utf-8')
            speed =  int(extra)
            print ("received [%s]" % speed)
            time.sleep(0.2)
            
            if speed == 123456:
                 time.sleep(1.5)
                 client_sock.close()
                 server_sock.close()
                 passenger_info = [EntranceCounter,ExitCounter]
#                 print(type(EntranceCounter))
                 print(type(max_accel[0]))
                 information = max_accel + passenger_info
                 message = ','.join(str(e) for e in information)
                 print("Message")
                 sendMessageTo("B8:27:EB:E0:8A:EF", str(message)) #The structure of the message is: [Acc_X, Acc_Y, Acc_Z, Pass_In, Pass_Out]
                 speed = 2 #We reset the speed to activate all sensors again and restart the communication.
                 max_accel = 2
                 max_magnitude = 2
                 print("Message Sent")
#                 EntranceCounter = 0
#                 ExitCounter = 0
                 
        
        except bluetooth.btcommon.BluetoothError as error:
            print("Could not connect: ", error, "; Retrying in 5s...")
            time.sleep(5)

def foreground():
    global max_accel
#    global EntranceCounter
#    global ExitCounter
    global speed
    global max_magnitude
    
    while True:
     try:
#Check the accelerator values, only if the speed is above 2 km/hr:
#Monitoring mode:
        if speed >= 0:
#We compare the maximum acceleration magnitude to the one registered in the record.            

            (Ax,Ay,Az)=get_accelerometer(0,0,0)
            current_accel = [Ax,Ay,Az]
            print(max_magnitude)
            magnitude = np.linalg.norm(current_accel)
            
            if max_magnitude < magnitude:
                max_accel = current_accel
                max_magnitude = magnitude
  
            else:
                continue
            
            time.sleep(0.01)
 #Counting mode:           
#        if speed < 3:
#            PassengerCounter()
     except:
           max_accel = 0
           max_magnitude = 0

#We initialize the background and foreground methods as threads.

b = threading.Thread(name='background', target=background)
f = threading.Thread(name='foreground', target=foreground)


b.start()
f.start()
    
