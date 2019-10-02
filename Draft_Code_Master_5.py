#The code to connect to AWS is based on the AWSDeviceSDK provided by AWS IoT found on: 
#https://docs.aws.amazon.com/iot/latest/developerguide/iot-gs.html
#Accessed first on July 10, 2019

#The code to Update the GPS status was adapted the NB-IoT SIM 7000E documentation found on:
#https://www.waveshare.com/wiki/File:SIM7000X-Demo.7z
#Accessed first on July 13, 2019

#The code to use the PiCamera to count passengers was adapted from the work of Pedro Henrique Fonseca Bertoleti, found on:
#https://www.hackster.io/phfbertoleti/counting-objects-in-movement-using-raspberry-pi-opencv-015ba5
#Accessed first on July 20, 2019

#The methods to connect to Bluetooth are based on the PyBluez library found on:
#https://people.csail.mit.edu/albert/bluez-intro/x232.html
#Accessed first on July 25, 2019

#All the cited code has been adapted, debugged and mixed by Juan Carlos Fernandez to meet the specific requirements of the transport network.

#Library to run several concurrent threads
import threading
from threading import Timer

#Libraries to connect to AWS 
import os
import sys
import AWSIoTPythonSDK
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTShadowClient
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
sys.path.insert(0, os.path.dirname(AWSIoTPythonSDK.__file__))

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

#Libraries to connect the RP0 to the sensors and the GPS 
import RPi.GPIO as GPIO
import serial
import random, time, datetime
import unicodedata


#NB-IoT: Method to connect to the WAN with the Wireless Hat. The default network is the NB-IoT, and the fallback is GPRS.**********************
#***********************************************************************INSERT METHOD****************************

#GPS: We initialize the mini UART port for the GPS to operate with the NB-IoT hat:
ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()
power_key = 4
rec_buff = ''
rec_buff2 = ''
time_count = 0
temporal = '' #This variable will temporarily store the response of rec_buff

#GPS: We declare the variables that will be extracted from the "AT+CGNSINF" response. They will be global variables:
latitude = 0;
longitude = 0;
speed = 0;
satelites = 0;

#AWS: Get the date object from datetime and initialize log
dateTimeObj = datetime.datetime.now()
DateObj = dateTimeObj     #In this case, we also use the timestamp.
log = ''
logcount = 1

#Google Maps API Key for Road Service:
maps_key = "AIzaSyD8gho9r2Lu8D6inBsqmJS6g7oUnL_4vus"

#AWS: A random programmatic shadow and client ID.
SHADOW_CLIENT = "myShadowClient"
IOT_CLIENT = "new_Client"
SHADOW_HANDLER = "RP0_Coordinator"

# AWS: The unique hostname generated for the network master, and the corresponding certificates. Must match the ones in the platform:
HOST_NAME = ""#This section must match the keys and certificates provided by the AWS platform.
ROOT_CA = ""
PRIVATE_KEY = ""
CERT_FILE = ""

# AWS: Create, configure, and connect the shadow client for the AWS Platform.
myShadowClient = AWSIoTMQTTShadowClient(SHADOW_CLIENT)
myShadowClient.configureEndpoint(HOST_NAME, 8883)
myShadowClient.configureCredentials(ROOT_CA, PRIVATE_KEY, CERT_FILE)
myShadowClient.configureConnectDisconnectTimeout(10)
myShadowClient.configureMQTTOperationTimeout(5)
#myShadowClient.connect()

# AWS: Create, configure, and connect the MQTT client for the AWS Platform.
myMQTTClient = AWSIoTMQTTClient(IOT_CLIENT)
myMQTTClient.configureEndpoint(HOST_NAME, 8883)
myMQTTClient.configureCredentials(ROOT_CA, PRIVATE_KEY, CERT_FILE)
myMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
myMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
myMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
myMQTTClient.configureMQTTOperationTimeout(5)
#myMQTTClient.connect()

# AWS: Create a programmatic representation of the shadow and the topic:
myDeviceShadow = myShadowClient.createShadowHandlerWithName(SHADOW_HANDLER, True)
mytopic = "" #This topic must match the MQTT topic in AWS

#PC (Passenger counter): Declare the global variables for the video frame and passenger counter:
width = 0
height = 0
EntranceCounter = 0;
ExitCounter = 0;
MinCountourArea = 7000  #Adjust ths value according to the size of the monitored space.
BinarizationThreshold =55  #Adjust ths value according to to lightning conditions
OffsetRefLines = 200  #Adjust ths value according to the camera characteristics

#PC: We initialize the PiCamera and discard some frames while it adapts to the lightning:
camera = VideoStream(usePiCamera= True).start()
time.sleep(2.0)
ReferenceFrame = None;

for i in range(0,20):
    (grabbed, Frame) = camera.read(), camera.read()

#Bluetooth: Declare the MAC addresses of all devices in the network:
MACAddress_Master = "B8:27:EB:E0:8A:EF"
MACAddress_Slave ="B8:27:EB:BB:85:7B"


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
    
#AWS: This method is called whenever the shadow is updated.
def myShadowUpdateCallback(payload, responseStatus, token):
#  print()
#  print('UPDATE: $aws/things/' + SHADOW_HANDLER + 
#    '/shadow/update/#')
#  print("payload = " + payload)
#  print("responseStatus = " + responseStatus)
#  print("token = " + token)
  print("Message sent to topic")

#GPS: This method is called to request the AT+CGNSINF status.
def send_at(command,back,timeout):
	global latitude
	global longitude
	global speed
	global satelites

	rec_buff = ''
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	if ser.inWaiting():
		time.sleep(0.01 )
		rec_buff = ser.read(ser.inWaiting())
	if rec_buff != '':
		if back not in rec_buff.decode():
			print(command + 'ERROR')
			print(command + ' back:\t' + rec_buff.decode())
			return 0
		else:
			#We convert from unicode to string to split the AT+CGNSINF response, and extract the relevant parameters
			temporal = rec_buff.decode()
#			temporal = temporal.encode('ascii','ignore')
			temporal = temporal.rsplit(',')
			print(temporal)
#			print(type(temporal))
			print(len(temporal))

			if len(temporal) <=7:
#				print(rec_buff.decode())    #Only print for testing
				return 1
			else:
				temporal.pop(20)		#Drop the String values
				temporal.pop(0)
				list = []
				for a in temporal:
					if a=='':
						n=0
					else:
						n=float(a)
					list.append(n) 
				latitude = list[2]  #The position number 2 of the AT+CGNSINF response is the latitude.
				print(list)
				longitude = list[3] #The position number 3 of the AT+CGNSINF response is the longitude.
				speed =list[5]     #The position number 5 of the AT+CGNSINF response is the speed.
#				print(speed)
				print(type(speed))
				print(rec_buff.decode())    #Only print for testing
				return 1
	else:
		print('GPS is not ready')
		return 0

#GPS and AWS: This method is used to update the GPS position and then, send the results to AWS.
def get_gps_position():
	rec_null = True
	answer = 0
	print('Start GPS session...')
	rec_buff = ''
	send_at('AT+CGNSPWR=1','OK',1) 
	time.sleep(2)
	while rec_null:
		answer = send_at('AT+CGNSINF','+CGNSINF: ',1)
		if 1 == answer:     
			answer = 0           
			if ',,,,,,' in rec_buff:
				print('GPS is not ready')
				rec_null = False
				time.sleep(1)
				return False
			else: 
				return True              
                
		else:
			print('error %d'%answer)
			rec_buff = ''
			send_at('AT+CGPS=0','OK',1)
			return False
		time.sleep(1.5)

#GPS: This method is used to initialize the GPS.
def power_on(power_key):
	print('SIM7000X	is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(power_key,GPIO.OUT)
	time.sleep(0.1)
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(2)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	ser.flushInput()
	print('SIM7000X is ready')

#GPS: This method is used to shut down the GPS and clear the GPIO.
def power_down(power_key):
	print('SIM7600X is loging off:')
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(3)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	print('Good bye')
    
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
def PassengerCounter():

#We declare the person counter as global variables       
    global EntranceCounter
    global ExitCounter
    global ReferenceFrame

#PC: Read the video stream from the PiCamera:
    (grabbed, Frame_orig) = camera.read(),camera.read()
    Frame = imutils.resize(Frame_orig, width=600)
    height = np.size(Frame,0)
    width = np.size(Frame,1)

#PC: This functions are used to apply gray-scale convertion and Gaussian blur filter
    GrayFrame = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
    GrayFrame = cv2.GaussianBlur(GrayFrame, (21, 21), 0)

    if ReferenceFrame is None:
        ReferenceFrame = GrayFrame
#        continue

#PC: Background subtraction and image binarization, compared to the Reference Frame
    FrameDelta = cv2.absdiff(ReferenceFrame, GrayFrame)
    FrameThresh = cv2.threshold(FrameDelta, BinarizationThreshold, 255, cv2.THRESH_BINARY)[1]

#PC: Dilate image and find all the contours
    FrameThresh = cv2.dilate(FrameThresh, None, iterations=2)
    _, cnts, _ = cv2.findContours(FrameThresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    QttyOfContours = 0

#PC: We define the reference lines (entrance and exit) and convert them to Integers for line function
    CoorXEntranceLine = (width / 2)-OffsetRefLines
    CoorXExitLine = (width / 2)+OffsetRefLines
    CoorXEntranceLine = int(CoorXEntranceLine)
    CoorXExitLine = int(CoorXExitLine)

    cv2.line(Frame, (CoorXEntranceLine,0), (CoorXEntranceLine,height), (255, 0, 0), 2)
    cv2.line(Frame, (CoorXExitLine,0), (CoorXExitLine,height), (0, 0, 255), 2)


    #check all found countours
    for c in cnts:
        #if a contour has small area, it'll be ignored
        if cv2.contourArea(c) < MinCountourArea:
            continue

        QttyOfContours = QttyOfContours+1

        #draw an rectangle "around" the object
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(Frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #find object's centroid
        CoordXCentroid = (x+x+w)/2
        CoordYCentroid = (y+y+h)/2
        CoordXCentroid = int(CoordXCentroid)
        CoordYCentroid = int(CoordYCentroid)

        ObjectCentroid = (CoordXCentroid,CoordYCentroid)
        cv2.circle(Frame, ObjectCentroid, 1, (0, 0, 0), 5)
        if (CheckEntranceLineCrossing(CoordXCentroid,CoorXEntranceLine,CoorXExitLine)):
          EntranceCounter += 1

        if (CheckExitLineCrossing(CoordXCentroid,CoorXEntranceLine,CoorXExitLine)):
          ExitCounter += 1

#    print( "Total countours found: " + str(QttyOfContours))

#PC: ONLY FOR TESTING: Write entrance and exit counter values on frame and shows it
#   cv2.putText(Frame, "Entrances: {}".format(str(EntranceCounter)), (10, 50),
#                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 0, 1), 2)
#    cv2.putText(Frame, "Exits: {}".format(str(ExitCounter)), (10, 70),
#                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#    cv2.imshow("Original Frame", Frame)
#    cv2.waitKey(1);

#print("Fin del programa")
# cleanup the camera and close any open windows
#camera.release()
#cv2.destroyAllWindows()

#The core program begins here, once the variables have been initialized and the methods declared:

#Initialize the GPS
power_on(power_key)
gps_counter = get_gps_position()

#for i in range(1,8):
#    print("GPS not ready")
#    gps_counter = get_gps_position()    
#    time.sleep(10)


#Internal communication method: To get the variables from the slave through a Bluetooth Message:
def GetSlave():
    global EntranceCounter
    global ExitCounter
    global latitude
    global longitude
    global speed
    PassInSlave = 0
    PassOutSlave = 0

    try:
        #Connect with the slave and get the data: TO ACTIVATE ON PILOT SECOND PHASE**********************************************
            sendMessageTo(MACAddress_Slave, "123456")
            time.sleep(0.5)

        #Open the socket to receive the information from the slave and transform it from bytes to integers.
            data = receiveMessages()
            data_string = str(data,'utf-8')
            data_string = data_string.rsplit(',')
            (A,B,C,D,E) = data_string[0], data_string[1], data_string[2], data_string[3], data_string[4]
            (max_accel_x, max_accel_y, max_accel_z, PassInSlave, PassOutSlave) = float(A),float(B),float(C), int(D), int(E)
            print(data_string)

        #Get the total number of passengers In and Out
            TotalPassIn = EntranceCounter  + PassInSlave
            TotalPassOut = ExitCounter + PassOutSlave
            EntranceCounter = 0
            ExitCounter = 0

            print ("Bluetooth received")
            return(max_accel_x, max_accel_y, max_accel_z, PassInSlave, PassOutSlave)


    except:
            print("Error in bluetooth")

            return(0,0,0,EntranceCounter,ExitCounter)
            EntranceCounter = 0
            ExitCounter = 0



#Background method: This method will be monitoring the GPS on an infinite loop. Every time the bus stops, 
# it will send instructions to the Slave to switch from counting mode to monitoring mode:
#When it sends the key code "11111", it will start listening and receive the variables from the slave:
def background():
    global speed
    speed_minus = 0 #This variable will allow us to store the previous value of speed.
    logic_speed = 0 #This is our logic switch, 0 if speed is below 3 and 1 if speed is above 1.
    message = ''
    while True:
        try:
            get_gps_position()

#            if speed > 3:
#                logic_speed = 1
#            else: 
#                logic_speed = 0

#            if logic_speed != speed_minus:
#               message = str(speed)
#                sendMessageTo(MACAddress_Slave, message)

#            speed_minus = logic_speed
            time.sleep(10)

        except bluetooth.btcommon.BluetoothError as error:
            print("Could not connect: ", error, "; Retrying in 2s...")
            time.sleep(4)

def foreground():
    global EntranceCounter
    global ExitCounter
    global speed

    while True:
#Check the accelerator values, only if the speed is above 2 km/hr:

#Counting mode:
        if speed < 3:
            PassengerCounter()
#            print("Counting passengers")
        else:
            continue


def sendDatatoAWS():
    global latitude
    global longitude
    global speed
    global log
    global logcount
    myShadowClient.connect()
    myMQTTClient.connect()


    while True:
        try:
            print("Sending message to AWS")
            timeObj = datetime.datetime.now()
            log = "London/25/"+str(logcount)
            logcount += 1
            (max_accel_x, max_accel_y, max_accel_z, TotalPassIn, TotalPassOut) = GetSlave()

        #Set the message for AWS including all the monitored variables:

            Info_to_shadow = '{"state":{"reported":{"Latitude":"' + str(latitude) +'","Longitude":"' + str(longitude) +'","Speed":"' + str(speed) +'","PassIn":"' + str(TotalPassIn) +'","PassOut":"' + str(TotalPassOut) +'","AccX":"' +str( max_accel_x) +'","AccY":"' + str(max_accel_z) +'","AccZ":"' + str(max_accel_z)+ '"}}}'
            DatatoTopic = '{"log":"' + str(log) +'","Date":"' + str(timeObj) +'","Latitude":"' + str(latitude) +'","Longitude":"' + str(longitude) +'","Speed":"'+ str(speed) +'","PassIn":"' + str(TotalPassIn) +'","PassOut":"' + str(TotalPassOut) +'","AccX":"' + str(max_accel_x) +'","AccY":"' + str(max_accel_y) +'","AccZ":"' + str(max_accel_z)+ '"}'

            myMQTTClient.publish(mytopic, DatatoTopic, 0)
            myDeviceShadow.shadowUpdate(Info_to_shadow, myShadowUpdateCallback, 5)

         #We reset the variables. If there was a mistake by sending them, we store them for the next message:
            EntranceCounter = 0
            ExitCounter = 0
            max_accel_x = 0
            max_accel_y = 0
            max_accel_z = 0

            print ("Message Sent")
            time.sleep(15)

        except:
            print("Error in sending the message")
            myShadowClient.connect()
            myMQTTClient.connect()
            time.sleep(15)

 
#We initialize the background and foreground methods as threads.

b = threading.Thread(name='background', target=background)
f = threading.Thread(name='foreground', target=foreground)
a = threading.Thread(name='sendDatatoAWS', target=sendDatatoAWS)


b.start()
f.start()
a.start()
