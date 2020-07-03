#!/usr/bin/env python3
# (c) Vaclav Jirovsky, 2020
# Licence (based on regulation for PRUSA face shields)
#  We share these files under non-commercial licence. 
#  It would be great if you donated this device to those in need for free. 
#  If you need to cover your production costs, we are ok with you selling the device  for production cost. 
#  However, we do not want to see these devices on eBay for 1000€.
import time

import serial
import RPi.GPIO as GPIO
from var_dump import var_dump
from multiprocessing import Process, Value
import queue
import multiprocessing
import datetime
import sys
import joblib
import numpy as np

from azure.iot.device import IoTHubDeviceClient, Message

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_BUTTON = 25

IOT_HUB_CONNECTION_STRING = "YOUR IOT HUB CONNECTION STRING"

debug=0



isFirstRun=True
state = None

Q = multiprocessing.Queue()

ser = serial.Serial("/dev/ttyS0", 115200)
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_BUTTON, GPIO.OUT)
 
def distance():
   # based on code from https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def listen_uart_rx(state, IOT_HUB_CONNECTION_STRING):

   while True:
      #print("State in listen is %d" % state.value)
     
      received_data = ser.read()
      time.sleep(0.03)
      data_left = ser.inWaiting()
      received_data += ser.read(data_left)
      received_data = received_data.decode('utf-8')
      #var_dump(received_data)
      
      if (received_data.find("W (0) MAIN:") > -1):
         with state.get_lock():
            state.value = 1
         print("detected intro\n")
         #detected intro from serial line
         continue

      if (received_data.find("W (121) NSA:") > -1):
         with state.get_lock():
            state.value = 2
         #detected intro #2
         continue

      if (received_data.find("DIS: invalid t 0") > -1):
         with state.get_lock():
            state.value = 0
         #thermometer sends sleep signal
         print("Thermometer went to sleep!\n")
         break

      if (state.value == 2):
         #parse values XXXX XXXX \n XXXX XXXX
         lines = received_data.split('\n')
         room_temps = [int(i) for i in lines[0].strip().split(' ')]
         body_temps = [int(i) for i in lines[1].strip().split(' ')]

         #send signal to main thread
         Q.put([room_temps, body_temps])
         break


if __name__ == '__main__':
   try:
      state = Value('i', 0)
      myIotHubClient = IoTHubDeviceClient.create_from_connection_string(IOT_HUB_CONNECTION_STRING)
      myIotHubClient.connect()
      predicter = joblib.load('./mymodel.joblib')

      while True:
         if (debug):
            time.sleep(2)
         time.sleep(0.10)
         dist = distance()
         #print ("Measured Distance = %.1f cm" % dist)
         
         if (dist > 10):
            continue
         
         print ("Simulating button...")

         GPIO.output(GPIO_BUTTON, GPIO.HIGH)
         time.sleep(0.1)
         GPIO.output(GPIO_BUTTON, GPIO.LOW)

         #create an child process for recieve UART
         action_process = Process(target=listen_uart_rx, args=(state,IOT_HUB_CONNECTION_STRING,))
      
         action_process.start()
         action_process.join(timeout=1.0)
         action_process.terminate()

         #wait for signal from child thread
         try:
            lastValue = Q.get(timeout=1.0)
         except queue.Empty:
            print("Queue were empty, measurement probably timeouted!")
            with state.get_lock():
               state.value = 0
            continue

         room_temps = lastValue[0]
         body_temps = lastValue[1]
         
         #calculate avg temps
         avg_room_temp = round(sum(room_temps) /float(len(room_temps))/100, 2)
         avg_body_temp = round(sum(body_temps) /float(len(body_temps))/100, 2)
         
         merged_temps = room_temps + body_temps

         datetime_object = datetime.datetime.now()

         result_body_temp = predicter.predict(np.array(merged_temps).reshape(1,-1))

         print("%s:%s:%s - Room avg. temp: %f; Body avg. temp: %f; Predicted result body temp: %f" %(datetime_object.hour,datetime_object.minute,datetime_object.second,avg_room_temp,avg_body_temp, result_body_temp))

         if(debug):
            #write down measured value for debug purposes
            with open("measurements.txt", "a") as measurementsFile:
               measurementsFile.write("%s:%s:%s" %(datetime_object.hour,datetime_object.minute,datetime_object.second) +"\t")
               for val in room_temps:
                  measurementsFile.write( str(val) + "\t" )
               for val in body_temps:
                  measurementsFile.write( str(val) + "\t" )

               measurementsFile.write( str(result_body_temp) + "\t" )
               measurementsFile.write( "\n" )
         else:
            #send data to IoT Hub
            try:
               iot_hub_payload_template = '{{"bodyTemp": {result_body_temp}}}'

               iot_hub_payload = iot_hub_payload_template.format(result_body_temp = result_body_temp)
               iot_hub_message = Message(iot_hub_payload)

               if result_body_temp > 38.0:
                  iot_hub_message.custom_properties["temperatureAlert"] = "true"
               else:
                  iot_hub_message.custom_properties["temperatureAlert"] = "false"

               print("[IoT Hub] Sending message to IoT Hub: {}".format(iot_hub_message) )
               
               myIotHubClient.send_message(iot_hub_message)
               print("[IoT Hub] Message successfully sent!")
            except:
               print("[IoT Hub] Unexpected error:", sys.exc_info()[0])
               raise

         time.sleep(0.8)

      # Reset by pressing CTRL + C
   except KeyboardInterrupt:
      print("Stopped by user!")
      GPIO.cleanup()
      myIotHubClient.disconnect()
