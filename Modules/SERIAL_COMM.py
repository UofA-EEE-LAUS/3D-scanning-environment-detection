import serial
import sys
dt = 1/80


ser = serial.Serial(port='/dev/ttyACM0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=dt/10)

def serial_comms(payload):
     if:
     ser.write(payload.encode())
     myData = ser.readline()
     myData = myData.decode("utf-8","ignore")
     print (myData)
     time.sleep(dt)
     Transfer = 1
     except
     Transfer = 0
return Transfer

def Formatting(inp, maxlength):  #Both inputs are numbers
     # print("Attempting Formatting")
     #This function checks if a numbers polarity, and outputs a string of maxLength with the correct polarity
     polarity = "-"
     if inp > 0:
        polarity = "+"
        
     tempVal = abs(inp)
     outp = str(tempVal)

#This functions pads zeros to the start of inp till it reaches maxlength
     if len(str(tempVal)) < maxlength:
        for i in range(maxlength - len(str(tempVal))):
            outp = "0" + outp
            
     return polarity + outp
     
def maker(RPY, sonar, lidar):
    payload = Formatting(round(RPY[0],4),8) + "/" + Formatting(round(RPY[1],4),8) + "/" + Formatting(round(RPY[2],4),8) + "/" + Formatting(Lidar_data,6) + "/" + Formatting(Sonar_data,6)
return payload