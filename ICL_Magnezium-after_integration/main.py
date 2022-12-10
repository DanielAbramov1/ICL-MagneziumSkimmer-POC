## main.py
import serial
import time
import threading
import serial.tools.list_ports
import telnetlib

import MCU
import CognexCamera
from ComQueues import *
import SM_Sequence

ports = list(serial.tools.list_ports.comports())
print("Avaliable Port : " + ports[0][0])
ser = serial.Serial(ports[0][0], baudrate=115200, timeout=.1)

# ser = serial.Serial(port = 'COM4', baudrate=115200, timeout=.1)
mcu = MCU.SerialComms(ser)

HOST = '192.168.1.32' #Camera IP port 23
user = 'admin'
password = ''
tn = telnetlib.Telnet(HOST)
cam1 = CognexCamera.camFuncs(tn,HOST,user,password)


def mcuListener():
    
    while True:
        mcu.readSerial()
        #if theres data in the mcu queue write it to the mcu
        if not mcu_q.empty():
            msg = mcu_q.get() 
            print("messege from mcu_q " + msg)
            mcu.sendCmd(msg)

        time.sleep(0.000001)

def camListener():
    while True:
        cam1.readCamera()
        if not cam_q.empty():
            msg = cam_q.get() 
            print("messege from cam_q " + msg)
            cam1.sendCmd(cam_q.get())
        time.sleep(0.000001)
        
def camActivate():
    
     cam1.credentialCheck()
        
def manualPythonInput():
    
    while True:
        input_data = input()
        
        # # examples for commands
        # if input_data == '1':
        #     mcu.sendCmd("reboot")
        # elif input_data == '2':
        #     mcu.sendCmd("ping/pong") 
        # elif input_data == '3':
        #     mcu.sendCmd("busy") 
        # elif input_data == '4':
        #     mcu.sendCmd("done") 
            
        # elif input_data == '5':
        #     cam1.sendCmd("camAcquire") 
        # elif input_data == '6': 
        #     cam1.sendCmd("getT") 
        
        # elif input_data == '7':
        #     mcu.sendCmd("testV")
        # elif input_data == '8':
        #     mcu.sendCmd("testA")
        # elif input_data == '9':
        #     mcu.sendCmd("testS") #testing state input
        
        if input_data[0] == '$':
            print(f"sm input {input_data[1:]}")
            sm_q.put(input_data[1:])
            
        elif input_data[0] == '#':
            print(f"mcu input {input_data[1:]}")
            mcu_q.put(input_data[1:])
            
        elif input_data[0] == '@':
            print(f"cam input {input_data[1:]}")
            cam_q.put(input_data[1:])
        
        time.sleep(0.000001)

def main():
    
    t1 = threading.Thread(target=mcuListener,daemon=True)
    t1.start()
    
    camActivate()
    t2 = threading.Thread(target=camListener,daemon=True)
    t2.start()
    
    # t3 = threading.Thread(target=manualPythonInput,daemon=True)
    # t3.start()
         
    t4 = threading.Thread(target=SM_Sequence.robodk_loop,daemon=True)
    t4.start()
    print("Started")
     
    while True:
        time.sleep(0.00001)

if __name__ == '__main__':
    main()
    