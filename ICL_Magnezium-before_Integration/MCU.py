from queue import Queue
import serial
from ComQueues import *

# examples for commands
CommandsDict = {
    #key : value
    "reboot": "-", 
    "ping/pong": "p", 
    "busy": "b", 
    "done": "d", 
    "testV": "<V12.3>",
    "testA": "<A22.2>",
    "testS": "<S2>",

    "Nati command": "asd123,123"
}   

class SerialComms:
    
    def __init__(self,ser):
        self.ser = ser
    
    """from mcu""" 
    def readSerial(self):
        
        if self.ser.in_waiting:
            data_from_serial = self.ser.readline().decode('utf-8')
            data_from_serial = data_from_serial.strip('>\r\n').replace('<', '')
            print("MCU data received: " + data_from_serial)
            sm_q.put(data_from_serial)
              
    """to mcu"""   
    def writeSerial(self,data_to_serial):
        
        self.ser.write((data_to_serial + "\n").encode())
           
    """read mcu_q and send cmnds to mcu via serial"""
    def read_from_q(self):
        
        if(mcu_q.empty() == False):
            self.writeSerial((mcu_q.get()).encode())
        
    """send manual cmnds to mcu via dictionary"""       
    def sendCmd(self, cmd_key):
        
        self.writeSerial(CommandsDict.get(cmd_key))
        
    
    
    

