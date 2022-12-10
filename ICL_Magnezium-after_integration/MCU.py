from queue import Queue
import serial
from ComQueues import *


# examples for commands
CommandsDict = {
    #key : value
    "reboot": "-", 
    "ping": "p", 
    "not busy": "b", 
    "done": "d",
    "#ERR" : "#ERR" 
    
    # "testV": "<V12.3>",
    # "testA": "<A22.2>",
    # "testS": "<S2>",
    
}   

class SerialComms:
    
    def __init__(self,ser):
        self.ser = ser
    
    """from mcu""" 
    def readSerial(self):
        
        if self.ser.in_waiting:
            data_from_serial = self.ser.readline().decode('utf-8')
            if(data_from_serial[0] == '<'):
                data_from_serial = data_from_serial.strip('>\r\n').replace('<', '')
                if not (data_from_serial[0] == 'P'):
                    print("Data recieved from MCU: " + data_from_serial)
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
        
        #todo for integration add the if statement
        if(cmd_key[0] == 'c'): 
            print("data from camera to mcu : " + cmd_key)
            self.writeSerial(cmd_key)
        else:
            self.writeSerial(CommandsDict.get(cmd_key))
        
    
    def flush_all(self):
        bytes = self.ser.in_waiting
        if bytes > 0:
            self.ser.read(bytes)
            
    

