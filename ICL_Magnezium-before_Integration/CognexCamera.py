from asyncore import write
import time
import sys
# import telnetlib
from telnetlib import *
from ComQueues import *

camDict = {
    #key : value
    "camAcquire": 'SW8', #Acquire an image and update the spreadsheet. This option requires the AcquireImage function's Trigger parameter to be set to External, Manual or Network. For more information, see AcquireImage.
    "getT": 'EV GetSystemConfig("Internal.Temperature")',
    "getX": 'gvc028',
    "getY": 'gvd028'
}

class camFuncs:
      
    def __init__(self,tn,HOST,user,password):
        self.tn = tn
        self.HOST = HOST
        self.user = user
        self.password = password 
            
    def credentialCheck(self):
        self.tn.read_until(b'User: ',5)
        self.tn.write(self.user.encode('ascii') + b'\r\n') #the user name is admin
        self.tn.read_until(b'Password: ',5)
        self.tn.write(self.password.encode('ascii') + b'\r\n') #there is no password - just return - now logged in
        tmp = self.tn.read_until(b'\r\n',5).decode('utf-8')
        if(tmp != 'User Logged In\r\n' ): # If OK prints 'User Logged In'
            print('Login Error!')
        else:
            print('User logged in successfully!')
    
    """from cam"""        
    def readCamera(self):
        data_from_cam = self.tn.read_until(b'\r\n',1).decode('utf-8')
        if(data_from_cam):
            data_from_cam = (self.tn.read_until(b'\r\n',1).decode('utf-8'))
            print("Cam data received: " + data_from_cam)
            sm_q.put(data_from_cam)
            
    """to mcu"""       
    def writeCamera(self,data_to_camera):
        
        self.tn.write((data_to_camera + '\r\n').encode())
        
    """read mcu_q and send cmnds to mcu via serial"""
    def read_from_q(self):
        
        if(cam_q.empty() == False):
            self.tn.write((cam_q.get()).encode())
    
    """send manual cmnds to mcu via dictionary"""
    def sendCmd(self, cmd_key):
        
        self.writeCamera(camDict.get(cmd_key))