import enum
from ComQueues import *

stateDict = {
    #key : value
    'CASE0': 0,
    'CASE1': 1,
    'CASE2': 2,
    'CASE3': 3,
}   

def get_key(val):
    
    for key, value in stateDict.items():
        if val == value:
            return key

class SM:
    state = 0 #sm_states()
    
    def __init__(self,state,v,a):
        
        self.state = state
        self.v = v
        self.a = a
        
    def set_sm_state(self,st):
        
        self.state = st
        print(f"State Machine is : {get_key(self.state)}")
    
    def get_sm_state(self):
        
        print(f"State Machine is : {get_key(self.state)}")
        return self.state
    
    def set_vel(self,v):
        
        self.v = v
        print(f"Velocity is set to : {self.v}")
    
    def get_vel(self):
        print(f"Velocity is set to : {self.v}")
        return self.v
        
    def set_acc(self,a):
        
        self.a = a
        print(f"Acceleration is set to : {self.a}")
        
    def get_acc(self):
        
        print(f"Acceleration is set to : {self.a}")
        return self.a
       
    #read from sm_q and set sm values
    def read_from_Q(self):
        
        if not sm_q.empty():
            # print(f"data in queue: {sm_q.queue[0]}")
            cmnd = sm_q.get()
            print(cmnd)
            if (cmnd[0]== 'V'):
                vel = float(cmnd[1:]) 
                self.set_vel(vel)
                return None

            elif (cmnd[0] == 'A'):
                acc = float(cmnd[1:]) 
                self.set_acc(acc)      
                return None
            
            #for machine state change if mcu/cam sent a critical command 
            elif (cmnd[0] == 'S'):
                st = cmnd[1:] 
                return st
            
            elif (cmnd[0] == 'X'):
                x_pos = cmnd[1:] 
                pass
            
            elif (cmnd[0] == 'Y'):
                y_pos = cmnd[1:] 
                pass
            
            else:
                print("no such command")
                return None
            
        else:
            return None
    
    def write_to_mcuQ(self,val):
        
        mcu_q.put(val)
      
    def sm_loop(self):
        
        input_state = self.read_from_Q()
        if input_state is None:
            return
        
        if (self.state == stateDict['CASE0']):
            if(input_state == '1'):
                self.set_sm_state(1)
                           
        elif (self.state == stateDict['CASE1']):
            if(input_state == '2'):
                self.set_sm_state(2)
            else:
                print("Wrong")
                self.set_sm_state(0)  
                  
        elif (self.state == stateDict['CASE2']):
            if(input_state == '3'):
                self.set_sm_state(3)
            else:
                print("Wrong")
                self.set_sm_state(0) 
                
        elif (self.state == stateDict['CASE3']):
            if(input_state == '4'):
                print("Approved!")
                self.set_sm_state(0)
            else:
                print("Wrong")
                self.set_sm_state(0)  
        

