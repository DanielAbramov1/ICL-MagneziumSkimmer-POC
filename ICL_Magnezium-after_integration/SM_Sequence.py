from calendar import c
from distutils.util import convert_path
from time import sleep
from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
# The following 2 lines keep your code compatible with older versions or RoboDK:
from robodk.robolink import *      # RoboDK API
from robodk.robomath import *      # Robot toolbox
from robolink import *    # Robot toolbox

from ComQueues import *

from robodk.robodialogs import *
from robodk.robofileio import *

# Link to RoboDK
RDK = robolink.Robolink()

#Define Test setup
# runinng_method = RUNMODE_SIMULATE # RUNMODE_SIMULATE - simulation, RUNMODE_RUN_ROBOT - real movement on robot
runinng_method = RUNMODE_RUN_ROBOT # RUNMODE_SIMULATE - simulation, RUNMODE_RUN_ROBOT - real movement on robot
withMCU = True

#Define default skimming length
APPROACH = 150 # define default approach distance
SKIMM_LEN = 590
SKIMM_DEPTH = 25
X_IDLE_APP = 55 # Estimator for latency

CONV_HOME = 1931.0 # 1931.0 is calibrated

CONV_SPEED = 27 #mm/s
ROBOT_SPEED = 1000 #mm/s
ROBOT_ACCELERATION = 4000 # mm/s^2
ROBOT_CONV_ACCELERATION = 48.6 # mm/s^2

#global variable to store velocity, acceleration and x positoin
state = -1
last_state = -1
StartConv = False

vel = CONV_SPEED
acc = ROBOT_CONV_ACCELERATION
xpos = 999.9
conv_pos = 0

X_ShiftAPP = 0.0
X_ShiftLEN = 0.0

# Set up station if not already on
station               = RDK.Item('Temp', ITEM_TYPE_STATION)
if not station.Valid():
    RDK.AddFile(r'G:\Shared drives\KAUFFMAN-TEAM\kaufman-rd\solid\102 - ICL\01 - Magnesium Ingot Scimming\RoboDK\IntegratedMCU_21.7_presentation\Temp.rdk')

robot               = RDK.Item('UR10e', ITEM_TYPE_ROBOT)
# Check for robot in RoboDK program
if not robot.Valid():
    RDK.ShowMessage("UR10e robot not found.")
    # quit()

ConvF               = RDK.Item('ConvF', ITEM_TYPE_ROBOT)
# Reset simulated conveyor
if ConvF.Valid():
    ConvF.setJoints([CONV_HOME])

# Gather tool and reference frames from the station
tool                = RDK.Item('skimmer_ITEM', ITEM_TYPE_TOOL)
frame_MCS           = RDK.Item('MCS Origin', ITEM_TYPE_FRAME)
frame_CBO           = RDK.Item('CBO', ITEM_TYPE_FRAME)
frame_PCS           = RDK.Item('PCS', ITEM_TYPE_FRAME)
frame_box           = RDK.Item('Frame_Box', ITEM_TYPE_FRAME)

# Gather targets
target_home_safe = RDK.Item('Home', ITEM_TYPE_TARGET)
target_Ready = RDK.Item('READY_J', ITEM_TYPE_TARGET)
target_FarGoal = RDK.Item('FarGoal', ITEM_TYPE_TARGET)
# target_PCS = RDK.Item('PCS', ITEM_TYPE_TARGET)

target_PCS_Origin = RDK.Item('PCS_Origin', ITEM_TYPE_TARGET)
target_conv_far = RDK.Item('ConveyorMove', ITEM_TYPE_TARGET)

target_Trash_up = RDK.Item('Trash_up', ITEM_TYPE_TARGET)
target_trash_turn = RDK.Item('trash_turn', ITEM_TYPE_TARGET)
target_Trash_dump = RDK.Item('Trash_dump', ITEM_TYPE_TARGET)

#Set Referances
robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
robot.setTool(tool)

RDK.setRunMode(runinng_method) # Change between simulation and actual movement

def CalcDXoffset (length, conv_speed, robot_speed, robot_acceleration):   
    # Calculates offset due to acceleration & decceleration 
    if (((robot_speed * robot_speed) / robot_acceleration) >= length):
        t = 2 * sqrt(length / robot_acceleration)
    else:
        t = (length / robot_speed) + (robot_speed / robot_acceleration)
    
    delta = conv_speed * t
    return delta

def ConnectToRobot(): 
    """ Connects to specific ip and updates simulated pos """
    # Connection to REAL UR10e robot
    robot_ip = '192.168.1.219'
    if robot.ConnectSafe(robot_ip) == ROBOTCOM_READY:
        RDK.ShowMessage("Connected to robot: " + robot.Name(), False)            
    else:
        RDK.ShowMessage("Unable to connect to robot: " + robot.Name(), False)
        
    # Retrieve the position from the robot (automatically updates the robot in RoboDK)
    robot.Joints()

def read_from_sm_q():  
        global state,vel,acc,xpos,conv_pos
        
        if not sm_q.empty():
            # print(f"data in queue: {sm_q.queue[0]}")
            cmnd = sm_q.get()
            print(f"read from sm_q: {cmnd}")
            
            if (cmnd[0] == 'S'):
                state = int(cmnd[1:]) 
            
            elif (cmnd[0]== 'V'):
                vel = float(cmnd[1:]) 

            elif (cmnd[0] == 'A'):
                acc = float(cmnd[1:])     
            
            elif (cmnd[0] == 'C'):
                xpos = float(cmnd[1:])

            elif (cmnd[0] == 'P'):
                conv_pos = float(cmnd[1:])    
            
            elif (cmnd[0] == '#'):
                mcu_q.put(cmnd)

            else:
                print("no such command")    
                
        return
        
def Update_Conveyor():
    """Update Conveyor position according to MCU Tracking. If running a simulation, to init movement at constant predefined speed"""
    global StartConv,CONV_SPEED,ConvF,conv_pos
    
    if (withMCU == True):
        if not (CONV_SPEED == vel): # Update conveyor speed
            CONV_SPEED = vel
        ConvF.setJoints([conv_pos+CONV_HOME]) # Update conveyor position
    elif(StartConv == False):#Simulate Conveyor
        ConvF.setSpeed(CONV_SPEED,CONV_SPEED,CONV_SPEED,CONV_SPEED) #to be changed to real live values
        ConvF.MoveL(target_conv_far,False)
        StartConv = True
    else:
        conv_pos = ConvF.Joints()[0,0] - CONV_HOME # Update from conveyor simulation X position

def SkimmingSM():
    """Skimming State-Machine""" 
    global IOP_X,xpos,CONV_HOME,last_state,state

    if (state == 0): # CAM_CheckUpdate 
        if (IOP_X == 999.9): # For as long as 
            cam_q.put('getX') # Send command to camera, to pick up X position value
            sleep(0.1) # Does this interfere with flow? Should this be just a counter
            # cam_q.put('getX')
            # sleep(0.1) # Does this interfere with flow? Should this be just a counter
            IOP_X = xpos
            if (withMCU == False): # Update, only for simulation
                ConvF.setJoints([IOP_X+CONV_HOME])
        else:
            msg = 'c' + str(IOP_X)
            print("data to mcu_q : " + msg)
            mcu_q.put(msg) #Send IOP_X to MCU
            IOP_X = 999.9 #reset default value
            last_state = state

            
    elif (state == 1): #Begin Sequence - Go to Ready position and notify MCU
        robot.setPoseFrame(frame_MCS)
        robot.MoveL(target_Ready,False) #Go and wait
        last_state = state
        # mcu_q.put('not busy') # msg = 'b' to the MCU - maybe not relevant anymore (if blending is on)

    elif (state == 2): # SYNCIN - Begin Sequence = accelerate with updated V,A for robot
        
        ROBOT_CONV_ACCELERATION = acc

        robot.setPoseFrame(frame_PCS)
        robot.setSpeed(CONV_SPEED,-1,ROBOT_CONV_ACCELERATION,-1) #to be changed to real live values
        
        #calc move vector acc to speed
        X_ShiftAPP = CalcDXoffset(APPROACH + SKIMM_DEPTH, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)
        X_ShiftLEN = CalcDXoffset(SKIMM_LEN, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)

        target_Approach = target_PCS_Origin.Pose() * transl(X_ShiftAPP,-10,APPROACH)
        target_conv_start = target_Approach * transl(X_ShiftAPP,0,-APPROACH-SKIMM_DEPTH)
        target_conv_final = target_conv_start * transl(X_ShiftLEN,-SKIMM_LEN,0)
        target_conv_det = target_conv_final * transl(0,SKIMM_LEN/5,APPROACH)
        
        # print(target_Approach)
        # robot.MoveL(target_Approach,False) # Set syncing mode
        robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
        # pause(2)
        robot.MoveL(target_conv_start,False)
        robot.MoveL(target_conv_final,False)
        robot.MoveL(target_conv_det,False)
        last_state = state

    elif (state == 3): # IN SYNC - Begin Skimm
        # robot.setPoseFrame(frame_PCS)

        # if not (robot.Busy()):
        mcu_q.put('done')

         # -------- Trash Movement--------------
        robot.setPoseFrame(frame_box) #Change Referance Frame

        robot.MoveL(target_Trash_up)
        # robot.MoveL(target_trash_turn)
        robot.MoveL(target_Trash_dump)
        # robot.MoveL(target_trash_turn)
        robot.MoveL(target_Trash_up)

        robot.setPoseFrame(frame_MCS) #Change Referance Frame
        robot.MoveL(target_Ready)    
        
        print("Succesfuly finished a cycle :)")
    
        state = -1 #Back to IDLE
        last_state = state
        
       

#todo change data storage to 3 variables
def robodk_loop():
    """ Main State-Machine Loop """
    global state,vel,acc,xpos,conv_pos,last_state,IOP_X
    IOP_X = xpos
    last_state = state

    # robot.setZoneData(10) #define a blending of 10mm
    robot.setRounding(10) #define a blending of 10mm

    robot.setPoseFrame(frame_MCS)
    
    # robot.MoveJ(target_home_safe,False) #Go Home
    robot.MoveL(target_Ready,False)

    while True: # State Machine loop

        read_from_sm_q() # update all tasks if more then 1 then check state machine
    
        Update_Conveyor()

        # Simulation of MCU state machine input
        if (withMCU == False and last_state != -1): 
            if (conv_pos < 40 and last_state == 0):
                state = 1       
            elif (conv_pos >= 40 and last_state == 1):
                state = 2
            elif (conv_pos >= 100 and last_state == 2):
                state = 3

        #State-Machine for skimming
        if (state != -1 and last_state != state):
            SkimmingSM()
        
        time.sleep(0.0001)

ConnectToRobot()

if (withMCU == False and runinng_method == RUNMODE_SIMULATE):
    robodk_loop() # To be removed when running thru main.py






"""
        if (state == -1): #StandBye - Do nothing
            if (last_state != -1 and withMCU == False): # Simulation of MCU state machine input
                if (conv_pos < 40 and last_state == 0):
                    state = 1       
                elif (conv_pos >= 40 and last_state == 1):
                    state = 2
                elif (conv_pos >= 100 and last_state == 2):
                    state = 3
            
        elif (state == 0): # CAM_CheckUpdate 

            if (IOP_X == 999.9): # For as long as 
                cam_q.put('getX') # Send command to camera, to pick up X position value
                sleep(0.1) # Does this interfere with flow? Should this be just a counter
                IOP_X = xpos
                if (withMCU == False): # Update, only for simulation
                    ConvF.setJoints([IOP_X+CONV_HOME])
            else:
                msg = 'c' + str(IOP_X)
                print("data to mcu_q : " + msg)
                mcu_q.put(msg) #Send IOP_X to MCU
                IOP_X = 999.9 #reset default value
                
                last_state = state
                state = -1 #Wait for MCU to trigger state 1

        elif (state == 1): #Begin Sequence - Go to Ready position and notify MCU
            robot.setPoseFrame(frame_MCS)
            robot.MoveL(target_Ready,False) #Go and wait
            
            
            mcu_q.put('not busy') # msg = 'b' to the MCU - maybe not relevant anymore (if blending is on)
            
            last_state = state
            state = -1
            
        elif (state == 2): # SYNCIN - Begin Sequence = accelerate with updated V,A for robot
            
            ROBOT_CONV_ACCELERATION = acc
            robot.setPoseFrame(frame_CBO)
            robot.setSpeed(CONV_SPEED,-1,10*ROBOT_CONV_ACCELERATION,-1) #to be changed to real live values
            
            ConvF.setJoints([conv_pos+CONV_HOME])
            print(conv_pos)
            print(robot.Pose())
            robot.MoveL(target_FarGoal,False) # Set syncing mode
            

            #calc move vector acc to speed
            X_ShiftAPP = CalcDXoffset(APPROACH + SKIMM_DEPTH, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)
            X_ShiftLEN = CalcDXoffset(SKIMM_LEN, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)

            last_state = state
            state = -1
            

        elif (state == 3): # IN SYNC - Begin Skimm
            # target_FarGoal = pose
            ConvF.setJoints([conv_pos+CONV_HOME])
            print(conv_pos)
            robot.setPoseFrame(frame_CBO)
            print(robot.Pose())
            # robot.setPoseFrame(frame_PCS)

            # robot.setPoseFrame(frame_MCS)

            # Calculate targets - Adjust position along movement according to conveyor speed
            # pause(3)
            # robot_cur_pose = robot.Pose()
            # conv_pos = target_PCS_Origin.Pose() #Only works for simulation
            # frame_PCS.Pose
            
            # target_PCS_Origin.setPose(transl(conv_pos,0,0))
            # robot.setRounding(10)
            # robot.Stop()
            pose = robot.Pose()                     # retrieve the current robot position as a pose (position of the active tool with respect to the active reference frame)
            # 
            # robot.MoveL(target_Ready)
            # Read the 4x4 pose matrix as [X,Y,Z, u,v,w] representation (position in mm and orientation vector in radians): same representation as Universal Robots
            xyzuvw = Pose_2_UR(pose)
            print(xyzuvw)
            x,y,z,a,b,c = xyzuvw                    # Use the KUKA representation (for example) and calculate a new pose based on the previous pose
            XYZABC2 = [x+(2*X_ShiftAPP),y,z-SKIMM_DEPTH-APPROACH,a,b,c]
            target_conv_start = UR_2_Pose(XYZABC2) 
            print(target_conv_start)
            # target_conv_start = target_PCS_Origin.Pose() * transl(2*X_ShiftAPP,0,0)
            # target_conv_start = robot.Pose() * transl((2* X_ShiftAPP),0,-APPROACH-SKIMM_DEPTH)
            # print(target_conv_start)
            # target_conv_start = conv_pos * transl(2*X_ShiftAPP,0,-SKIMM_DEPTH)
            target_conv_final = target_conv_start * transl(X_ShiftLEN,-SKIMM_LEN,0)
            target_conv_det = target_conv_final * transl(X_ShiftAPP,10,APPROACH)
            
            robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
            robot.MoveL(target_conv_start,False)
            robot.MoveL(target_conv_final,False)
            robot.MoveL(target_conv_det,False)

            #Send 'd' to MCU - for done skimming
            # msg = 'd'
            mcu_q.put('done')
            
            last_state = state
            state = 4

        elif (state == 4): #Trash state
            print("S4")
            # robot.setPoseFrame(frame_MCS) #Change Referance Frame
            # robot.MoveJ(target_home_safe)    
            
            # -------- Trash Movement--------------
            robot.setPoseFrame(frame_box) #Change Referance Frame

            robot.MoveL(target_Trash_up)
            # robot.MoveL(target_trash_turn)
            robot.MoveL(target_Trash_dump)
            # robot.MoveL(target_trash_turn)
            robot.MoveL(target_Trash_up)

            robot.setPoseFrame(frame_MCS) #Change Referance Frame
            robot.MoveL(target_Ready)    

            last_state = state
            state = -1
            """