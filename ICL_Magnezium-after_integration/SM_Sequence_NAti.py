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
# RDK.AddFile(r'C:\Users\Daniel\Desktop\New folder\Daniel_Test.rdk')
# RDK.AddFile(r'G:\Shared drives\KAUFFMAN-TEAM\kaufman-rd\solid\102 - ICL\01 - Magnesium Ingot Scimming\Software\ICL_magnezium-main\Daniel_Test.rdk')

#todo finish open program from robodk app
# rdk_file_path = RDK.getParam("PATH_OPENSTATION")
# path_file = getOpenFile(rdk_file_path + "/")
# if not path_file:
#     print("Nothing selected")
#     quit()

#global variable to store velocity, acceleration and x positoin
state = -1
vel = 0
acc = 0
xpos = 999.9
conv_pos = 0

runinng_method = RUNMODE_RUN_ROBOT # RUNMODE_SIMULATE - simulation, RUNMODE_RUN_ROBOT - real movement on robot
withMCU = True

#Define default skimming length
APPROACH = 150 # define default approach distance
SKIMM_LEN = 590
SKIMM_DEPTH = 45
X_IDLE_APP = 55 # Estimator for latency

CONV_HOME = 1931.0 # 1931.0 is calibrated

CONV_SPEED = 27 #mm/s
ROBOT_SPEED = 1000 #mm/s
ROBOT_ACCELERATION = 4000 # mm/s^2
ROBOT_CONV_ACCELERATION = 48.6 # mm/s^2

def CalcDXoffset (length, conv_speed, robot_speed, robot_acceleration):   
    """ Calculates offset due to acceleration & decceleration """
    if (((robot_speed * robot_speed) / robot_acceleration) >= length):
        t = 2 * sqrt(length / robot_acceleration)
    else:
        t = (length / robot_speed) + (robot_speed / robot_acceleration)
    
    delta = conv_speed * t
    return delta


# pose_distance = distance(pose_from.Pos(), pose_to.Pos())

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
        
        while not sm_q.empty():
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
        
#todo change data storage to 3 variables
def robodk_loop_sim():
    """ Main State-Machine Loop """
    global state,vel,acc,xpos,conv_pos
    IOP_X = xpos
    cam_flag = 0
    # Init SM params - TDB
    # state = 0
    last_state = -1
    # conv_pos = CONV_HOME
    X_ShiftAPP = 0.0
    X_ShiftLEN = 0.0

    robot.setPoseFrame(frame_MCS)
    # robot.MoveJ(target_home_safe) #Go Home
    robot.MoveL(target_Ready)
    while True:
        #Get State
        # if not sm_q.empty():
        #     state = sm_q.get() # getting state as string
        #     state = int(state[1:]) # changing string to int
        
        read_from_sm_q() # update all tasks if more then 1 then check state machine
        
        if (state == -1): #StandBye
            # if (runinng_method == RUNMODE_SIMULATE and last_state != -1):
            if (last_state != -1):
                if (runinng_method == RUNMODE_SIMULATE and withMCU == False):
                    x_conv = ConvF.Joints()[0,0] - conv_pos #mimic MCU controller
                    if (x_conv < 40 and last_state == 0):
                        state = 1       
                    elif (x_conv >= 40 and last_state == 1):
                        state = 2
                    elif (x_conv >= 70 and last_state == 2):
                        state = 3
            
        elif (state == 0): #CAM_CheckUpdate
            # if(not cam_flag == 2): #due to cam picturing delay
            #     cam_q.put('getX')
            #     cam_flag = cam_flag + 1
            # read_from_sm_q()
           
           
            # IOP_X = xpos
           
            # print(f"IOP : {IOP_X}")
            if IOP_X == 999.9:
                cam_q.put('getX')
                IOP_X = xpos
                sleep(0.1)

            else:
                msg = 'c' + str(IOP_X)
                print("data to mcu_q : " + msg)
                mcu_q.put(msg) #Send IOP_X to MCU
                # Conv_pos = CONV_HOME + IOP_X # Manual moving PCS to determine CBO offset from MCS origin due to cam IOP_X
                IOP_X = 999.9 #reset default value
                
                last_state = state
                state = -1 #Wait for MCU to trigger state 1

        elif (state == 1): #Begin Sequence - Go to Ready position and notify MCU
            cam_flag = 0
            
            CONV_SPEED = vel                    
            
            #Simulate Conveyor
            ConvF.setJoints([conv_pos])
            # ConvF.setSpeed(CONV_SPEED,-1,-1,-1) #to be changed to real live values
            # ConvF.MoveJ(target_conv_far,False)
            
            robot.setPoseFrame(frame_MCS)
            robot.MoveL(target_Ready) #Go and wait
            
            # msg = 'b'
            mcu_q.put('not busy')
            last_state = state
            state = -1
            
        elif (state == 2): # SYNCIN - Begin Sequence = accelerate with updated V,A for robot
            
            CONV_SPEED = vel
            ROBOT_CONV_ACCELERATION = acc
            robot.setPoseFrame(frame_CBO)
            robot.setSpeed(CONV_SPEED,-1,10*ROBOT_CONV_ACCELERATION,-1) #to be changed to real live values
            robot.setRounding(10)
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
            # conv_pos = target_IOP.Pose() #Only works for simulation
            # frame_PCS.Pose
            
            # target_IOP.setPose(transl(conv_pos,0,0))
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
            # target_conv_start = target_IOP.Pose() * transl(2*X_ShiftAPP,0,0)
            # target_conv_start = robot.Pose() * transl((2* X_ShiftAPP),0,-APPROACH-SKIMM_DEPTH)
            # print(target_conv_start)
            # target_conv_start = conv_pos * transl(2*X_ShiftAPP,0,-SKIMM_DEPTH)
            target_conv_final = target_conv_start * transl(X_ShiftLEN,-SKIMM_LEN,0)
            target_conv_det = target_conv_final * transl(X_ShiftAPP,10,APPROACH)
            
            robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
            robot.MoveL(target_conv_start)
            robot.MoveL(target_conv_final)
            robot.MoveL(target_conv_det)

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

target_IOP = RDK.Item('IOP', ITEM_TYPE_TARGET)
target_conv_far = RDK.Item('ConveyorMove', ITEM_TYPE_TARGET)

target_Trash_up = RDK.Item('Trash_up', ITEM_TYPE_TARGET)
target_trash_turn = RDK.Item('trash_turn', ITEM_TYPE_TARGET)
target_Trash_dump = RDK.Item('Trash_dump', ITEM_TYPE_TARGET)

#Set Referances
robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
robot.setTool(tool)

ConnectToRobot()

RDK.setRunMode(runinng_method) # Change between simulation and actual movement



# robodk_loop_sim() # To be removed when running thru main.py






