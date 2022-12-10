# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project
from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox

# The following 2 lines keep your code compatible with older versions or RoboDK:
from robodk.robolink import *      # RoboDK API
from robodk.robomath import *      # Robot toolbox
from robolink import *    # Robot toolbox




# Link to RoboDK
RDK = robolink.Robolink()
RDK.AddFile(r'C:\Users\Daniel\Desktop\New folder\Daniel_Test.rdk')

#Define default skimming length
APPROACH = 150 # define default approach distance
SKIMM_LEN = 590
SKIMM_DEPTH = 45
X_IDLE_APP = 55 # Estimator for latency

CONV_HOME = 1938 # 1938.0 is calibrated

CONV_SPEED = 27 #mm/s
ROBOT_SPEED = 1000 #mm/s
ROBOT_ACCELERATION = 4000 # mm/s^2
ROBOT_CONV_ACCELERATION = 48.6 # mm/s^2

#Define initial Values
# nparts = 5 # Number of molds in system
# PauseT = 0.65

def CalcDXoffset (length, conv_speed, robot_speed, robot_acceleration):
    
    if (((robot_speed * robot_speed) / robot_acceleration) >= length):
        t = 2 * sqrt(length / robot_acceleration)
    else:
        t = (length / robot_speed) + (robot_speed / robot_acceleration)
    
    delta = conv_speed * t

    return delta

# gather robot, tool and reference frames from the station
robot               = RDK.Item('UR10e', ITEM_TYPE_ROBOT)
ConvF               = RDK.Item('ConvF', ITEM_TYPE_ROBOT)
tool                = RDK.Item('skimmer_ITEM', ITEM_TYPE_TOOL)
frame_conv          = RDK.Item('Conveyor F', ITEM_TYPE_FRAME)
frame_conv_moving   = RDK.Item('Frame_Conv1', ITEM_TYPE_FRAME)
frame_MCS           = RDK.Item('MCS Origin', ITEM_TYPE_FRAME)
frame_CBO           = RDK.Item('CBO', ITEM_TYPE_FRAME)
frame_PCS           = RDK.Item('PCS', ITEM_TYPE_FRAME)
frame_box           = RDK.Item('Frame_Box', ITEM_TYPE_FRAME)

# gather targets
target_home_safe = RDK.Item('Home', ITEM_TYPE_TARGET)
target_Ready = RDK.Item('Ready', ITEM_TYPE_TARGET)
target_FarGoal = RDK.Item('FarGoal', ITEM_TYPE_TARGET)
# target_PCS = RDK.Item('PCS', ITEM_TYPE_TARGET)

target_IOP = RDK.Item('IOP', ITEM_TYPE_TARGET)
target_conv_far = RDK.Item('ConveyorMove', ITEM_TYPE_TARGET)

target_Trash_up = RDK.Item('Trash_up', ITEM_TYPE_TARGET)
target_trash_turn = RDK.Item('trash_turn', ITEM_TYPE_TARGET)
target_Trash_dump = RDK.Item('Trash_dump', ITEM_TYPE_TARGET)

#rest simulated conveyor
if ConvF.Valid():
    ConvF.setJoints([CONV_HOME])
# ResetConv = RDK.Item('Reset Conveyor', ITEM_TYPE_PROGRAM)
# ResetConv.RunProgram()

# camera_ref_conv = target_conv.PoseAbs()

#------------------------------------------Track&Move--------------------------------------------
#Set Referances
robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)

robot.setTool(tool)

# robot.MoveJ(target_home_safe) #Go Home

state = -1
IOP_X = 20.0
Conv_pos = CONV_HOME
X_ShiftAPP = 0.0
X_ShiftLEN = 0.0


def robodk_loop():
    global state
    try:
        while True:
            #Get State
            if (state == -1): #Home
                robot.setPoseFrame(frame_MCS)
                robot.MoveJ(target_home_safe) #Go Home
                state = 0
                print(state)
                
            elif (state == 0): #MCU Triggered
                #Get IOP_X
                #Get Conv_speed = ROBOT_SPEED
                #Send IOP_X to MCU

                #Simulate Conveyor
                Conv_pos = CONV_HOME + IOP_X
                ConvF.setJoints([Conv_pos])
                ConvF.setSpeed(CONV_SPEED,-1,100 * CONV_SPEED,-1) #to be changed to real live values
                ConvF.MoveJ(target_conv_far,False)
                # target_IOP_new = target_IOP.Pose()*transl(IOP_X ,0,0) #Initial t0 skimming position
                
                state = 1
                print(state)

            elif (state == 1): #Begin Sequence
                Ready_pos = frame_CBO.Pose()*transl(X_IDLE_APP,0,APPROACH)
                robot.MoveL(Ready_pos,False) #Go and wait
                print(state)
                #send 'b' to MCU
                #Set state = 2
                state = 2
                print(state)

            elif (state == 2): # Waiting to Begin Sequence
                #mimic MCU controller
                x_conv = ConvF.Pose()[0, 3] - Conv_pos
                if (x_conv >= 40):
                    state = 3
                    print(state)

            elif (state == 3): # SYNCIN - Begin Sequence
                #Get Velocity - probably the same, mybe just an update CONV_SPEED
                #Get Acceleration ROBOT_CONV_ACCELERATION
                robot.setPoseFrame(frame_CBO)
                robot.setSpeed(CONV_SPEED,-1,ROBOT_CONV_ACCELERATION,-1) #to be changed to real live values
                robot.MoveL(target_FarGoal,False) # Set syncing mode
                

                #calc move vector acc to speed
                X_ShiftAPP = CalcDXoffset(APPROACH + SKIMM_DEPTH, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)
                X_ShiftLEN = CalcDXoffset(SKIMM_LEN, CONV_SPEED, ROBOT_SPEED, ROBOT_ACCELERATION)

                state = 4
                print(state)
                #Set state = 3

            elif (state == 4): # Waiting to Begin Sequence
                #mimic MCU controller
                x_conv = ConvF.Pose()[0, 3] - Conv_pos
                if (x_conv >= 70 ):
                    state = 5
                    print(state)

            elif (state == 5): # IN SYNC - Begin Skimm
                
                robot.setPoseFrame(frame_PCS)

                # Calculate targets - Adjust position along movement according to conveyor speed
                # pause(3)
                # robot_cur_pose = robot.Pose()
                conv_pos = target_IOP.Pose()
                # frame_PCS.Pose
                robot.setRounding(10)
                robot.Stop()
                # target_conv_start = robot_cur_pose * transl(X_ShiftAPP,0,-SKIMM_DEPTH-APPROACH)
                target_conv_start = conv_pos * transl(2*X_ShiftAPP,0,-SKIMM_DEPTH)
                target_conv_final = target_conv_start * transl(X_ShiftLEN,-SKIMM_LEN,0)
                target_conv_det = target_conv_final * transl(X_ShiftAPP,10,APPROACH+SKIMM_DEPTH)
                
                robot.setSpeed(ROBOT_SPEED,-1,ROBOT_ACCELERATION,-1)
                robot.MoveL(target_conv_start)
                robot.MoveL(target_conv_final)
                robot.MoveL(target_conv_det)

                #Send 'd' to MCU - for done skimming

                #Set state = 4
                state = 6
                # state = 8
                print(state)

            elif (state == 6):
                robot.setPoseFrame(frame_MCS) #Change Referance Frame
                robot.MoveJ(target_home_safe)    
                
                # -------- Trash Movement--------------
                robot.setPoseFrame(frame_box) #Change Referance Frame
                robot.MoveL(target_Trash_up)
                # robot.MoveL(target_trash_turn)
                robot.MoveL(target_Trash_dump)
                # robot.MoveL(target_trash_turn)
                robot.MoveL(target_Trash_up)

                #Set state = -1
                state = -1
                print(state)

    except KeyboardInterrupt:
        # Close RoboDK
        pass
    # RDK.CloseRoboDK()    


#----------Back To IDLE movement--------
robot.setPoseFrame(frame_MCS) #Change Referance Frame
robot.MoveL(target_home_safe)

# # Set Robot speeds
# robot.setSpeedJoints(180)
# robot.setSpeed(ROBOT_SPEED,-1,ROBOT_SPEED,-1)

# #----------Skimming Movement-----------------
# robot.MoveJ(target_conv_app)
# # print(PauseT)
# # pause(PauseT)    




