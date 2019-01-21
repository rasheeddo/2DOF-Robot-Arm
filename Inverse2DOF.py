import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import pygame
import numpy
import math
import matplotlib.pyplot as plt

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def INV(px,py):
    L = math.sqrt(px**2 + py**2)
    pi_Q2 = math.acos( (px**2 + py**2 - a1**2 - a2**2) / (-2*a1*a2) )
    Q2 = math.pi - pi_Q2
    gam = math.asin( (a2*math.sin(Q2))/L)
    alp = math.atan2(py,px)
    Q1 = alp - gam
    return Q1, Q2

def VelocityLimit2(set_V_Limit):
    # max is 1023 raw value  => 1405 deg/s
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

def RunServo1(inputDeg1):
    pos1 = inputDeg1+90.0
    if pos1 >= 60.0 and pos1 < 300.0:
        servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
        dxl1_goal_position = int(servo_com1)
        dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
        if dxl_comm_result1 != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
        elif dxl_error1 != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error1))
    else:
        print("input1 out of range!")

def RunServo2(inputDeg2):
    pos2 = inputDeg2+90.0
    if pos2 >= 60.0 and pos2 < 300.0:
        servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
        dxl2_goal_position = int(servo_com2)
        dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
        if dxl_comm_result2 != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
        elif dxl_error2 != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error2))
    else:
        print("input2 out of range!")

def RunWithSpeed(inputDeg1,inputSpeed1,inputDeg2,inputSpeed2):
    inputDeg1 = inputDeg1 + 90.0
    inputDeg2 = inputDeg2 + 180.0

    if inputDeg1 >= 60.0 and inputDeg1 < 300.0 and inputDeg2 >= 60.0 and inputDeg2 < 300.0:
        # inputSpeed = deg/sec
        inputSpeedRaw1 = int(inputSpeed1/1.374)
        inputSpeedRaw2 = int(inputSpeed2/1.374)  # convertig deg/s to raw value

        servo_ang1 = map(inputDeg1, 0.0, 360.0, 0, 4095)
        servo_ang2 = map(inputDeg2, 0.0, 360.0, 0, 4095)
        dxl1_goal_position = int(servo_ang1)
        dxl2_goal_position = int(servo_ang2)

        velocity_position1 = [DXL_LOBYTE(DXL_LOWORD(inputSpeedRaw1)), 
                            DXL_HIBYTE(DXL_LOWORD(inputSpeedRaw1)),
                            DXL_LOBYTE(DXL_HIWORD(inputSpeedRaw1)), 
                            DXL_HIBYTE(DXL_HIWORD(inputSpeedRaw1)),
                            DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), 
                            DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
                            DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), 
                            DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]

        velocity_position2 = [DXL_LOBYTE(DXL_LOWORD(inputSpeedRaw2)), 
                                DXL_HIBYTE(DXL_LOWORD(inputSpeedRaw2)),
                                DXL_LOBYTE(DXL_HIWORD(inputSpeedRaw2)), 
                                DXL_HIBYTE(DXL_HIWORD(inputSpeedRaw2)),
                                DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), 
                                DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
                                DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), 
                                DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePositionWithSpeed.addParam(DXL1_ID, velocity_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" %DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePositionWithSpeed.addParam(DXL2_ID, velocity_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" %DXL2_ID)
            quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePositionWithSpeed.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePositionWithSpeed.clearParam()

    else:
        print("inputDeg out of range!")
        
def ReadRobotAngle():

        # Syncread present position
        dxl_comm_result = groupSyncReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncReadPosition.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" %DXL1_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncReadPosition.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" %DXL2_ID)
            quit()


        # Get Dynamixel#1 present position value
        dxl_present_position1 = groupSyncReadPosition.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        # Get Dynamixel#2 present position value
        dxl_present_position2 = groupSyncReadPosition.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        deg1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
        deg2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)

        deg1 = deg1 - 90
        deg2 = deg2 - 180

        return deg1, deg2

def IsMoving1():
    Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
    return Moving1

def IsMoving2():
    Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
    return Moving2
            

def TorqueOn():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

def SetPID1(set_P_Gain,set_I_Gain,set_D_Gain):
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
    
    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 1: %d" %position_P_gain)
    print("Position I Gain 1: %d" %position_I_gain)
    print("Position D Gain 1: %d" %position_D_gain)
    print("------------------------------")

def SetPID2(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 2: %d" %position_P_gain)
    print("Position I Gain 2: %d" %position_I_gain)
    print("Position D Gain 2: %d" %position_D_gain)
    print("------------------------------")
####################################################### Set Servo Configuration #############################################################
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84
ADDR_PRO_FEEDFORWARD_2nd_GAIN = 88
ADDR_PRO_FEEDFORWARD_1st_GAIN = 90

ADDR_PRO_MOVING             = 122
ADDR_PRO_MOVING_STATUS       = 123

ADDR_PRO_REALTIME_TICK       = 120

# Data Byte Length
LEN_PRO_PRESENT_POSITION            = 4
LEN_PRO_GOAL_POSITION               = 4
LEN_PRO_PROFILE_VELOCITY            = 4
LEN_PRO_POS_VEL                     = 8

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM15'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

groupSyncWritePositionWithSpeed = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_POS_VEL)

groupSyncReadPosition = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncReadPosition.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" %DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncReadPosition.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" %DXL2_ID)
    quit()


global a1, a2
# Parameters
a1 = 211  # mm
a2 = 180  # mm

#VelocityLimit2(1023)
SetPID1(900,100,5000)
SetPID1(900,100,5000)
TorqueOn()


# Input
PX_start = 250
PY_start = 100

PX_goal = 250
PY_goal = 250


Ang1, Ang2 = INV(PX_start,PY_start)
Ang1 = Ang1*180.0/math.pi
Ang2 = Ang2*180.0/math.pi
RunWithSpeed(Ang1,90,Ang2,90)
print("Ang1",Ang1)
print("Ang2",Ang2)
time.sleep(6)



# Variables
des_vel = 40 # mm/s

distance = math.sqrt( (PX_goal - PX_start)**2 + (PY_goal - PY_start)**2 )   # Total distance
vector = [(PX_goal-PX_start), (PY_goal-PY_start)]  # a vector of path
unit_vec = [vector[0]/distance, vector[1]/distance] # Specify the direction
Tip_vel = [unit_vec[0]*des_vel, unit_vec[0]*des_vel] # tip's velocity vector 
finish_time = distance/des_vel  # Total time [sec] from start to goal points
points = round(distance)
gap = distance/points
gap_time = finish_time/points

print("Distance", distance)
print("Desired Speed", des_vel)
print("Unit vector", unit_vec)
print("Finish time", finish_time)
print("Points", points)
print("Gap distance", gap)
print("Gap time", gap_time)

Px = list()
Py = list()
Q1 = list()
Q2 = list()
The1 = list()
The2 = list()

for i in range(0,points):
    if i == 0:
        Px.append(PX_start + unit_vec[0]*gap)
        Py.append(PY_start + unit_vec[1]*gap)
    else:
        Px.append(Px[i-1] + unit_vec[0]*gap)
        Py.append(Py[i-1] + unit_vec[1]*gap)

    qq1, qq2 = INV(Px[i], Py[i])
    Q1.append(qq1)
    Q2.append(qq2)
    The1.append(qq1*180.0/math.pi)
    The2.append(qq2*180.0/math.pi)



startTime = time.time()
for i in range(0,points):
    #print("i", i)
    startTimeLoop = time.time()
    RunWithSpeed(The1[i],0,The2[i],0)
    TimeInLoop = time.time() - startTimeLoop
    #print("TimeInLoopRun", TimeInLoop)
    # normally one loop should take t seconds, but time in loop of RunWithSpeed may effect
    time.sleep(abs(gap_time-TimeInLoop)) 

    #time.sleep(0.01)  
    

period = time.time() - startTime
print("period", period)


Deg1,Deg2 = ReadRobotAngle()
print("Deg1",Deg1)
print("Deg2",Deg2)


'''
time=numpy.arange(0,points,1)
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(time,The1,'r')
plt.grid()
plt.title("Angle1")
plt.subplot(2, 1, 2)
plt.plot(time,The2,'b')
plt.grid()
plt.title("Angle2")

plt.show()
'''