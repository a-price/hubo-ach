#!/usr/bin/env python
from ctypes import *
import ach
import sys

HUBO_JOINT_COUNT         = 42
HUBO_JMC_COUNT           = 0x26
HUBO_IMU_COUNT           = 3
HUBO_CHAN_REF_NAME       = 'hubo-ref'        
HUBO_CHAN_BOARD_CMD_NAME = 'hubo-board-cmd'
HUBO_CHAN_STATE_NAME     = 'hubo-state'     

LHY = 19# Left Hip Yaw
LHR = 20# Left Hip Roll
LHP = 21# Left Hip Pitch
LKN = 22# Left Knee Pitch
LAP = 23# Left Ankle Pitch
LAR = 24# Left Ankle Roll

RSP = 11# Right Shoulder Pitch
RSR = 12# Right Shoulder Roll
RSY = 13# Right Shoulder Yaw
REB = 14# Right Elbow Pitch
RWY = 15# right wrist yaw
RWR = 16# right wrist roll
RWP = 17# right wrist Pitch

LSP = 4 # Left Shoulder Pitch
LSR = 5 # Left Shoulder Yaw
LSY = 6 # Left Shoulder Roll
LEB = 7 # Left Elbow Pitch
LWY = 8 # left wrist yaw
LWR = 9 # left wrist roll
LWP = 10# left wrist pitch

NKY = 1 # neck yaw
NK1 = 2 # neck 1
NK2 = 3 # neck 2

WST = 0 # Trunk Yaw

RF1 = 32 # Right Finger
RF2 = 33 # Right Finger
RF3 = 34 # Right Finger
RF4 = 35 # Right Finger
RF5 = 36 # Right Finger
LF1 = 37 # Left Finger
LF2 = 38 # Left Finger
LF3 = 39 # Left Finger
LF4 = 40 # Left Finger
LF5 = 41 # Left Finger


class HUBO_SENSOR_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("sensNo"  , c_uint),
                ("can"     , c_uint),
                ("boardNo" , c_uint),
                ("active"  , c_ubyte),
                ("name"    , c_ubyte*5)]

class HUBO_JOINT_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("refEnc"   , c_uint32),
                ("motNo"    , c_uint),
                ("jntNo"    , c_uint), 
                ("drive"    , c_uint), 
                ("driven"   , c_uint), 
                ("harmonic" , c_uint), 
                ("enc"      , c_uint), 
                ("jmc"      , c_uint), 
                ("dir"      , c_ubyte), 
                ("can"      , c_ubyte), 
                ("numMot"   , c_ubyte),
                ("name"     , c_ubyte*4)]

class HUBO_JMC_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("joints" , c_ubyte*5)]


class HUBO_PARAM(Structure):
    _pack_ = 1
    _fields_ = [("joint"  , HUBO_JOINT_PARAM*HUBO_JOINT_COUNT),
                ("driver" , HUBO_JMC_PARAM*HUBO_JOINT_COUNT),
                ("sensor" , HUBO_SENSOR_PARAM*HUBO_JOINT_COUNT)]


class HUBO_IMU(Structure):
    _pack_ = 1
    _fields_ = [("a_x", c_double),
                ("a_y", c_double),
                ("a_z", c_double),
                ("w_x", c_double),
                ("w_y", c_double),
                ("w_z", c_double)]

class HUBO_FT(Structure):
    _pack_ = 1
    _fields_ = [("m_x", c_double),
                ("m_y", c_double),
                ("f_z", c_double)]

class HUBO_JOINT_STATE(Structure):
    _pack_ = 1
    _fields_ = [("ref"   , c_double),
                ("pos"   , c_double),
                ("cur"   , c_double),
                ("vel"   , c_double),
                ("heat"  , c_double),
                ("tmp"   , c_double),
                ("active", c_ubyte),
                ("zeroed", c_ubyte)]

class HUBO_JOINT_STATUS(Structure):
    _pack_ = 1
    _fields_ = [("driveOn"      , c_ubyte),
                ("ctrlOn"       , c_ubyte),
                ("mode"         , c_ubyte),
                ("limitSwitch"  , c_ubyte),
                ("homeFlag"     , c_ubyte),
                ("jam"          , c_ubyte),
                ("pwmSaturated" , c_ubyte),
                ("bigError"     , c_ubyte),
                ("encError"     , c_ubyte),
                ("driverFault"  , c_ubyte),
                ("motorFail0"   , c_ubyte),
                ("motorFail1"   , c_ubyte),
                ("posMinError"  , c_ubyte),
                ("posMaxError"  , c_ubyte),
                ("velError"     , c_ubyte),
                ("accError"     , c_ubyte),
                ("tempError"    , c_ubyte)]

class HUBO_JMC_STATE(Structure):
    _pack_ = 1
    _fields_ = [("temp" , c_double)]

class HUBO_STATE(Structure):
    _pack_ = 1
    _fields_ = [("imu"    , HUBO_IMU*HUBO_IMU_COUNT),
                ("ft"     , HUBO_FT*4),
                ("joint"  , HUBO_JOINT_STATE*HUBO_JOINT_COUNT),
                ("status" , HUBO_JOINT_STATUS*HUBO_JOINT_COUNT),
                ("driver" , HUBO_JMC_STATE*HUBO_JMC_COUNT),
                ("time"   , c_double),
                ("refWait", c_int)]




class HUBO_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",  c_double*HUBO_JOINT_COUNT),
                ("mode", c_int*HUBO_JOINT_COUNT)]

