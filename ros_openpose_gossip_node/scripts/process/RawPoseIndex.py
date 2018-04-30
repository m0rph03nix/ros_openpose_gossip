#! /usr/bin/env python

__author__ = 'Raphael LEBER'

from enum import IntEnum

class RawPoseIndex(IntEnum):
    Nose        = 0
    Neck        = 1
    R_Shoulder  = 2
    R_Elbow     = 3
    R_Wrist     = 4 
    L_Shoulder  = 5 
    L_Elbow     = 6 
    L_Wrist     = 7 
    R_Hip       = 8 
    R_Knee      = 9 
    R_Ankle     = 10 
    L_Hip       = 11 
    L_Knee      = 12 
    L_Ankle     = 13
    R_Eye       = 14 
    L_Eye       = 15 
    R_Ear       = 16 
    L_Ear       = 17
