#! /usr/bin/env python
__author__ = 'Raphael LEBER'
__copyright__ = "Copyright 2018, CPE Lyon, Lyontech"
__credits__ = ["Raphael Leber"]
__license__ = "MIT"
__version__ = "1.1.0"
__maintainer__ = 'Raphael LEBER'
__email__ = 'raphael.leber@gmail.com    '
__status__ = "Robocup 2018"


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from openpose_ros_msgs.msg import Persons, PersonDetection, BodyPartDetection
from ros_openpose_gossip_msgs.msg import PersonGossip, PersonsGossip
from math import sqrt, pow, fabs, atan2, pi, cos, sin
from enum import IntEnum
from RawPoseIndex import RawPoseIndex
import csv
from copy import deepcopy

import rospkg




# Used to make Bounding boxes
class BoundingBoxes():

    def __init__(self, image_w, image_h, inflate_x=0.1, inflate_y=0.1):

        self.image_w = 0
        self.image_h = 0



    def _makeBounginBox(self, body_part_limited, limbs):

        if len(body_part_limited) == 0 :
            return [ ]

        body_partSorted_x = sorted(body_part_limited, key=lambda attributes: attributes.x)   # sort by 
        body_partSorted_y = sorted(body_part_limited, key=lambda attributes: attributes.y)   # sort by 

        min_x = body_partSorted_x[0].x
        max_x = body_partSorted_x[-1].x
        min_y = body_partSorted_y[0].y
        max_y = body_partSorted_y[-1].y

        x_length = max_x - min_x
        y_length = max_y - min_y

        if ("R_NoseToEye" in limbs['abs']) and ("L_NoseToEye" in limbs['abs']) :
            eyes_to_hair = (limbs['abs']["R_NoseToEye"] + limbs['abs']["R_NoseToEye"]) / 2
        elif ("R_NoseToEye" in limbs['abs']) :
            eyes_to_hair = limbs['abs']["R_NoseToEye"]
        elif ("L_NoseToEye" in limbs['abs']) :
            eyes_to_hair = limbs['abs']["L_NoseToEye"]         
        else :
            eyes_to_hair = 4         

        min_x = min_x - 0.1 * x_length
        if min_x < 0 : min_x = 0

        max_x = max_x + 0.1 * x_length
        if max_x > self.image_w : max_x = self.image_w

        min_y = min_y - 0.1 * y_length - eyes_to_hair # TODO : eyes_to_hair must consider person orientation
        if min_y < 0 : min_y = 0

        max_y = max_y + 0.1 * y_length
        if max_y > self.image_h : max_y = self.image_h       
        
        TopLeft     =   Point32(    x = min_x   , y = min_y      )
        #TopRight    =   Point32(    x = max_x   , y = min_y      )
        #DownLeft    =   Point32(    x = min_x   , y = max_y     )
        DownRight   =   Point32(    x = max_x   , y = max_y     )

        print "bb: " + str([ TopLeft, DownRight ])

        return [ TopLeft, DownRight ]     



    def bodyBoundingBox(self, body_part, limbs):

        bps = deepcopy(body_part)

        for i in xrange(len(body_part)-1, -1, -1):
            if  bps[i].confidence == 0 \
                or body_part[i].x > self.image_w \
                or body_part[i].y > self.image_h \
                or body_part[i].x < 0 \
                or body_part[i].y < 0 :

                bps.pop(i)

        return self._makeBounginBox(bps, limbs)



    def headBoundingBox(self, body_part, limbs):

        bps = deepcopy(body_part)

        for i in xrange(len(body_part)-1, -1, -1):
            if  bps[i].confidence == 0 \
                or body_part[i].x > self.image_w \
                or body_part[i].y > self.image_h \
                or body_part[i].x < 0 \
                or body_part[i].y < 0 \
                or i == RawPoseIndex.R_Shoulder \
                or i == RawPoseIndex.R_Elbow    \
                or i == RawPoseIndex.R_Wrist    \
                or i == RawPoseIndex.L_Shoulder \
                or i == RawPoseIndex.L_Elbow    \
                or i == RawPoseIndex.L_Wrist    \
                or i == RawPoseIndex.R_Hip      \
                or i == RawPoseIndex.R_Knee     \
                or i == RawPoseIndex.R_Ankle    \
                or i == RawPoseIndex.R_Ankle    \
                or i == RawPoseIndex.R_Hip      \
                or i == RawPoseIndex.L_Knee     \
                or i == RawPoseIndex.L_Ankle    :

                bps.pop(i)

        return self._makeBounginBox(bps, limbs)



    def getShirtSampleRect(self, body_part):

        if (body_part[RawPoseIndex.L_Hip ].confidence != 0) and (body_part[RawPoseIndex.R_Hip ].confidence != 0) and (body_part[RawPoseIndex.Neck ].confidence != 0) :
            x = fabs( body_part[RawPoseIndex.L_Hip].x - body_part[RawPoseIndex.R_Hip].x )
            y = fabs( body_part[RawPoseIndex.Neck].y - body_part[RawPoseIndex.R_Hip].y )            
            middle_x = ( body_part[RawPoseIndex.L_Hip].x + body_part[RawPoseIndex.R_Hip].x ) / 2
            middle_y = (body_part[RawPoseIndex.Neck].y * 2 + body_part[RawPoseIndex.R_Hip].y + body_part[RawPoseIndex.L_Hip].y ) / 4

        elif (body_part[RawPoseIndex.L_Hip ].confidence != 0) and (body_part[RawPoseIndex.Neck ].confidence != 0) :
            x = fabs( body_part[RawPoseIndex.Neck].x - body_part[RawPoseIndex.L_Hip].x )
            y = fabs( body_part[RawPoseIndex.Neck].y - body_part[RawPoseIndex.L_Hip].y )            
            middle_x = ( body_part[RawPoseIndex.L_Hip].x + body_part[RawPoseIndex.Neck].x ) / 2
            middle_y = (body_part[RawPoseIndex.Neck].y + body_part[RawPoseIndex.L_Hip].y ) / 2
        
        elif (body_part[RawPoseIndex.R_Hip ].confidence != 0) and (body_part[RawPoseIndex.Neck ].confidence != 0) :
            x = fabs( body_part[RawPoseIndex.Neck].x - body_part[RawPoseIndex.R_Hip].x )
            y = fabs( body_part[RawPoseIndex.Neck].y - body_part[RawPoseIndex.R_Hip].y )               
            middle_x = ( body_part[RawPoseIndex.R_Hip].x + body_part[RawPoseIndex.Neck].x ) / 2
            middle_y = (body_part[RawPoseIndex.Neck].y + body_part[RawPoseIndex.R_Hip].y ) / 2  
        
        else :
            return [ ]


        coef_x = 0.4 # Must be < 0.5
        coef_y = 0.35 # Must be < 0.5
    
        TopLeft     =   Point32(    x = middle_x - x*coef_x   , y = middle_y - y*coef_y      )

        DownRight   =   Point32(    x = middle_x + x*coef_x   , y = middle_y + y*coef_y     )

        return [ TopLeft, DownRight ]



    def getTrouserSampleRect(self, body_part, limbs):

        if "R_Thigh" in limbs['abs']:
            x = limbs['x']['R_Thigh']
            y = limbs['y']['R_Thigh']
            middle_x = ( body_part[RawPoseIndex.R_Hip].x + body_part[RawPoseIndex.R_Knee].x ) / 2
            middle_y = (body_part[RawPoseIndex.R_Hip].y + body_part[RawPoseIndex.R_Knee].y ) / 2     
            thigh_abs = limbs['abs']['R_Thigh']     
        elif "L_Thigh" in limbs['abs']:
            x = limbs['x']['L_Thigh']
            y = limbs['y']['L_Thigh']
            middle_x = ( body_part[RawPoseIndex.L_Hip].x + body_part[RawPoseIndex.L_Knee].x ) / 2
            middle_y = (body_part[RawPoseIndex.L_Hip].y + body_part[RawPoseIndex.L_Knee].y ) / 2   
            thigh_abs = limbs['abs']['L_Thigh'] 
        else :
            return [  ]


        kernel_side = 4
     

        TopLeft     =   Point32(    x = middle_x - kernel_side   , y = middle_y - kernel_side      )

        DownRight   =   Point32(    x = middle_x + kernel_side   , y = middle_y + kernel_side     )        

        return [ TopLeft, DownRight ]        

