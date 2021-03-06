#! /usr/bin/env python
__author__ = 'Raphael LEBER'
__copyright__ = "Copyright 2018, CPE Lyon, Lyontech"
__credits__ = ["Raphael Leber"]
__license__ = "MIT"
__version__ = "2.0.0"
__maintainer__ = 'Raphael LEBER'
__email__ = 'raphael.leber@gmail.com    '
__status__ = "Robocup 2018"


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from openpose_ros_msgs.msg import Persons, PersonDetection, BodyPartDetection
from ros_openpose_gossip_msgs.msg import PersonGossip, PersonsGossip
from math import sqrt, pow, fabs, atan2, pi, cos, sin
from enum import IntEnum
from RawPoseIndex import RawPoseIndex
import csv
from copy import deepcopy

import rospkg



class OpenPoseGossip():

    def __init__(self):

        rospack = rospkg.RosPack()

        package_path=rospack.get_path('ros_openpose_gossip_node')

        self.image_w = 0
        self.image_h = 0

        self.calib_folder = package_path + "/calib/"

        #self.EnrichPersonsData(resp3.personList)

        

    def PartToLimb(self, limb, pair_key, person, part1, part2):
        if(person.body_part[part1].confidence > 0.1 and person.body_part[part2].confidence > 0.1):
            limb['x'][pair_key] = fabs(person.body_part[part1].x - person.body_part[part2].x)
            limb['y'][pair_key] = fabs(person.body_part[part1].y - person.body_part[part2].y)
            limb['abs'][pair_key] = sqrt( pow(person.body_part[part1].x - person.body_part[part2].x, 2) + pow(person.body_part[part1].y - person.body_part[part2].y, 2) )

    def PartsToLimbs(self, person):
        
        abs_limbs = {}
        x_limbs = {}
        y_limbs = {}
        limbs = {"x":x_limbs, "y":y_limbs, "abs":abs_limbs}

        # "Right/Left" means right/left of the person and not of the image
        self.PartToLimb(limbs, "R_Flank", person, 1, 8) # Right Flankappend
        self.PartToLimb(limbs, "L_Flank", person, 1, 11) # Left Flank
        self.PartToLimb(limbs, "R_Thigh", person, 8, 9) # Right Thigh
        self.PartToLimb(limbs, "L_Thigh", person, 11, 12) # Left Thigh
        self.PartToLimb(limbs, "R_Calf", person, 9, 10)   # Right Calf
        self.PartToLimb(limbs, "L_Calf", person, 12, 13) # Left Calf

        self.PartToLimb(limbs, "R_Shoulder", person, 1, 2) # Right Shoulder
        self.PartToLimb(limbs, "L_Shoulder", person, 1, 5) # left Shoulder
        self.PartToLimb(limbs, "R_Arm", person, 2, 3) # Right Arm
        self.PartToLimb(limbs, "L_Arm", person, 5, 6) # left Arm   
        self.PartToLimb(limbs, "R_Forearm", person, 3, 4) # Right Forearm
        self.PartToLimb(limbs, "L_Forearm", person, 6, 7) # left Forearm

        self.PartToLimb(limbs, "NeckToNose", person, 1, 0) 
        self.PartToLimb(limbs, "R_NoseToEye", person, 0, 14) 
        self.PartToLimb(limbs, "L_NoseToEye", person, 0, 15) 
        self.PartToLimb(limbs, "R_EyeToEar", person, 14, 16) 
        self.PartToLimb(limbs, "L_EyeToEar", person, 15, 17) 

        return limbs #abs_pairs

    def PartToJoint(self, joints, joint_key, person, pre_part, joint_part, post_part):
        if(person.body_part[pre_part].confidence > 0.1 and person.body_part[joint_part].confidence > 0.1 and person.body_part[post_part].confidence > 0.1):
            pre_x =   person.body_part[pre_part].x  - person.body_part[joint_part].x 
            pre_y =   person.body_part[pre_part].y  - person.body_part[joint_part].y
            post_x =  person.body_part[post_part].x - person.body_part[joint_part].x
            post_y =  person.body_part[post_part].y - person.body_part[joint_part].y
            joints[joint_key] = -atan2(pre_y, pre_x ) + atan2(post_y, post_x )
            if(joints[joint_key] > pi):    
                joints[joint_key] = joints[joint_key] - 2*pi

    def PartsToJoints(self, person):

        joints = {}

        self.PartToJoint(joints, "R_Shoulder", person, 1, 2, 3 )
        self.PartToJoint(joints, "L_Shoulder", person, 1, 5, 6 )

        self.PartToJoint(joints, "R_Elbow", person, 2, 3, 4 )
        self.PartToJoint(joints, "L_Elbow", person, 5, 6, 7 )

        self.PartToJoint(joints, "R_Hip", person, 1, 8, 9 )
        self.PartToJoint(joints, "L_Hip", person, 1, 11, 12 )

        self.PartToJoint(joints, "R_Knee", person, 8, 9, 10 )
        self.PartToJoint(joints, "L_Knee", person, 11, 12, 13 )

        return joints




    def EstimatePosture(self, limbs, joints, body_part):
        stand_up_score = 0
        sitting_score = 0
        lying_score = 0
        
        #Stand up
        if "R_Thigh" in limbs['abs']:
            if limbs['y']['R_Thigh'] > 2*limbs['x']['R_Thigh']:
                stand_up_score += 0.5
            else:
                stand_up_score -= 0.5
            if "R_Calf" in limbs['abs']:
                if fabs(joints['R_Knee']) > 2.9:
                    stand_up_score += 1
                else:
                    stand_up_score -= 0        

        if "L_Thigh" in limbs['abs']:
            if limbs['y']['L_Thigh'] > 2*limbs['x']['L_Thigh']:
                stand_up_score += 0.5
            else:
                stand_up_score -= 0.5
            if "L_Calf" in limbs['abs']:
                if fabs(joints['L_Knee']) > 2.9:
                    stand_up_score += 1
                else:
                    stand_up_score -= 0    

        if (body_part[RawPoseIndex.Neck ].confidence != 0):
            if body_part[RawPoseIndex.Neck ].y < ( self.image_h / 3 ) :
                stand_up_score += 4

        #Sitting
        if "R_Thigh" and "R_Calf" in limbs['y'] :
            if limbs['y']['R_Calf'] > limbs['y']['R_Thigh']:
                sitting_score += 1
            else:
                sitting_score -= 0.5

            if fabs(joints['R_Knee']) < 2.7:
                sitting_score += (2.7 - fabs(joints['R_Knee']))
            else:
                sitting_score -= 0   

        if "L_Thigh" and "L_Calf" in limbs['y'] :
            if limbs['y']['L_Calf'] > limbs['y']['L_Thigh']:
                sitting_score += 1
            else:
                sitting_score -= 0.5
           
            if fabs(joints['L_Knee']) < 2.7:
                sitting_score += (2.7 - fabs(joints['L_Knee']))
            else:
                sitting_score -= 0        

        #Lying
        if "R_Thigh" in limbs['y']:
            if limbs['x']['R_Thigh'] > 2*limbs['y']['R_Thigh']:
                lying_score += 0.5  

        if "R_Calf" in limbs['y']:
            if limbs['x']['R_Calf'] > 2*limbs['y']['R_Calf']:
                lying_score += 1              

        if "L_Thigh" in limbs['y']:
            if limbs['x']['L_Thigh'] > 2*limbs['y']['L_Thigh']:            
                lying_score += 0.5  

        if "L_Calf" in limbs['y']:
            if limbs['x']['L_Calf'] > 2*limbs['y']['L_Calf']:
                lying_score += 1 

        #i = RawPoseIndex()
        ratio = (2*self.image_h / 3)
        below_score = 0
        if  body_part[RawPoseIndex.R_Eye].confidence > 0.1 :
            if  body_part[RawPoseIndex.R_Eye].y > ratio :
                below_score += 1
        if  body_part[RawPoseIndex.L_Eye].confidence > 0.1 :
            if  body_part[RawPoseIndex.L_Eye].y > ratio :
                below_score += 1
        if  body_part[RawPoseIndex.Nose].confidence > 0.1 :
            if  body_part[RawPoseIndex.Nose].y > ratio :
                below_score += 1
        if  body_part[RawPoseIndex.Neck].confidence > 0.1 :
            if  body_part[RawPoseIndex.Neck].y > ratio :
                below_score += 1
        if  body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
            if  body_part[RawPoseIndex.R_Ear].y > ratio :
                below_score += 1
        if  body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
            if  body_part[RawPoseIndex.L_Ear].y > ratio :
                below_score += 1                                                                                

        if below_score >= 1 :
            lying_score += 6

        


        #print "test: %d" %  RawPoseIndex.L_Eye

        rospy.logdebug( "\tsitting: "   + str(sitting_score) )
        rospy.logdebug( "\tstanding: "  + str(stand_up_score) )
        rospy.logdebug( "\tlying: "     + str(lying_score) )


        if sitting_score > stand_up_score:
            if sitting_score > lying_score:
                return "Sitting"
            elif sitting_score < lying_score:
                return "Lying"
            else:
                return "Undefined"

        elif sitting_score < stand_up_score:            
            if stand_up_score > lying_score:
                return "Standing"
            elif stand_up_score < lying_score:
                return "Lying"
            else:
                return "Undefined"            

        else:
            if stand_up_score < lying_score:
                return "Lying"
            else:
                return "Undefined" 




    def EstimateCallHand(self, limbs, joints, body_part):

        above_score = 0
        callHand_score = 0

        if  body_part[RawPoseIndex.R_Wrist].confidence > 0.1 :

            if body_part[RawPoseIndex.R_Elbow].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Elbow].y > body_part[RawPoseIndex.R_Wrist].y :
                    callHand_score += 1

            if  body_part[RawPoseIndex.R_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Eye].y > body_part[RawPoseIndex.R_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Eye].y > body_part[RawPoseIndex.R_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.Nose].confidence > 0.1 :
                if  body_part[RawPoseIndex.Nose].y > body_part[RawPoseIndex.R_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Ear].y > body_part[RawPoseIndex.R_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Ear].y > body_part[RawPoseIndex.R_Wrist].y :
                    above_score += 1         


        if  body_part[RawPoseIndex.L_Wrist].confidence > 0.1 :
            
            if body_part[RawPoseIndex.L_Elbow].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Elbow].y > body_part[RawPoseIndex.L_Wrist].y :
                    callHand_score += 1

            if  body_part[RawPoseIndex.R_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Eye].y > body_part[RawPoseIndex.L_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Eye].y > body_part[RawPoseIndex.L_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.Nose].confidence > 0.1 :
                if  body_part[RawPoseIndex.Nose].y > body_part[RawPoseIndex.L_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Ear].y > body_part[RawPoseIndex.L_Wrist].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Ear].y > body_part[RawPoseIndex.L_Wrist].y :
                    above_score += 1     

        if above_score > 1 :
            callHand_score += 2
        if callHand_score >= 2 :
            return True
        else:
            return False


    def EstimateHandPosture(self, limbs, joints, body_part):     

        wrist_LR_list = [RawPoseIndex.L_Wrist, RawPoseIndex.R_Wrist]
        hand_status = ["", ""]
        wrist_nb = 0
        ratio_xy = 0.5

        # For each arm : Left, then Right
        for lr, wrist_LR in enumerate(wrist_LR_list) :

            if wrist_LR == RawPoseIndex.L_Wrist :
                shoulder_LR = RawPoseIndex.L_Shoulder
                elbow_LR = RawPoseIndex.L_Elbow
                forearm_LR = 'L_Forearm'
                arm_LR = 'L_Arm'
            else :
                shoulder_LR = RawPoseIndex.R_Shoulder
                elbow_LR = RawPoseIndex.R_Elbow
                forearm_LR = 'R_Forearm'
                arm_LR = 'R_Arm'

            
            if  body_part[wrist_LR].confidence > 0.1 and body_part[elbow_LR].confidence > 0.1 and body_part[shoulder_LR].confidence > 0.1 :
                wrist_nb += 1

                # Test pointing
                if limbs['x'][forearm_LR] > limbs['y'][forearm_LR] : # Forearm mainly with horizontal component ?
                    if limbs['x'][arm_LR] > ratio_xy*limbs['y'][arm_LR] : # Arm mainly with horizontal component ?
                        if body_part[elbow_LR].x > body_part[shoulder_LR].x : # Right side of the robot ?
                            if body_part[wrist_LR].x > body_part[elbow_LR].x :
                                hand_status[lr] = "Pointing Right" # Robot's right
                            else : 
                                hand_status[lr] = "Undefined" # hand on Hips
                        elif body_part[elbow_LR].x < body_part[shoulder_LR].x : # Left side of the robot ?
                            if body_part[wrist_LR].x < body_part[elbow_LR].x :
                                hand_status[lr] = "Pointing Left" # Robot's right
                            else :                             
                                hand_status[lr] = "Undefined" # hand on Hips
                        else :
                            hand_status[lr] = "Undefined"
                    else :
                        hand_status[lr] = "Undefined"

                # Test hand call
                elif body_part[wrist_LR].y < body_part[shoulder_LR].y :
                    hand_status[lr] = "Call"
                elif body_part[RawPoseIndex.Nose].confidence > 0.1 :
                    if body_part[elbow_LR].y < body_part[RawPoseIndex.Nose].y :
                        hand_status[lr] = "Call"
                elif body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
                    if body_part[elbow_LR].y < body_part[RawPoseIndex.R_Ear].y :
                        hand_status[lr] = "Call"
                elif body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
                    if body_part[elbow_LR].y < body_part[RawPoseIndex.L_Ear].y :
                        hand_status[lr] = "Call"
                    
                else :
                    hand_status[lr] = "Undefined"
                    
            else :
                hand_status[lr] = "Undefined"

        # Test arms crossed
        if wrist_nb == 2 : #hand_status[0].find("Pointing") >= 1 and hand_status[1].find("Pointing") >= 1 :
            fa_L = body_part[RawPoseIndex.L_Wrist].x - body_part[RawPoseIndex.L_Shoulder].x
            sh_L = body_part[RawPoseIndex.Neck].x - body_part[RawPoseIndex.L_Shoulder].x
            fa_R = body_part[RawPoseIndex.R_Wrist].x - body_part[RawPoseIndex.R_Shoulder].x
            sh_R = body_part[RawPoseIndex.Neck].x - body_part[RawPoseIndex.R_Shoulder].x            

            if fa_L * sh_L > 0 and fa_R * sh_R > 0 : # Both forearm in the same direction that the soulder to neck vector ?
                #if limbs['x']['L_Forearm'] > limbs['y']['L_Forearm'] and limbs['x']['R_Forearm'] > limbs['y']['R_Forearm'] : # Forearm mainly with horizontal component ?
                hand_status = ["Crossed", "Crossed"]
            

        return hand_status


#    def AbsToLimb(self, limbs, pair_key, value):
#
#        limbs[pair_key] = value
#
#
#        # "Right/Left" means right/left of the person and not of the image
#        self.PartToLimb(limbs, "R_Flank",       10  )     # Right Flank
#        self.PartToLimb(limbs, "L_Flank",       10  )     # Left Flank
#        self.PartToLimb(limbs, "R_Thigh",       10  )     # Right Thigh
#        self.PartToLimb(limbs, "L_Thigh",       10  )     # Left Thigh
#        self.PartToLimb(limbs, "R_Calf",        10  )     # Right Calf
#        self.PartToLimb(limbs, "L_Calf",        10  )     # Left Calf
#        self.PartToLimb(limbs, "R_Shoulder",    10  )     # Right Shoulder
#        self.PartToLimb(limbs, "L_Shoulder",    10  )     # left Shoulder
#        self.PartToLimb(limbs, "R_Arm",         10  )     # Right Arm
#        self.PartToLimb(limbs, "L_Arm",         10  )     # left Arm   
#        self.PartToLimb(limbs, "R_Forearm",     10  )     # Right Forearm
#        self.PartToLimb(limbs, "L_Forearm",     10  )     # left Forearm
#        self.PartToLimb(limbs, "NeckToNose",    10  ) 
#        self.PartToLimb(limbs, "R_NoseToEye",   10  ) 
#        self.PartToLimb(limbs, "L_NoseToEye",   10  ) 
#        self.PartToLimb(limbs, "R_EyeToEar",    10  ) 
#        self.PartToLimb(limbs, "L_EyeToEar",    10  ) 
#
#
#    def MaxAbsLimbFromNormalized(self, limbs):     
#
#        Normal_Abs_Limbs_3m = {}
#        limbs[pair_key] = value
#
#        Normalized_Abs_Limbs = {}
#
#        for limb in limbs:
#            limbs[pair_key] = value
#
#        return 0    




    def SaveLimbsProfil(self, limbs):
        w = csv.writer(open(self.file_name + '.csv', "w"))
        for key, val in limbs.items():
            w.writerow([key, val])                

    def LoadLimbsProfil(self, norm_distance):

        with open(self.calib_folder + 'norm_%sm.csv' % str(norm_distance), mode='r') as infile:
            reader = csv.reader(infile)
            limbs = {rows[0]:float(rows[1]) for rows in reader} 
        
        return limbs         
        

    def RefLimb(self, limbs):
        normalized_limbs = {}
        norm_limbs = self.LoadLimbsProfil(3)
        for key in limbs["abs"]:
            if key != "R_NoseToEye" and key != "L_NoseToEye" and key != "R_EyeToEar" and key != "L_EyeToEar":
                normalized_limbs[key] = ( float(self.image_w) / 1280.0 ) * limbs["abs"][key] / norm_limbs[key]
            
        #del normalized_limbs["R_NoseToEye"]
        #del normalized_limbs["L_NoseToEye"]
        #del normalized_limbs["R_EyeToEar"]
        #del normalized_limbs["L_EyeToEar"]

        #max_key = max(normalized_limbs)
        if not normalized_limbs.items():
            for key in limbs["abs"]:
                if key == "R_NoseToEye" or key == "L_NoseToEye" or key == "R_EyeToEar" or key == "L_EyeToEar":
                    normalized_limbs[key] = (float(self.image_w) / 1280.0) * limbs["abs"][key] / norm_limbs[key]

            if not normalized_limbs.items():
                return None

        return max(normalized_limbs.items(), key=lambda (_,v): v )



        #return (max_key , normalized_limbs[max_key] )



    def EstimateDistance(self, limbs, limb_key):     
        norm_px_profils = {}

        if limb_key is None:
            return 0

        for num in xrange(1,9):
            limbs_norm = self.LoadLimbsProfil(num)
            if limb_key in limbs_norm.keys():
                norm_px_profils[num] = self.LoadLimbsProfil(num)[limb_key] * ( float(self.image_w) / 1280.0 )

        # return pair of normalized value with the closest value to the reference limb
        pair1 = min(norm_px_profils.items(), key=lambda (_, v): abs(v - limbs["abs"][limb_key]))

        del norm_px_profils[ pair1[0] ]

        # return the 2nd pair of normalize value with the closest value to the reference limb
        pair2 = min(norm_px_profils.items(), key=lambda (_, v): abs(v - limbs["abs"][limb_key]))
        
        #determine linear fonction
        if ( (pair2[1] - pair1[1]) != 0.0 ):
            a = (pair2[0] - pair1[0]) / (pair2[1] - pair1[1])
            b = pair1[0] - a * pair1[1]
            return a * limbs["abs"][limb_key] + b
        else:
            return pair1[0]




    def makeBounginBox(self, body_part_limited, limbs):

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
            eyes_to_hair = (limbs['abs']["R_NoseToEye"] + limbs['abs']["L_NoseToEye"]) / 2
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

        rospy.logdebug( "bb: " + str([ TopLeft, DownRight ]) )

        return [ TopLeft, DownRight ]     



    def getBoundingBox(self, body_part, limbs):

        bps = deepcopy(body_part)

        for i in xrange(len(body_part)-1, -1, -1):
            if  bps[i].confidence == 0 \
                or body_part[i].x > self.image_w \
                or body_part[i].y > self.image_h \
                or body_part[i].x < 0 \
                or body_part[i].y < 0 :

                bps.pop(i)

        return self.makeBounginBox(bps, limbs)



    def getHeadRect(self, body_part, limbs):

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

        return self.makeBounginBox(bps, limbs)



    def getShirtRect(self, body_part):

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



    def getTrouserRect(self, body_part, limbs):

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



    def getCam2MapXYPoint(self, neck_x, distance):
        
        HFov = 57.2 * pi / 180.0  # Horizontal field of view of the front Pepper Camera
        #Phi = (HFov / 2.0) * ( (2*neck_x)/self.image_w + 1)  #Angle from the center of the camera to neck_x
        Phi = (HFov / 2.0) *  (neck_x - self.image_w / 2)/(self.image_w/2) #Angle from the center of the camera to neck_x
        return Point32(    x = distance  , y = distance * sin(Phi)      )


    def getOrientation(self, body_part, limbs):

        front = 0
        right = 0
        left = 0
        back = 0
        angle = 0.0
        orientation = ""

        ## If ears 
        #if (body_part[RawPoseIndex.R_Ear ].confidence != 0) and (body_part[RawPoseIndex.L_Ear ].confidence != 0) :
        #    
        #    # If no eyes nor nose
        #    if (body_part[RawPoseIndex.Nose ].confidence == 0) and (body_part[RawPoseIndex.R_Eye ].confidence == 0) and (body_part[RawPoseIndex.R_Eye ].confidence == 0):
        #        back += 1
        #    
        #    # If eyes and nose
        #    elif (body_part[RawPoseIndex.Nose ].confidence != 0) and (body_part[RawPoseIndex.R_Eye ].confidence != 0) and (body_part[RawPoseIndex.R_Eye ].confidence != 0):
        #        #R = limbs['abs']["R_EyeToEar"]               
        #        #L = limbs['abs']["L_EyeToEar"]
        #        #e = abs(R-L)/R
        #        #if e < 0.2 :
        #        front += 1

        if (body_part[RawPoseIndex.Nose ].confidence != 0) :
            front += 1
        else :
            back += 0.5
        
        if (body_part[RawPoseIndex.L_Shoulder].confidence !=0) and (body_part[RawPoseIndex.R_Shoulder].confidence !=0) :
            if body_part[RawPoseIndex.L_Shoulder].x < body_part[RawPoseIndex.R_Shoulder].x :
                back += 1
            else :
                front += 1

        R_confidence = body_part[RawPoseIndex.R_Ear].confidence + body_part[RawPoseIndex.R_Eye].confidence + body_part[RawPoseIndex.R_Shoulder].confidence
        L_confidence = body_part[RawPoseIndex.L_Ear].confidence + body_part[RawPoseIndex.L_Eye].confidence + body_part[RawPoseIndex.L_Shoulder].confidence
        
        if R_confidence + L_confidence > 0.1 :
            e = 2 * (R_confidence - L_confidence) / (R_confidence + L_confidence)
        else :
            e = 0

        if e > 0.1 :
            right += e * 45.0
        elif  e < -0.1 :
            left += -e * 45.0

        if front > back :
            orientation = "Front"   
            angle = 180 - min(left, 90) + min(right, 90)         
        elif front < back :
            orientation = "Back"
            angle = min(left, 90) - min(right, 90)
        else:
            angle = angle + min(left, 90) - min(right, 90)        
        
        if right >= 10:
            if orientation != "":
                orientation += " "
            orientation += "Right"

        if left >= 10:
            if orientation != "":
                orientation += " "
            orientation += "Left"   

        if orientation == "":
            orientation = "undefined"

        return (orientation, angle) 



    def faceConfidence(self, body_part):

        confidence =    \
                        (body_part[RawPoseIndex.R_Ear ].confidence  \
                        + body_part[RawPoseIndex.L_Ear ].confidence \
                        + body_part[RawPoseIndex.Nose ].confidence  \
                        + body_part[RawPoseIndex.R_Eye ].confidence \
                        + body_part[RawPoseIndex.L_Eye ].confidence) / 5

        return confidence



    def getPoseOnTheGround(self, neck_x, distance, persAngle):
        
            HFov = 57.2 * pi / 180.0  # Horizontal field of view of the front Pepper Camera
            #Phi = (HFov / 2.0) * ( (2*neck_x)/self.image_w + 1)  #Angle from the center of the camera to neck_x
            Phi = (HFov / 2.0) *  (neck_x - self.image_w / 2)/(self.image_w/2) #Angle from the center of the camera to neck_x

            p = Pose()
            p.position.x = distance  
            p.position.y = -distance * sin(Phi)  
            p.orientation = Quaternion( *quaternion_from_euler(0, 0, persAngle*pi/180.0) )

            return p



    def EnrichPersonsData(self, persons):
        self.image_w = persons.image_w #resp3.personList.image_w
        self.image_h = persons.image_h #resp3.personList.image_h

        #persons = sorted(persons.persons, key = lambda person : person.body_part[RawPoseIndex.Neck].x)  
        personsEnriched = []
        pgs = PersonsGossip()


        for num, person in enumerate(persons.persons):

            limbs = self.PartsToLimbs(person)
            joints = self.PartsToJoints(person)

            rospy.logdebug( "Person " + str(num) )

            posture = self.EstimatePosture(limbs, joints, person.body_part)
            
            handPosture = self.EstimateHandPosture(limbs, joints, person.body_part)    #self.EstimateCallHand(limbs, joints, person.body_part)

            tuppRefLimb = self.RefLimb(limbs)

            distance = self.EstimateDistance(limbs, tuppRefLimb[0])

            #print "\tPosture:\t" + posture
            #print "\tCall hand:\t" + str(callHand)
            #print "\tDistance:\t" 
            

            #Cam2MapXYPoint = self.getCam2MapXYPoint(person.body_part[RawPoseIndex.Neck].x, distance)

            #poseOnTheGround = self.getPoseOnTheGround(person.body_part[RawPoseIndex.Neck].x, distance, self.getOrientation(person.body_part, limbs)[1] )


            personsEnriched.append((person.body_part, limbs, joints, posture, handPosture, distance))
            #personsEnriched.append((person.body_part, limbs, joints, posture, handPosture, distance))

        personsEnrichedSorted = sorted(personsEnriched, key=lambda attributes: attributes[0][RawPoseIndex.Neck].x)   # sort by     

        

       # for num, personEnriched in enumerate(personsEnrichedSorted):  
        #    norm_limbs = LoadLimbsProfil(num)
        

        for num, personEnriched in enumerate(personsEnrichedSorted):  
            pg = PersonGossip()

            rospy.logdebug( "Person " + str(num) )
            rospy.logdebug( "\tPosture:\t" + personEnriched[3] )
            rospy.logdebug( "\tCall hand:\t" + str(personEnriched[4]) )

            rospy.logdebug( "\tDistance:\t" + str(personEnriched[5]) )      

            #print "\tCam2Map_XY:\t" + str(personEnriched[6] )   

            pg.id = num
            pg.posture = personEnriched[3]
            #pg.handCall = None
            pg.handPosture = personEnriched[4]
            pg.boundingBox.points = self.getBoundingBox(personEnriched[0], personEnriched[1])
            pg.headRect.points = self.getHeadRect(personEnriched[0], personEnriched[1])
            pg.shirtRect.points = self.getShirtRect(personEnriched[0])
            pg.trouserRect.points = self.getTrouserRect(personEnriched[0], personEnriched[1])

            rospy.logdebug( "\tBody Bounding Box:\t" + str(pg.boundingBox.points  ) )
            rospy.logdebug( "\tHead Bounding Box:\t" + str(pg.headRect.points  ) )
            rospy.logdebug( "\tShirt Sample:\t" + str(pg.shirtRect.points  ) )
            rospy.logdebug( "\Trousers Sample:\t" + str(pg.trouserRect.points  ) )

            #pg.shirtRect = tuppRefLimb
            #pg.trouserRect = 
            pg.distanceEval = personEnriched[5]       

            pg.faceConfidence = self.faceConfidence(personEnriched[0]) 
            rospy.logdebug( "\Face COnfidence:\t" + str(pg.faceConfidence ) )

            pg.orientation, persAngle = self.getOrientation(personEnriched[0], personEnriched[1])


            #poseOnTheGround = 
            pg.pose = self.getPoseOnTheGround(personEnriched[0][RawPoseIndex.Neck].x, pg.distanceEval, persAngle)
            
            rospy.logdebug( "\orientation:\t" + str(pg.orientation ) + " --> " + str(persAngle) + " deg" )

            rospy.logdebug( "#########\n" + str(self.image_w) + "\n#########" )

            pgs.personsGossip.append(pg)
        
        return pgs
            
        #self.SaveLimbsProfil(limbs["abs"])

        
                
        # spin
        rospy.spin()

if __name__ == '__main__':
    try:
        #LoadImgAndPublish()
        OpenPoseGossip()
    except rospy.ROSInterruptException:
        pass
