#! /usr/bin/env python
__author__ = 'Raphael LEBER'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from openpose_ros_msgs.msg import Persons, PersonDetection, BodyPartDetection
from ros_openpose_gossip_msgs.msg import PersonGossip, PersonsGossip
from math import sqrt, pow, fabs, atan2, pi
from enum import IntEnum
from RawPoseIndex import RawPoseIndex
import csv

import rospkg




# Node example class.
class OpenPoseGossip():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        self.image_w = 0
        self.image_h = 0

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

        #Sitting
        if "R_Thigh" and "R_Calf" in limbs['abs'] :
            if limbs['y']['R_Calf'] > limbs['y']['R_Thigh']:
                sitting_score += 1
            else:
                sitting_score -= 0.5

            if fabs(joints['R_Knee']) < 2.7:
                sitting_score += (2.7 - fabs(joints['R_Knee']))
            else:
                sitting_score -= 0   

        if "L_Thigh" and "L_Calf" in limbs['abs'] :
            if limbs['y']['L_Calf'] > limbs['y']['L_Thigh']:
                sitting_score += 1
            else:
                sitting_score -= 0.5
           
            if fabs(joints['L_Knee']) < 2.7:
                sitting_score += (2.7 - fabs(joints['L_Knee']))
            else:
                sitting_score -= 0        

        #Lying
        if "R_Thigh" in limbs['abs']:
            if limbs['x']['R_Thigh'] > 2*limbs['y']['R_Thigh']:
                lying_score += 0.5  

        if "R_Calf" in limbs['abs']:
            if limbs['x']['R_Calf'] > 2*limbs['y']['R_Calf']:
                lying_score += 1              

        if "L_Thigh" in limbs['abs']:
            if limbs['x']['L_Thigh'] > 2*limbs['y']['L_Thigh']:            
                lying_score += 0.5  

        if "L_Calf" in limbs['abs']:
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

        print "\tsitting: "   + str(sitting_score)
        print "\tstanding: "  + str(stand_up_score)
        print "\tlying: "     + str(lying_score)


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

        if  body_part[RawPoseIndex.R_Ankle].confidence > 0.1 :

            if body_part[RawPoseIndex.R_Wrist].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Wrist].y > body_part[RawPoseIndex.R_Ankle].y :
                    callHand_score += 1

            if  body_part[RawPoseIndex.R_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Eye].y > body_part[RawPoseIndex.R_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Eye].y > body_part[RawPoseIndex.R_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.Nose].confidence > 0.1 :
                if  body_part[RawPoseIndex.Nose].y > body_part[RawPoseIndex.R_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Ear].y > body_part[RawPoseIndex.R_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Ear].y > body_part[RawPoseIndex.R_Ankle].y :
                    above_score += 1         


        if  body_part[RawPoseIndex.L_Ankle].confidence > 0.1 :
            
            if body_part[RawPoseIndex.L_Wrist].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Wrist].y > body_part[RawPoseIndex.L_Ankle].y :
                    callHand_score += 1

            if  body_part[RawPoseIndex.R_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Eye].y > body_part[RawPoseIndex.L_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Eye].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Eye].y > body_part[RawPoseIndex.L_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.Nose].confidence > 0.1 :
                if  body_part[RawPoseIndex.Nose].y > body_part[RawPoseIndex.L_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.R_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.R_Ear].y > body_part[RawPoseIndex.L_Ankle].y :
                    above_score += 1
            if  body_part[RawPoseIndex.L_Ear].confidence > 0.1 :
                if  body_part[RawPoseIndex.L_Ear].y > body_part[RawPoseIndex.L_Ankle].y :
                    above_score += 1     

        if above_score > 1 :
            callHand_score += 2
        if callHand_score >= 2 :
            return True
        else:
            return False


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

        with open('norm_%sm.csv' % str(norm_distance), mode='r') as infile:
            reader = csv.reader(infile)
            limbs = {rows[0]:float(rows[1]) for rows in reader} 
        
        return limbs         
        

    def RefLimb(self, limbs):
        normalized_limbs = {}
        norm_limbs = self.LoadLimbsProfil(3)
        for key in limbs["abs"]:
            normalized_limbs[key] = (1280.0 / float(self.image_w)) * limbs["abs"][key] / norm_limbs[key]
  
        max_key = max(normalized_limbs) 

        return (max_key , normalized_limbs[max_key] )



    def EstimateDistance(self, limbs, limb_key):     
        norm_px_profils = {}

        for num in xrange(1,9):
            limbs_norm = self.LoadLimbsProfil(num)
            if limb_key in limbs_norm.keys():
                norm_px_profils[num] = self.LoadLimbsProfil(num)[limb_key] * (1280.0 / float(self.image_w))

        # return pair of normalized value with the closest value to the reference limb
        pair1 = min(norm_px_profils.items(), key=lambda (_, v): abs(v - limbs["abs"][limb_key]))

        del norm_px_profils[ pair1[0] ]

        # return the 2nd pair of normalize value with the closest value to the reference limb
        pair2 = min(norm_px_profils.items(), key=lambda (_, v): abs(v - limbs["abs"][limb_key]))
        
        #determine linear fonction
        a = (pair2[0] - pair1[0]) / (pair2[1] - pair1[1])
        b = pair1[0] - a * pair1[1]

        return a * limbs["abs"][limb_key] + b



    def EnrichPersonsData(self, persons):
        self.image_w = persons.image_w #resp3.personList.image_w
        self.image_h = persons.image_h #resp3.personList.image_h

        #persons = sorted(persons.persons, key = lambda person : person.body_part[RawPoseIndex.Neck].x)  
        personsEnriched = []
        pgs = PersonsGossip()

        for num, person in enumerate(persons.persons):

            limbs = self.PartsToLimbs(person)
            joints = self.PartsToJoints(person)

            print "Person " + str(num) #+ " at " + str(float(person.body_part[RawPoseIndex.Neck].x)) + " %"

            posture = self.EstimatePosture(limbs, joints, person.body_part)
            
            callHand = self.EstimateCallHand(limbs, joints, person.body_part)

            tuppRefLimb = self.RefLimb(limbs)

            distance = self.EstimateDistance(limbs, tuppRefLimb[0])

            #print "\tPosture:\t" + posture
            #print "\tCall hand:\t" + str(callHand)
            #print "\tDistance:\t" 
            
            pg = PersonGossip()
            

            pg.posture = posture
            pg.handCall = callHand
            #pg.shirtRect = tuppRefLimb
            #pg.trouserRect = 
            pg.distanceEval = distance

# string                  posture         # Possible values: Standing Sitting Lying Undefined
# bool                    handCall        # Possible valuses: True False
# geometry_msgs/Polygon   shirtRect       # coordinates of shirt rectangle sample 
# geometry_msgs/Polygon   trouserRect     # coordinates of trouser rectangle sample
# float32                 distanceEval    # Distance evaluation in meters from 2D informations            

            pgs.append(pg)

            personsEnriched.append((person.body_part, limbs, joints, posture, callHand, distance))

        personsEnrichedSorted = sorted(personsEnriched, key=lambda attributes: attributes[0][RawPoseIndex.Neck].x)   # sort by     

        return pgs

       # for num, personEnriched in enumerate(personsEnrichedSorted):  
        #    norm_limbs = LoadLimbsProfil(num)
        

        for num, personEnriched in enumerate(personsEnrichedSorted):     
            print "Person " + str(num) 
            print "\tPosture:\t" + personEnriched[3]
            print "\tCall hand:\t" + str(personEnriched[4]  )
            print "\tDistance:\t" + str(personEnriched[5]   )        
            
        #self.SaveLimbsProfil(limbs["abs"])

        
                
        # spin
        rospy.spin()

if __name__ == '__main__':
    try:
        #LoadImgAndPublish()
        OpenPoseGossip()
    except rospy.ROSInterruptException:
        pass
