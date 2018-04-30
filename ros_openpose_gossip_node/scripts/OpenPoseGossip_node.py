#!/usr/bin/env python  

__author__ ='Raphael Leber'


import rospy 
import actionlib
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
from skimage import color
import numpy as np

from ros_openpose_gossip_msgs.msg import PersonGossip, PersonGossip
#from ros_openpose_gossip_actions.msg import *
from ros_openpose_gossip_srvs.srv import OpenPoseGossip as OPG_Srv

from process.RawPoseIndex import RawPoseIndex
from process.OpenPoseGossip import OpenPoseGossip as OPG


class OpenPoseGossip_node():

    def __init__(self):
        rospy.init_node('openspose_gossip_node', anonymous=False)

        #declare ros service 
        self.detectColorSrv = rospy.Service('openpose_gossip_srv', OPG_Srv, self.OPG_SrvCallback)

        rospy.spin()



    def OPG_SrvCallback(self,req):
        return OPG.EnrichPersonsData(req.personList)


def main():
    #""" main function
    #"""
    node = OpenPoseGossip_node()

if __name__ == '__main__':
    main()