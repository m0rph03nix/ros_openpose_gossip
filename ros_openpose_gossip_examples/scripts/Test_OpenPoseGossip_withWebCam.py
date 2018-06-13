#! /usr/bin/env python
__author__ ='Raphael Leber'
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_openpose_gossip_srvs.srv import OpenPoseGossip as OPG_Srv

from naoqi_access.head_mvt import head_mvt

import rospkg

class Test_OpenPoseGossip():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.ready = 0
        self.service_running = 1

        #self._naoqi_headMvt = head_mvt("pepper3.local",9559)

        



    def img_callback(self, msg_img):
        #rospy.loginfo("image")

        if self.ready == 1 :
            if self.service_running == 0:
                self.msg_img = msg_img

    


    def LoadImgAndCallGossip(self):
        

        rospy.init_node('Test_OpenPoseGossip', anonymous=True)

        #rospy.Subscriber("/pepper_robot/naoqi_driver/camera/front/image_raw", Image, self.img_callback)

        rospy.Subscriber("/videofile/image_raw", Image, self.img_callback)


        #call service to learn people
        rospy.wait_for_service('people_pose_from_img')

        #self._naoqi_headMvt.enableAwareness(False)

        self.service_running = 0
        self.ready = 1

        rospy.loginfo("Tapez entrer pour test l'image courante du pepper")
        rospy.loginfo("Tapez q puis entrer pour quitter")

        while( "q" != raw_input("Make another picture ? Enter to continue. q + Enter to quit") ) and not rospy.is_shutdown():

            self.service_running = 1

            #self._naoqi_headMvt.look_straight()

            try:
                detect_from_img_srv = rospy.ServiceProxy('people_pose_from_img', DetectPeoplePoseFromImg)
                resp = detect_from_img_srv(self.msg_img)
                print "nb people"
                print "service:" + str(resp.personList)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            try:
                gossip_srv = rospy.ServiceProxy('openpose_gossip_srv',  OPG_Srv)
                resp.personList.image_w = self.msg_img.width
                resp.personList.image_h = self.msg_img.height        
                resp_g = gossip_srv(resp.personList)
                print "gossip"
                print "service:" + str(resp_g.personsGossip)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e       

            self.service_running = 0    

        
        

        # spin
        rospy.spin()

        #self._naoqi_headMvt.enableAwareness(True)



if __name__ == '__main__':
    try:
        test_opg = Test_OpenPoseGossip()
        test_opg.LoadImgAndCallGossip()
    except rospy.ROSInterruptException:
        pass



