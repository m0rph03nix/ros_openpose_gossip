#! /usr/bin/env python
__author__ ='Raphael Leber'

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from openpose_ros_srvs.srv import DetectPeoplePoseFromImg
from ros_openpose_gossip_srvs.srv import OpenPoseGossip as OPG_Srv

import rospkg



def LoadImgAndCallGossip():
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    package_path=rospack.get_path('ros_openpose_gossip_examples')
    
    _bridge = CvBridge()
    
    media_folder=package_path+'/media'
    rospy.loginfo("media_folder:"+str(media_folder))
    #print "--> media_folder:"+str(media_folder) +'/moris_1m.jpg'


    rospy.init_node('LoadAndPublishImg', anonymous=True)

    img_loaded = cv2.imread(media_folder+'/group-diff-position.jpg')

    #img_loaded = cv2.LoadImage(media_folder+'/moris.jpg', CV_LOAD_IMAGE_COLOR)

    print "--> " + str(img_loaded)

    #msg_img = _bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
    msg_img = _bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")

    print "import image OK !"

    #call service to learn people
    rospy.wait_for_service('people_pose_from_img')

    try:
        detect_from_img_srv = rospy.ServiceProxy('people_pose_from_img', DetectPeoplePoseFromImg)
        resp = detect_from_img_srv(msg_img)
        print "nb people"
        print "service:" + str(resp.personList)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    try:
        gossip_srv = rospy.ServiceProxy('openpose_gossip_srv',  OPG_Srv)
        resp.personList.image_w = msg_img.width
        resp.personList.image_h = msg_img.height        
        resp_g = gossip_srv(resp.personList)
        print "gossip"
        print "service:" + str(resp_g.personsGossip)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e        
    

    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        LoadImgAndCallGossip()
    except rospy.ROSInterruptException:
        pass
