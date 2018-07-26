# ros_openpose_gossip

## 1. Description
This package gives information on people based on openpose messages 

[comment]: <> (ajouter image)

openpose_gossip gives informations on people sorted from left to right such as :
<pre>
- id                # num of the person in the image (from left to right)
- posture           # Possible values: "Standing" "Sitting" "Lying" "Undefined"
- handPosture       # Values : "Undefined" "Call" "Pointing Left" "Crossed" "Pointing Right" 
- boundingBox       # coordinates of a person bounding box
- shirtRect         # coordinates of shirt rectangle sample 
- trouserRect       # coordinates of trouser rectangle sample
- distanceEval      # Distance evaluation in meters from 2D informations (need calibration)
</pre>

This program was done for the pepper robot (front camera) with some specifications :
- /!\ **posture** needs camera to be parallel to the ground
- /!\ **distanceEval** needs a calibration from csv files (see 5.  Calibration)

## 2. Authors
* Raphaël LEBER

## 3.  Dependencies
[openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
[ros_openpose (fork)](https://github.com/jacques-saraydaryan/ros-openpose.git)

## 4.  Quick start

### 41 Launch the openpose service
&nbsp;&nbsp; roslaunch openpose_ros_node serviceReadyTest.launch  

### 4.2 Run the Gossip service
&nbsp;&nbsp; rosrun ros_openpose_gossip_node OpenPoseGossip_node.py  

### 4.3 You can try the Gossip service with one of the following test or do your own call : 
&nbsp;&nbsp; roslaunch ros_openpose_gossip_examples test_on_pepper.launch  
&nbsp;&nbsp; roslaunch ros_openpose_gossip_examples test_on_webcam.launch  
&nbsp;&nbsp; roslaunch ros_openpose_gossip_examples test_with_file.launch  

## 5.  Calibration
If you use this package with another camera than the front camera of the pepper, you will need to do a calibration. It consists of recording the limbs length of an "average" person, in a standing position, maximizing (with a natural front pose) each limbs length. Function SaveLimbsProfil can be used but is not implemented in a calibration program (TODO).
