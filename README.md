# ros_openpose_gossip


**Launch the openpose service**
    roslaunch openpose_ros_node serviceReadyTest.launch  

**Run the Gossip service**
    rosrun ros_openpose_gossip_node OpenPoseGossip_node.py  

**You can try the Gossip service with one of the following test or do your own call :**
    roslaunch ros_openpose_gossip_examples test_on_pepper.launch  
    roslaunch ros_openpose_gossip_examples test_on_webcam.launch  
    roslaunch ros_openpose_gossip_examples test_with_file.launch  
