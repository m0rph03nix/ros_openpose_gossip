int32                   id              # num of the person in the image (from left to right)
string                  posture         # Possible values: "Standing" "Sitting" "Lying" "Undefined"
#bool                    handCall        # Possible values: True False
string[]                handPosture     # 1st string : left hand value, 2nd string : right hand value
                                        # Values : "Undefined" "Call" "Pointing Left" "Crossed" "Pointing Right" (/!\ robot's left or right) 
geometry_msgs/Polygon   boundingBox     # coordinates of a person bounding box
geometry_msgs/Polygon   headRect        # coordinates of a person's head bounding box
geometry_msgs/Polygon   shirtRect       # coordinates of shirt rectangle sample 
geometry_msgs/Polygon   trouserRect     # coordinates of trouser rectangle sample
float32                 distanceEval    # Distance evaluation in meters from 2D informations
geometry_msgs/Point32   Cam2MapXYPoint  # XY coordinates of the person's neck, parallel to the map  
float32                 faceConfidence  # Face confidence
string                  orientation     # person orientation : "Front" "Back" "Right" "Left" "Front Right" "Front Left"
