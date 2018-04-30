int32                   id              # num of the person in the image (from left to right)
string                  posture         # Possible values: Standing Sitting Lying Undefined
bool                    handCall        # Possible valuses: True False
geometry_msgs/Polygon   shirtRect       # coordinates of shirt rectangle sample 
geometry_msgs/Polygon   trouserRect     # coordinates of trouser rectangle sample
float32                 distanceEval    # Distance evaluation in meters from 2D informations
