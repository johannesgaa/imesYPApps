# object_finder.msg

# for legacy Reasons:
float32[] objectXposition
float32[] objectYposition
# new message Format:
int32 objectsFound
string[] objectNames
nav_msgs/Path objectPoses
float32 featureCompliance