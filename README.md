#Description:
This package provides an easy to extend system for detection objects. It can use multiple keypoint types, descriptor types, correspondence clustering types and more. All parameters can be changed by using ros parameters and calling the update service afterwards. It can use .pcd files, simulator or xtion/kinect as an input. The DetectionWidget in combination with the detection node allows easy usage of profiles for parameter management. It comes with an GUI and database.
#detection:
This is the main node used. It has to be used together with the DetectionWidget and the database. You can start both by using:

	roslaunch detection detection.launch

#main:
Use the main node if you want to run the detection example which subscribes to the kinect camera. The models will be automatically taken from the handling_recogntion/models folder. It will publish marker messages to rviz and send out handling_msgs/detectedObjects which then can be used for futher applications.
#main2:
Use the main2 node if you want to use alredy recorded scenes as an input. The scenes will be automatically taken from handling_recogntion/scenes. The models will be automatically taken from handling_recogntion/models.
#Evaluation:
Use evaluation to measure the current performance of the detection system. It is there to try out different parameters and direclty see how the system behaves. The scenes and groundturh will be automatically taken from handling_recogntion/scenes. The models will be automatically taken from handling_recogntion/models. Results will be put into handling_recognition/evalu. Use handling_scan/groundTruthCreator to create groundtruth files.
#Evaluation2:
Uses evaluation to measure the current performance of descriptors and matching. If works similar to evaluation, except that it does not use pose estimation at all.
#DetectionWidget:
Used in combination with detection and database. Should be launched by using:

	roslaunch detection detection.launch