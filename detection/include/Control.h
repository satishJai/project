/*
 * Control.h
 *
 *  Created on: Nov 20, 2018
 *      Author: philipf & satishj
 *
 *  Control is the interface to the 3d object detection system from the dexterity project group SS2018/WS2018-19.
 *  First lean models, then regognize them inside a given scene. If you want to change parameters, you can access them on the ros parameter server.
 *  After parameters have been changed, the /updateParameters service has to be called, which will trigger the retrain process if a new scene is recognized.
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_CONTROL_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_CONTROL_H_
#include "Blackboard.h"
#include <pcl/point_cloud.h>
#include "keypoint/KeypointCreator.h"
#include "keypoint/VoxelGridCreator.h"
#include "Result.h"
#include "Model.h"
#include "ProcessedCloud.h"
#include "Segment.h"
#include "segmentation/SegmentsCreator.h"
#include "RosParameter.h"
#include "RosParameterManager.h"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <handling_msgs/Trigger.h>
#include <handling_msgs/SetSource.h>
#include <handling_msgs/GetFeatures.h>
#include <handling_msgs/ReadFeatures.h>
#include "visualization/visualization.h"

class Control
{
	public:
		Control(bool visualization=true, bool externalDatabase=false, bool enableCheatSegmentation=false);
		//Used to learn a given pointcloud.
		bool learnModel(const pcl::PointCloud<PointType>& cloud, const std::string modelName, const unsigned int modelId);
		//Used to learn a pointcloud which is stored on the hard disk.
		bool learnModel(const std::string file, const std::string modelName, const unsigned int modelId);
		/*
		 * Called internally when parameters have been changed and a new scene should be recognized.
		 * @param forceRetrain Has to be set to true, if all models should be retrained. Otherwise only notValid models will be retrained.
		 * All models will become notValid, if a updateParametes callback has been received.
		 */
		void checkForRetrain(bool forceRetrain=false);
		//Recognizes all objects in the given cloud. Only recognized objects which have been learned before.
		void recognize(const pcl::PointCloud<PointType>& cloud, std::vector<Result> &results);
		//Gives all correspondences mapped to each object. Used for evaluation
		void evaluateDescriptors(const pcl::PointCloud<PointType>& cloud, std::map<std::string,std::vector<double>> &results, unsigned long &numKeypoints);
		//Recognizes all objects in the given cloud which is stored on the hard disk. Only recognized objects which have been learned before.
		bool recognize(const std::string file, std::vector<Result> &results);
		//Updates the visualization
		void spinViewer();
		/*
		 * Called from gui when /detection/database/models parameter has been updated.
		 * New models will be loaded and inactive models will be deleted.
		 */
		bool updateModelsCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		// Called when visualization parameters have been updated.
		bool updateVisualsCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		//Called from database when features are requested to save a profile
		bool getFreaturesCB(handling_msgs::GetFeatures::Request &req, handling_msgs::GetFeatures::Response &res);
		//Called if the ros parameters have been changed.
		bool settingsChangedCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		bool updateParamCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	protected:
		//Names of the currenlty used descriptors, keypoints...
		RosParameter<std::string>::Ptr descriptorCreatorName;
		RosParameter<std::string>::Ptr keypointCreatorName;
		RosParameter<std::string>::Ptr poseEstimatorName;
		RosParameter<std::string>::Ptr matchingName;
		RosParameter<std::string>::Ptr evaluationName;

	private:
		//Reads the ros parameters for the algorithm classes and sets them in the blackboard
		void setAlgorithmClasses();
		/* Reads the new features for the given models from database. If a models is unknown for the detection system, it will be added.*/
		bool readFeaturesFromDatabase(std::vector<std::string> models, std::string profileId);

		//Blackboard is storing pointclouds and the used algorithm objects.
		std::shared_ptr<Blackboard> board;
		ros::NodeHandle nh;
		//Shows the current data inside the blackboard.
		std::shared_ptr<Visualization> visualizer;
		bool allowVisualization = true;
		bool externalDatabaseEnabled = false;
		//Used for evaluation when segmentation should be disabled
		bool cheatSegmentation = false;
		ros::ServiceServer getFeaturesServer;
		ros::ServiceServer updatedetectionParamsServer;
		ros::ServiceServer updatedParamsServer;
		ros::ServiceServer updateModelsParamsServer;
		ros::ServiceServer updateVisualParamsServer;
		std::string latestProfileId = "";
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_CONTROL_H_ */
