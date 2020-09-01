/*
 * visualization.h
 *
 *  Created on: Dec 3, 2018
 *      Author: philipf
 *
 *  Class for generating a visual feedback of the detection results. The results will be published and send to the gui.
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_VISUALIZATION_VISUALIZATION_GUI_TXT_
#define SRC_DEXTERITY_DETECTION_INCLUDE_VISUALIZATION_VISUALIZATION_GUI_TXT_

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <handling_msgs/VisualCloudUpdate.h>
#include <geometry_msgs/Vector3.h>

#include "../Segment.h"
#include "../Model.h"
#include "../Blackboard.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class Visualization
{
	public:
		Visualization();
		~Visualization();
		/*Visualization needs to know the Blackboard to get a pointer to all models and segments*/
		void init(std::shared_ptr<Blackboard> gBoard, std::shared_ptr<ros::NodeHandle> nh_ptr, double gPublishRate=1.0);
		/*Update the Visualization. It will find out if new models/segments/results have to be drawn*/
		void spinOnce();
		/*Redraws the learned models.*/
		void drawModels();
		/*Each model will be placed with an offset next to the other model. You can get the offset here.*/
		tf::Vector3 getModelOffset();

		bool forceRedraw = false;

	protected:
		//Parameters to configure the viewer
		RosParameter<bool>::Ptr showKeypoints;
		RosParameter<bool>::Ptr showNormals;
		RosParameter<bool>::Ptr showDense;
		RosParameter<bool>::Ptr showCorrespondences;
		RosParameter<bool>::Ptr showEstimatedObjects;
		RosParameter<bool>::Ptr showReferenceFrames;
		RosParameter<bool>::Ptr showAllCorrespondences;
		RosParameter<bool>::Ptr showMatchingColor;
		RosParameter<bool>::Ptr showOnlyWinner;
		RosParameter<double>::Ptr denseSize;
		RosParameter<double>::Ptr keypointSize;
		RosParameter<double>::Ptr estimationSize;

		tf::Vector3 modelOffset = tf::Vector3(0.2,0,0.25);
		bool dirty = true;

	private:
		/*Clears all data regarding segments. Model data will stay valid!*/
		void removeAllSegments();
		/*Draws all segments.*/
		void drawCloud(ProcessedCloud *data, bool isModel, unsigned int id, tf::Vector3 modelOffset = tf::Vector3(0,0,0));
		/*Draws all correspondences.*/
		void drawAllCorrespondences(Segment *seg, std::string id);
		/*Only draws correpondences which resulted into a guess.*/
		void drawCorrespondences(Segment *seg, Result *result, Model *model, std::string id);
		/*Creates a visualCloudUpdate message which is published to the gui.*/
		void checkAndPublishMessage();

		/*Storage for Models and Segments.*/
		std::shared_ptr<Blackboard> board;
		std::shared_ptr<ros::NodeHandle> nh;
		ros::Publisher visPub;
		ros::Time lastPublishTime;
		double publishRate = 1.0;

		/*Store the infomration for the next messagees. This makes it easy to replace/modify.*/

		pcl::PointCloud<PointType> models;
		pcl::PointCloud<PointType> modelKeypoints;
		pcl::PointCloud<NormalType> modelNormals;
		pcl::PointCloud<RFType> modelRFs;
		unsigned long sequendeIdModels;

		pcl::PointCloud<PointType> estimatedObjects;
		pcl::PointCloud<PointType> scene;
		pcl::PointCloud<PointType> sceneKeypoints;
		pcl::PointCloud<NormalType> sceneNormals;
		pcl::PointCloud<RFType> sceneRFs;
		unsigned long sequendeIdScene;

		std::vector<geometry_msgs::Vector3> pointA;
		std::vector<geometry_msgs::Vector3> pointB;
		std::vector<geometry_msgs::Vector3> lineColor;

		/*Remember the last drawn state. Used for giving different semgents different colors.*/
		unsigned long counter = 42;
		int modelOffsetCounter = 0;
		unsigned long lastDrawnState = 0;
		double r,g,b;
		long colorCounter;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_VISUALIZATION_VISUALIZATION_H_ */
