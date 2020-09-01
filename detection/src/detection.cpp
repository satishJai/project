/*
 * main.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: philipf
 *
 *  The detection node should be used together with the DetectionWidget. It offers services and functionalities to control
 *  the detection system from a gui. It can run in trigger and continuinus mode. It can work on a PointCloud stream and on files.
 */

#include <iostream>
#include <stdlib.h>
#include <memory>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <handling_msgs/DetectedObjects.h>
#include <handling_msgs/DetectedObject.h>
#include <handling_msgs/ReadFeatures.h>

#include "../include/Control.h"

enum class RecordMode{
	TRIGGER,CONTINUOUS
};
enum class PointCloudSource{
		FILESYSTEM,TOPIC
};

RecordMode mode = RecordMode::TRIGGER;
PointCloudSource source = PointCloudSource::TOPIC;

std::shared_ptr<tf::TransformListener> tf_listener; //Will convert subscribed pointclouds into the frameId frame!
std::string topicName = "/camera/depth_registered/points";
std::string fileName;
std::string frameId = "/world";

std::shared_ptr<ros::NodeHandle> nh;
ros::Subscriber sub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
ros::Publisher marker_pub;

bool dirtyCloud = false;
bool recording = false;
std::vector<Result> results;

/* Callback from camera.*/
void pointcloudCallback (const pcl::PointCloud<PointType>::ConstPtr& pointcloud)
{
	if(!recording) return;
	if(dirtyCloud) return;
	if(source == PointCloudSource::FILESYSTEM) return;

	std::cout << "PointCloud callback!\n";
	tf::StampedTransform transform;
	try{
		tf_listener->waitForTransform(frameId,pointcloud->header.frame_id,ros::Time(0),ros::Duration(4.0));
		pcl_ros::transformPointCloud(frameId,*pointcloud,*cloud,*tf_listener);
		dirtyCloud = true;
		recording = false;
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("[Detection]: pointcloudCallback: %s", ex.what ());
		ros::Duration (1.0).sleep ();
	}
}
/* Callback triggered by GUI for detecting one single time.*/
bool triggerDetectCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	mode = RecordMode::TRIGGER;
	recording = true;
	std::cout << "Trigger!\n";
	return true;
}
/* Callback triggered by GUI for continuous detection. */
bool continuousDetectCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if(req.data)
	{
		mode = RecordMode::CONTINUOUS;
		recording = true;
	}else{
		std::cout << "Switch to trigger.\n";
		mode = RecordMode::TRIGGER;
		recording = false;
	}
	return true;
}
/*Sets the file or the topic to recieve clouds.*/
bool setCloudSourceCB(handling_msgs::SetSource::Request &req, handling_msgs::SetSource::Response &res)
{
	if(req.isTopic)
	{
		source = PointCloudSource::TOPIC;
		topicName = req.name;
		std::cout << "[Main]: Setting topic to: "<< topicName << " and frame_id to: " << frameId << std::endl;
		frameId = req.frame_id;
		sub.shutdown();
		sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGB>> (topicName, 1, pointcloudCallback);
	}else
	{
		source = PointCloudSource::FILESYSTEM;
		fileName = req.name;
		frameId = req.frame_id;
		std::cout << "[Main]: Setting file name to: "<< fileName << " and frame_id to: " << frameId << std::endl;
	}

	return true;
}

void publishMarker(tf::Vector3 pos, tf::Quaternion rot, int id, std::string text, double r, double g, double b, double lifetime=5.0)
{
	if(marker_pub.getNumSubscribers() == 0)
		{
			std::cout << "NO MARKER SUB!\n";
			return;
		}
	std::cout << "Marker: " << text << " lifetime: " << lifetime << " id: " << id << std::endl;
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frameId;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "Object_Name";
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.text = text;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pos.getX();
	marker.pose.position.y = pos.getY();
	marker.pose.position.z = pos.getZ();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(lifetime);
	marker_pub.publish(marker);
/*
	marker.ns = "Object_Frame";

	marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	tf::Transform a1(rot,tf::Vector3(0,0,0));
	tf::Transform a2(tf::Quaternion(-M_PI*0.5,0,0),tf::Vector3(0,0,0));

	rot = (a1*a2).getRotation();

	marker.pose.orientation.x = rot.x();
	marker.pose.orientation.y = rot.y();
	marker.pose.orientation.z = rot.z();
	marker.pose.orientation.w = rot.w();

	marker.scale.x = 0.1;
	marker.scale.y = 0.005;
	marker.scale.z = 0.005;

	marker.id = id*100;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.text = "";
	marker_pub.publish(marker);*/
}


int main (int argc, char* argv[])
{
	ros::init (argc, argv, "detection");

	cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	tf_listener = std::make_shared<tf::TransformListener>();
	nh = std::make_shared<ros::NodeHandle>();

	ros::ServiceServer triggerServer = nh->advertiseService("/detection/trigger", &triggerDetectCB);
	ros::ServiceServer continuousServer = nh->advertiseService("/detection/continuous", &continuousDetectCB);
	ros::ServiceServer setSourceServer = nh->advertiseService("/detection/setSource", &setCloudSourceCB);
	ros::Publisher objectPub = nh->advertise<handling_msgs::DetectedObjects> ("/dexterity/detection/objects", 1);
	marker_pub = nh->advertise<visualization_msgs::Marker>("/dexterity/marker",10);
	handling_msgs::DetectedObjects msg;
	msg.header.seq = 0;
	msg.object_count = 0;

	sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGB>> (topicName, 1, pointcloudCallback);
	Control detectionControl(true,true,false);

	ros::Duration(0.5).sleep();
	std::cout << "Starting loop." << std::endl;
	ros::Rate r(20);

	while (ros::ok())
	{
		if (dirtyCloud) //Did we recieve a new cloud? Lets process!
		{
			std::cout << "[Main]: Starting detection!\n";
			results.resize(0);
			if(source == PointCloudSource::FILESYSTEM)
			{
				detectionControl.recognize (fileName, results); //Use file for detection
			}else{
				detectionControl.recognize (*cloud, results); //Use recieved pointcloud for detection
			}
			std::cout << "\n\n[Main]: Found " << results.size () << " matches! " << std::endl;

			msg.object_count = 0;
			msg.objects.clear();
			msg.header.frame_id = frameId;
			msg.header.seq++;
			msg.header.stamp = ros::Time::now();
			for( size_t i = 0; i < results.size(); i++)
			{

				std::cout << results.at(i).name << std::endl;
				std::cout << "[" << results.at(i).pose.getOrigin().x() << "," << results.at(i).pose.getOrigin().y() << "," << results.at(i).pose.getOrigin().z() << "]\n";

				publishMarker(results.at(i).pose.getOrigin(),results.at(i).pose.getRotation(),i,results.at(i).name,1,1,1,5.0);

				handling_msgs::DetectedObject obj;
				obj.guess = (results.at(i).pose.getRotation() == tf::Quaternion(0,0,0,1))? true : false;
				obj.object_id = results.at(i).modelId;
				obj.object_name = results.at(i).name;
				obj.pose.position.x = results.at(i).pose.getOrigin().x();
				obj.pose.position.y = results.at(i).pose.getOrigin().y();
				obj.pose.position.z = results.at(i).pose.getOrigin().z();
				obj.pose.orientation.x = results.at(i).pose.getRotation().x();
				obj.pose.orientation.y = results.at(i).pose.getRotation().y();
				obj.pose.orientation.z = results.at(i).pose.getRotation().z();
				obj.pose.orientation.w = results.at(i).pose.getRotation().w();
				obj.stamp = ros::Time::now();
				obj.probability = results.at(i).probability;
				msg.objects.push_back(obj);
				msg.object_count++;
			}
			objectPub.publish(msg);

			std::cout << "\n\n";

			dirtyCloud = false;
			if(mode == RecordMode::CONTINUOUS) recording = true; //If we should continuously detect, we have to record again.
		}
		detectionControl.spinViewer ();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

