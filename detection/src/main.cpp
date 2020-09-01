/*
 * main.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: philipf & satishj
 *  Used to detect objects in on a continuius stream from a depth camera. Publishes visualization messages to rviz and handling_msgs/detectedObjects which can be used for
 *  futher processing tasks like grasping.
 */

#include <iostream>
#include <stdlib.h>
#include <memory>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <handling_msgs/DetectedObjects.h>
#include <handling_msgs/DetectedObject.h>

#include "../include/Control.h"

std::string TARGET_FRAME = "/camera_link";
const std::string CAMERA_LINK = "/camera_rgb_optical_frame"; ///GETjag/xtion_sensor_link
std::string CAMERA_TOPIC = "/camera/depth_registered/points"; ///GETjag/xtion/depth/points

ros::Time last_viewer_update;
const float UPDATE_RATE = 0.15;
std::shared_ptr<tf::TransformListener> tf_listener;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
bool cloudIsUpdated = false;
bool visualization = false;
std::vector<Result> results;

ros::Publisher marker_pub;

void publishMarker(tf::Vector3 pos, tf::Quaternion rot, int id, std::string text, double r, double g, double b, double lifetime=5.0)
{
	if(marker_pub.getNumSubscribers() == 0) return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = TARGET_FRAME;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "Object_Name";
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.text = text;
	std::cout << "Marker: " << text << std::endl;

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
	marker_pub.publish(marker);
}

void pointcloudCallback (const pcl::PointCloud<PointType>::ConstPtr& pointcloud)
{
	if(cloudIsUpdated) return;
	if(pointcloud->size() == 0) return;
	tf::StampedTransform transform;
	try{
		std::cout << "[Main]: Cloud callback!\n";
		tf_listener->waitForTransform(TARGET_FRAME,CAMERA_LINK,ros::Time(0),ros::Duration(4.0));
		pcl_ros::transformPointCloud(TARGET_FRAME,*pointcloud,*cloud,*tf_listener);
		std::cout << "[Main]: Size: " << cloud->size() << std::endl;
		if(pointcloud->size() > 0)
		{
			cloudIsUpdated = true;
		}else{
			cloudIsUpdated = false;
		}
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what ());
		ros::Duration (1.0).sleep ();
		cloudIsUpdated = false;
	}
}

int main (int argc, char* argv[])
{
	if(argc > 1)
	{
		CAMERA_TOPIC = argv[1];
		if(argc > 2)
		{
			TARGET_FRAME = argv[2];
			if(argc > 3)
			{
				if(std::string(argv[3]).find("true") != std::string::npos)
				{
					visualization = true;
				}
			}

		}
	}else{
		std::cout << "main [CAMERA_TOPIC=/camera/depth_registered/points] [TARGET_FRAME=/world] [VISUALIZATION=false]";
	}
	std::cout << "Using camera Topic: " << CAMERA_TOPIC << std::endl;
	std::cout << "Using target frame: " << TARGET_FRAME << std::endl;
	std::cout << "Visualization: " << visualization << std::endl;

	ros::init (argc, argv, "mainDetection");

	ros::NodeHandle nh;
	cloud = boost::make_shared<pcl::PointCloud<PointType>>();
	tf_listener = std::make_shared<tf::TransformListener> ();

	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> (CAMERA_TOPIC, 1, pointcloudCallback);
	ros::Publisher objectPub = nh.advertise<handling_msgs::DetectedObjects> ("/dexterity/detection/objects", 1);
	marker_pub = nh.advertise<visualization_msgs::Marker>("/dexterity/marker",1);

	handling_msgs::DetectedObjects msg;
	msg.header.frame_id = TARGET_FRAME;
	msg.header.seq = 0;
	msg.object_count = 0;

	Control detectionControl(visualization,false,false);
	std::string packagePath = ros::package::getPath("handling_recognition");
	size_t id = 0;
	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(packagePath + "/objects/models"),{}))
	{
		std::string path = entry.path().string();
		std::string extension = boost::filesystem::extension(path);
		if(extension != ".pcd") continue;
		std::string name = boost::filesystem::basename(path);
		detectionControl.learnModel(path,name,id);
		id++;
	}

	ros::Duration(0.5).sleep();
	std::cout << "Starting loop." << std::endl;
	last_viewer_update = ros::Time::now();
	while (ros::ok())
	{
		if(cloudIsUpdated)
		{
			ros::Time current_time = ros::Time::now ();
			if (((current_time - last_viewer_update).toSec () > (1.0 / UPDATE_RATE)))
			{
				results.resize(0);
				std::cout << "[Main]: Starting detection!\n";
				detectionControl.recognize (*cloud, results);
				std::cout << "[Main]: Found " << results.size () << " matches!\n";
				cloudIsUpdated = false;
				last_viewer_update = ros::Time::now();

				if(results.size() > 0)
				{
					msg.object_count = 0;
					msg.objects.clear();
					msg.header.seq++;
					msg.header.stamp = ros::Time::now();
					for( size_t i = 0; i < results.size(); i++)
					{
						std::cout << results.at(i).name << std::endl;
						std::cout << "[" << results.at(i).pose.getOrigin().x() << "," << results.at(i).pose.getOrigin().y() << "," << results.at(i).pose.getOrigin().z() << "]\n";
						publishMarker(results.at(i).pose.getOrigin(),results.at(i).pose.getRotation(),i,
								results.at(i).name + "=" + std::to_string(results.at(i).model_scene_corrs.size()),1,1,1,(ros::Time::now() - current_time).toSec() + (1.0 / UPDATE_RATE));

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
				}

			}
		}
		detectionControl.spinViewer ();
		ros::spinOnce();
	}
	return 0;
}

