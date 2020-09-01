/*
 * main.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: philipf & satishj
 *
 *  Detection system with trigger mode. Usefull to analyze and try out parameters on a set of scenes.
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

#include "../include/Control.h"

bool update = true;
bool newScene = true;
std::vector<Result> results;

bool triggerCB(handling_msgs::Trigger::Request &req, handling_msgs::Trigger::Response &res)
{
	update = true;
	newScene = req.start;
	return true;
}

int main (int argc, char* argv[])
{
	ros::init (argc, argv, "detection");
	ros::NodeHandle nh;
	ros::ServiceServer updateParameters = nh.advertiseService("/run", &triggerCB);
	Control detectionControl;
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

	std::vector<std::string> scenes;
	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(packagePath + "/objects/scenes"),{}))
	{
		std::string path = entry.path().string();
		std::string extension = boost::filesystem::extension(path);
		if(extension != ".pcd") continue;
		scenes.push_back(path);
	}

	ros::Duration(0.5).sleep();
	std::cout << "Starting loop." << std::endl;
	unsigned long count = 0;
	ros::Rate r(48);
	while (ros::ok())
	{
		if (update)
		{
			results.resize(0);
			std::cout << "[Main]: Starting detection!\n";
			//detectionControl.recognize (*cloud, results);
			if(!newScene)
			{
				count--;
			}
			detectionControl.recognize(scenes.at(count%scenes.size()),results);
			std::cout << "[Main]: Found " << results.size () << " matches! Using szene: " << scenes.at(count%scenes.size()) << std::endl;
			count++;
			update = false;
		}
		detectionControl.spinViewer ();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}

