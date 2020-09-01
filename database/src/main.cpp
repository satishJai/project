/*
 * main.cpp
 *
 *  Created on: Jan 23, 2019
 *      Author: satishj
 *	This is the main function for the database. It contains all the service and its callback functions which in turn invokes a
 *	specific function from database.cpp
 */

#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <sstream>
#include <xmlrpcpp/XmlRpcException.h>
#include <handling_msgs/AddModel.h>
#include <handling_msgs/GetModels.h>
#include <handling_msgs/GetModelFile.h>
#include <handling_msgs/DeleteModel.h>
#include <handling_msgs/DeleteProfile.h>
#include <handling_msgs/SaveProfile.h>
#include <handling_msgs/LoadProfile.h>
#include <handling_msgs/GetFeatures.h>
#include <handling_msgs/ReadFeatures.h>
#include <handling_msgs/GetProfiles.h>
#include <handling_msgs/GetObjectMeshPaths.h>
#include <handling_msgs/GetObjectHandlingParam.h>
#include <yaml-cpp/yaml.h>

#include <Database.h>

std::string profileName;
std::string comment;
std::string modelName;

std::string profileId;
std::string modelPath;

bool newProfile = false;

std::shared_ptr<Database> database;

bool setProfile (handling_msgs::SaveParameter::Request &req, handling_msgs::SaveParameter::Response &res)
{
	profileName = req.name;
	comment = req.comment;
	newProfile = true;
	return true;
}

/*This service is used to pull all the currently available profiles from the database */
bool getProfiles (handling_msgs::GetProfiles::Request &req, handling_msgs::GetProfiles::Response &res)
{
	std::cout << "GetProfiles service called" << std::endl;
	database->getProfiles (res.profiles);
	for (auto i : res.profiles)
	{
		std::cout << "The profiles are here: " << i << "\n" << std::endl;
	}
	return true;
}

/*This service is used to get the list of all available models from the database*/
bool getModels (handling_msgs::GetModels::Request &req, handling_msgs::GetModels::Response &res)
{
	std::cout << "GetModels service called" << std::endl;
	database->getModels (res.models);
	for (auto i : res.models)
	{
		std::cout << "The models are here " << i << "\n" << std::endl;
	}
	return true;
}

/*This service is used to get the location where the models are stored in the hard disk*/
bool getModelFile (handling_msgs::GetModelFile::Request &req, handling_msgs::GetModelFile::Response &res)
{
	std::cout << "GetModelFile service called" << std::endl;
	database->getModelFile (req.name, res.modelFile);
	return true;
}
/*This service is used to add new models to the database*/
bool addModel (handling_msgs::AddModel::Request &req, handling_msgs::AddModel::Response &res)
{
	std::cout << "AddModel service called" << std::endl;
	modelName = req.name;
	modelPath = req.model;
	database->addModel (modelName, modelPath);
	std::cout << "Model stored successfully" << "\n" << std::endl;
	res.success = true;
	return true;
}

/*This service is used to delete a particular model from the database*/
bool deleteModel (handling_msgs::DeleteModel::Request &req, handling_msgs::DeleteModel::Response &res)
{
	std::cout << "DeleteModel service called" << std::endl;
	modelName = req.name;
	database->deleteModel (modelName);
	std::cout << "Model removed successfully" << "\n" << std::endl;
	res.success = true;
	return true;
}

/*This service is used to add new profile to the database*/
bool saveProfile (handling_msgs::SaveProfile::Request &req, handling_msgs::SaveProfile::Response &res)
{
	std::cout << "SaveProfile service called" << std::endl;
	profileName = req.profileName;
	newProfile = true;
	return true;
}

/*This service is used to get a specific profile from the database*/
bool loadProfile (handling_msgs::LoadProfile::Request &req, handling_msgs::LoadProfile::Response &res)
{
	std::cout << "LoadProfile service called" << std::endl;
	std::string profileName;
	std::string yamlPath;
	std::string models;
	profileName = req.profile;
	database->loadProfile (profileName, yamlPath, models);
	std::string path;
	path = "rosparam load " + yamlPath + " /detection/";
	system (path.c_str ());
	ros::param::set ("detection/database/models", models);
	return true;
}
/*This service is used to read the features of the models available for the requested profile */
bool readFeatures (handling_msgs::ReadFeatures::Request &req, handling_msgs::ReadFeatures::Response &res)
{
	std::cout << "ReadFeatures service called" << std::endl;
	std::string profileName = req.profileId;
	database->readFeatures (profileName, req.name, res.modelFiles, res.keypointFiles, res.descriptorFiles, res.normalFiles, res.referenceFrameFiles);
	return true;
}

bool getActionsPoints(handling_msgs::GetObjectHandlingParam::Request &req, handling_msgs::GetObjectHandlingParam::Response &res)
{
	database->getActions(req.actionType, req.modelName, res.actionPoint, res.actionName);
	return true;
}

bool getObjectMesh(handling_msgs::GetObjectMeshPaths::Request &req, handling_msgs::GetObjectMeshPaths::Response &res)
{
	database->getObjectMesh(req.actionType, res.modelName, res.meshPath);
	return true;
}

bool deleteProfile(handling_msgs::DeleteProfile::Request &req, handling_msgs::DeleteProfile::Response &res)
{
	database->deleteProfile(req.profileName);
	return true;
}

int main (int argc, char* argv[])
{
	ros::init (argc, argv, "database_node");
	ros::NodeHandle nh;
	ros::ServiceServer setProfileService = nh.advertiseService ("/detection/database/setProfile", setProfile);
	ros::ServiceServer getProfileService = nh.advertiseService ("/detection/database/getProfiles", getProfiles);
	ros::ServiceServer getModelService = nh.advertiseService ("/detection/database/getModels", getModels);
	ros::ServiceServer getModelFileService = nh.advertiseService ("/detection/database/getModelFile", getModelFile);
	ros::ServiceServer addModelService = nh.advertiseService ("/detection/database/addModel", addModel);
	ros::ServiceServer deleteModelService = nh.advertiseService ("/detection/database/deleteModel", deleteModel);
	ros::ServiceServer deleteProfileService = nh.advertiseService ("/detection/database/deleteProfile", deleteProfile);
	ros::ServiceServer saveProfileService = nh.advertiseService ("/detection/database/saveProfile", saveProfile);
	ros::ServiceServer loadProfileService = nh.advertiseService ("/detection/database/loadProfile", loadProfile);
	ros::ServiceServer readFeaturesService = nh.advertiseService ("/detection/database/readFeatures", readFeatures);
	ros::ServiceClient getFeaturesClient = nh.serviceClient<handling_msgs::GetFeatures> ("/detection/database/getFeatures");
	ros::ServiceServer GetObjectMeshPathsService = nh.advertiseService ("/detection/database/getObjectMesh", getObjectMesh);
	ros::ServiceServer GetObjectHandlingParamService = nh.advertiseService ("/detection/database/getActionPoints", getActionsPoints);
	ros::Rate loop_rate (10);
	/*Setting up the common database path*/
	database = std::make_shared<Database> (ros::package::getPath ("handling_recognition"));
	/*Setting up the sqlite database connection path*/
	database->open ("sqlite3", ros::package::getPath ("handling_recognition") + std::string ("/database/test.sqlite"));

	while (ros::ok ())
	{
		if (newProfile)
		{
			std::cout << "Creating new profile in the database \n" << std::endl;
			std::string modelNames;
			nh.getParam ("detection/database/models", modelNames);
			std::cout << "Models are: " << modelNames << std::endl;
			/*Creating a dump file with the ros parameters*/
			std::string path = ros::package::getPath ("handling_recognition");
			/*Removing the old file*/
			std::string removePath = path + "/dump.yaml";
			std::remove (removePath.c_str());
			/*Adding the new file with new ros parameters*/
			std ::string addPath = "rosparam dump " + path + "/dump.yaml /detection/";
			system (addPath.c_str ());
			handling_msgs::GetFeatures::Request req;
			handling_msgs::GetFeatures::Response res;
			database->saveProfile (profileName, modelNames, req.names, req.keypointFiles, req.descriptorFiles, req.normalFiles, req.referenceFrameFiles);
			getFeaturesClient.call (req, res);
			newProfile = false;
			std::cout << "New profile created successfully" << "\n" << std::endl;
		}
		ros::spinOnce ();
		loop_rate.sleep ();
	}
	return 0;
}

