/*
 * main.cpp
 *
 *  Created on: Jan 23, 2019
 *      Author: satishj
 */

#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <sstream>
//#include "std_msgs/String.h"
#include <xmlrpcpp/XmlRpcException.h>
#include <handling_msgs/AddModel.h>
#include <handling_msgs/GetModels.h>
#include <handling_msgs/GetModelFile.h>
#include <handling_msgs/DeleteModel.h>
#include <handling_msgs/SaveProfile.h>
#include <handling_msgs/LoadProfile.h>
#include <handling_msgs/GetFeatures.h>
#include <handling_msgs/ReadFeatures.h>
#include <handling_msgs/GetProfiles.h>
#include <yaml-cpp/yaml.h>

#include <Database.h>

std::string profileName;
std::string comment;
std::string modelName;

std::string profileId;
std::string  modelPath;

//bool newProfile = false;
bool loadprofile = false;
bool loadModels = false;
bool saveModel = false;
bool newModel = false;
bool removeModel = false;
bool newProfile = false;

//Database database ("/home/pg/rrs_ss18/satishj/ros/src/dexterity/");
std::shared_ptr<Database> database;

bool setProfile (handling_msgs::SaveParameter::Request &req, handling_msgs::SaveParameter::Response &res)
{
	profileName = req.name;
	comment = req.comment;
	newProfile = true;
	//res.success = true;
	return true;
}

bool getProfiles (handling_msgs::GetProfiles::Request &req, handling_msgs::GetProfiles::Response &res)
{
	std::cout << "GetProfiles service called" << std::endl;
	//std::vector<std::string> profiles;
	database->getProfiles (res.profiles);
	for (auto i : res.profiles)
	{
		std::cout << "The profiles are here: " << i << "\n" << std::endl;
	}
	//res.profiles = profiles;
	return true;
}

bool getModels (handling_msgs::GetModels::Request &req, handling_msgs::GetModels::Response &res)
{
	std::cout << "GetModels service called" << std::endl;
	//std::vector<std::string> models;
	database->getModels (res.models);
	for(auto i : res.models)
	{
		std::cout <<"The models are here " << i << "\n" << std::endl;
	}
	//res.models = models;
	return true;
}

bool getModelFile (handling_msgs::GetModelFile::Request &req, handling_msgs::GetModelFile::Response &res)
{
	std::cout << "GetModelFile service called" << std::endl;

	database->getModelFile(req.name, res.modelFile);
	return true;
}


bool addModel (handling_msgs::AddModel::Request &req, handling_msgs::AddModel::Response &res)
{
	std::cout << "AddModel service called" << std::endl;
	modelName = req.name;
	modelPath = req.model;
	newModel = true;
	res.success = true;
	return true;
}

bool deleteModel (handling_msgs::DeleteModel::Request &req, handling_msgs::DeleteModel::Response &res)
{
	std::cout << "DeleteModel service called" << std::endl;
	modelName = req.name;
	removeModel = true;
	res.success = true;
	return true;
}

bool saveProfile (handling_msgs::SaveProfile::Request &req, handling_msgs::SaveProfile::Response &res)
{
	std::cout << "SaveProfile service called" << std::endl;
	profileName = req.profileName;
	newProfile = true;
	return true;
}

bool loadProfile (handling_msgs::LoadProfile::Request &req, handling_msgs::LoadProfile::Response &res)
{
	std::cout << "LoadProfile service called" << std::endl;
	std::string profileName;
	std::string yamlPath;
	std::string models;
	profileName = req.profile;
	database->loadProfile(profileName, yamlPath, models);
	//system("rosparam load /home/pg/rrs_ss18/satishj/ros/src/dexterity/outfile.yaml /detection");
	std::string path ;//= ros::package::getPath("handling_recognition");
	path = "rosparam load " + yamlPath + " /detection/";
	system(path.c_str());
	ros::param::set("detection/database/models", models);
	std::cout << "The tic tok main............." << std::endl;

	return true;
}

bool readFeatures (handling_msgs::ReadFeatures::Request &req, handling_msgs::ReadFeatures::Response &res)
{
	std::cout << "ReadFeatures service called" << std::endl;
	std::string profileName = req.profileId;
	database->readFeatures(profileName, req.name, res.modelFiles, res.keypointFiles, res.descriptorFiles, res.normalFiles, res.referenceFrameFiles);
	return true;
}

/*void loadYamlFile(const std::string &filename)
{
	std::ifstream f(filename);
	if (!f.is_open ())
	{
		throw std::invalid_argument ("Invalid file: " + filename);
	}
	std::stringstream buf;
	buf << f.rdbuf ();
	std::string yaml = buf.str ();
	YAML::Node detection;
	try
	{
		detection = YAML::Load (yaml);
	}
	catch (const YAML::ParserException& e)
	{
		throw std::invalid_argument ("Invalid syntax in yaml file.");
	}

	YAML::Node paramList = detection["Detection"];

	for (auto paramNode : paramList)
	{

	}

	YAML::Node key = paramList["CorrespondenceVoting"];
	std::cout << key << std::endl;
	YAML::Node key2 = key["correspondenceRatioThreshold"];
	std::cout << key2 << std::endl;
	YAML::Node maxNode = key2["max"];
	std::cout << maxNode << std::endl;
	double maxValue = maxNode.as<double>();
	std::cout << maxValue << std::endl;
}*/

int main (int argc, char* argv[])
{
	//loadYamlFile("/home/pg/rrs_ss18/satishj/Downloads/satish.txt");

	ros::init (argc, argv, "database_node");
	ros::NodeHandle nh;
	ros::ServiceServer setProfileService = nh.advertiseService ("/detection/database/setProfile", setProfile);
	ros::ServiceServer getProfileService = nh.advertiseService ("/detection/database/getProfiles", getProfiles);
	ros::ServiceServer getModelService = nh.advertiseService ("/detection/database/getModels", getModels);
	ros::ServiceServer getModelFileService = nh.advertiseService ("/detection/database/getModelFile", getModelFile);
	ros::ServiceServer addModelService = nh.advertiseService ("/detection/database/addModel", addModel);
	ros::ServiceServer deleteModelService = nh.advertiseService ("/detection/database/deleteModel", deleteModel);
	ros::ServiceServer saveProfileService = nh.advertiseService ("/detection/database/saveProfile", saveProfile);
	ros::ServiceServer loadProfileService = nh.advertiseService ("/detection/database/loadProfile", loadProfile);
	ros::ServiceServer readFeaturesService = nh.advertiseService ("/detection/database/readFeatures", readFeatures);
	ros::ServiceClient getFeaturesClient = nh.serviceClient<handling_msgs::GetFeatures>("/detection/database/getFeatures");
	ros::Rate loop_rate (10);

	database = std::make_shared<Database>(ros::package::getPath("handling_recognition"));// + std::string("/database/"));
	//database->open ("sqlite3", "/home/pg/rrs_ss18/satishj/database/test.sqlite");
	database->open ("sqlite3", ros::package::getPath("handling_recognition") + std::string("/database/test.sqlite")); //"/home/pg/rrs_ss18/satishj/database/test.sqlite"

	while (ros::ok ())
	{
		/*if (newProfile)
		{
			std::cout << "Saving new profile \n";
			system (" echo 'ROS Params dumped at: ' $(pwd); rosparam dump infile.yaml /test");
			//database.saveProfile (profileName, comment);
			newProfile = false;
		}*/
		/*if (loadprofile)
		{
			std::cout << "Loading the profile \n";
			system ("$(pwd); rosparam load infile.yaml  /sec");
			//database.loadProfile (profileId);
			newProfile = false;
		}*/
		if (newModel)
		{
			std::cout << "Storing new model to the database \n" << std::endl;;
			database->addModel(modelName, modelPath);
			newModel= false;
			std::cout << "Model stored successfully" << "\n" << std::endl;;
		}
		if (removeModel)
		{
			std::cout << "Deleting model from the database \n" << std::endl;
			database->deleteModel (modelName);
			removeModel = false;
			std::cout << "Model removed successfully" << "\n" << std::endl;
		}
		if (newProfile)
		{
			std::cout << "Creating new profile in the database \n" << std::endl;
			std::string modelNames;
			nh.getParam("detection/database/models", modelNames);
			std::cout << "Models are: " << modelNames << std::endl;
			//system(" echo 'ROS Params dumped at: ' $(pwd); rosparam dump /home/pg/rrs_ss18/satishj/ros/src/dexterity/dump.yaml");
			//std::vector<std::string> keypointFiles;
			std::remove("/home/pg/rrs_ss18/satishj/ros/src/dexterity/dump.yaml");
			//system(" rosparam dump /home/pg/rrs_ss18/satishj/ros/src/dexterity/dump.yaml /detection/");
			std::string path = ros::package::getPath("handling_recognition");
			path = "rosparam dump " + path + "/dump.yaml /detection/";
			system(path.c_str());
			handling_msgs::GetFeatures::Request req;
			handling_msgs::GetFeatures::Response res;
			database->saveProfile (profileName, modelNames,req.names, req.keypointFiles, req.descriptorFiles, req.normalFiles, req.referenceFrameFiles);
			getFeaturesClient.call(req,res);
			newProfile = false;
			std::cout << "New profile created successfully" << "\n" << std::endl;
		}
		ros::spinOnce ();
		loop_rate.sleep();
	}
	return 0;
}

