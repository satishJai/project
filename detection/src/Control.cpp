/*
 * Control.cpp
 *
 *  Created on: Nov 20, 2018
 *      Author: philipf & satishj
 */
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

#include "../include/Control.h"
#include "../include/descriptor/DescriptorCreator.h"
#include "../include/keypoint/KeypointCreator.h"
#include "../include/matching/Matching.h"
#include "../include/pose_estimator/PoseEstimator.h"
#include "../include/evaluation/Guesser.h"
#include "../include/evaluation/ResultEvaluation.h"
#include "../include/segmentation/SegmentsCreator.h"
#include "../include/visualization/visualization.h"
#include <ClassLoadManager.h>

/*Will create ros parameters, the ros parameter manager, start the visualizer and load the default algorithm classes. Advertises all services.*/
Control::Control(bool visualization, bool externalDatabase, bool enableCheatSegmentation){
	if(!ros::isInitialized()){
		std::perror("[Control]: Ros needs to be initialized before Control object is created.");
		return;
	}
	board = std::make_shared<Blackboard>();
	allowVisualization = visualization;
	cheatSegmentation = enableCheatSegmentation;
	if(externalDatabase)
	{
		externalDatabaseEnabled = true;
		getFeaturesServer = nh.advertiseService("/detection/database/getFeatures",&Control::getFreaturesCB,this);
		updateModelsParamsServer = nh.advertiseService("/detection/database/updateModels",&Control::updateModelsCB,this);
		updatedetectionParamsServer = nh.advertiseService("/detection/database/updateParameters", &Control::settingsChangedCB, this);
		updateVisualParamsServer = nh.advertiseService("/detection/database/updateVisuals",&Control::updateVisualsCB,this);
	}
	descriptorCreatorName = RosParameterManager::createParam<std::string> ("ShotCreator",ParamType::ENUM,
			"ShotCreator,CShotCreator,RSDCreator,FPFHCreator,ShapeContextCreator,UShapeContextCreator","/control/descriptorClass");
	keypointCreatorName = RosParameterManager::createParam<std::string> ("VoxelGridCreator",ParamType::ENUM,"VoxelGridCreator,NDCreator,ISSCreator","/control/keypointClass");
	poseEstimatorName = RosParameterManager::createParam<std::string> ("GCClustering",ParamType::ENUM,"GCClustering,HoughClustering","/control/poseEstimatorClass");
	matchingName = RosParameterManager::createParam<std::string> ("ColorMatching",ParamType::ENUM,"ColorMatching,KColorMatching,KSColorMatching,SimpleMatching","/control/matchingClass");
	evaluationName = RosParameterManager::createParam<std::string> ("CorrespondenceVoting",ParamType::ENUM,"CorrespondenceVoting,CorrespondenceVotingICP","/control/evaluationClass");
	if(allowVisualization)
	{
		visualizer = std::make_shared<Visualization>();
		visualizer->init(board,std::shared_ptr<ros::NodeHandle>(&nh),1.0); //,std::shared_ptr<ros::NodeHandle>(&nh),1.0
	}
	Blackboard::parameterManager.initParameters(nh);
	RosParameterManager::readParameters(nh); //Updates all RosParameter objects
	setAlgorithmClasses();
	updatedParamsServer = nh.advertiseService("/updateParameters", &Control::updateParamCB, this);

	if(externalDatabase)
	{
		std_srvs::Empty srv;
		updateModelsCB(srv.request,srv.response);
	}
}
/*Will read the current set algorithm class names and load them into the blackboard.*/
void Control::setAlgorithmClasses()
{
	auto descriptorObject = ClassLoadManager::getDescriptorCreator(descriptorCreatorName->var);
	if(descriptorObject != nullptr)
	{
		board->descriptorCreator = descriptorObject;
	}
	auto keypointObject = ClassLoadManager::getKeypointCreator(keypointCreatorName->var);
	if(keypointObject != nullptr)
	{
		board->keypointCreator = keypointObject;
	}
	auto poseEstimatorObject = ClassLoadManager::getPoseEstimator(poseEstimatorName->var);
	if(poseEstimatorObject != nullptr)
	{
		board->poseEstimator = poseEstimatorObject;
	}
	auto matchingObject = ClassLoadManager::getMatching(matchingName->var);
	if(matchingObject != nullptr)
	{
		board->matching = matchingObject;
	}
	auto evaluationObject = ClassLoadManager::getResultEvaluation(evaluationName->var);
	if(evaluationObject != nullptr)
	{
		board->resultEvaluation = evaluationObject;
	}
}
/*Checks if a model is not valid anymore and will retrain it. ForceRetrain can force a retrain.*/
void Control::checkForRetrain(bool forceRetrain)
{
	std::shared_ptr<Blackboard> b = board;
	#pragma omp parallel for shared(b,forceRetrain)
	for(size_t i = 0; i < b->models.size(); i++)
	{
		if(!forceRetrain && b->models.at(i).isValid) continue; //Only retrain if neccecary!

		Model *oldModel = &b->models.at(i);
		oldModel->data.keypoints->clear();
		oldModel->data.normals->clear();
		oldModel->data.descriptors = boost::shared_ptr<Descriptors>();
		//Run learning pipeline
		std::cout << "[Control]: Relearning model: " << oldModel->name << std::endl;
		oldModel->data.createNormals();
		b->keypointCreator->run(oldModel->data.cloud,oldModel->data.keypoints);
		oldModel->data.createRF();
		b->descriptorCreator->run(oldModel->data.cloud,oldModel->data.keypoints,oldModel->data.normals,oldModel->data.descriptors);
		oldModel->isValid = true;
	}
	std::cout << "[Control]: Done with learning.\n";
	if(allowVisualization) visualizer->drawModels();
}

/* Learns the given model. Therefore keypoints are generated. For each keypoint descriptors are generated. The model will be stored in the blackboard. */
bool Control::learnModel(const pcl::PointCloud<PointType>& cloud, const std::string modelName, const unsigned int modelId)
{
	if(cloud.size() == 0) std::cout << "[Control]: Cloud is empty" << std::endl;
	Model model(modelId,modelName);
	tf::Vector3 offset = tf::Vector3(0,0,0);
	model.positionOffset = tf::Vector3(offset.x() * model.id,offset.y(),offset.z());

	pcl::copyPointCloud(cloud,*model.data.cloud);
	pcl::transformPointCloud(*model.data.cloud,*model.data.cloud, Eigen::Vector3f (model.positionOffset.x(),
																			       model.positionOffset.y(),
																				   model.positionOffset.z()), Eigen::Quaternionf (1, 0, 0, 0));

	//Run learning pipeline
	std::cout << "[Control]: Learning model!\n";
	model.data.createNormals();
	board->keypointCreator->run(model.data.cloud,model.data.keypoints);
	model.data.createRF();
	board->descriptorCreator->run(model.data.cloud,model.data.keypoints,model.data.normals,model.data.descriptors);
	model.isValid = true;
	//Store model in blackboard.
	board->models.push_back(model);
	std::cout << "[Control]: Drawing model!\n";
	//TODO Store model into database
	if(allowVisualization) visualizer->drawModels();
	return true;
}
/*Loads the point cloud from disk and runs learnModel*/
bool Control::learnModel(const std::string file, const std::string modelName, const unsigned int modelId)
{
	pcl::PointCloud<PointType> newModel;
	if (pcl::io::loadPCDFile (file, newModel) < 0)
	{
		ROS_ERROR("[Control::learnModel]: Error loading cloud '%s'",file.c_str());
		return false;
	}
	std::cout << "[Control]: Loaded: " << modelName << "@" << file << "=" << newModel.size() << " points\n";
	return learnModel(newModel,modelName,modelId);
}
/*Cuts a szene into objects and tries to recognize each object.*/
void Control::recognize(const pcl::PointCloud<PointType>& cloud, std::vector<Result> &results)
{
	checkForRetrain();
	board->segments.clear();

	if(cheatSegmentation) //If real segmentation is disabled, it will do a cylindric cut around (0,0,0)
	{
		double radius = 0.2;
		double minHeight = 0.02;
		std::cout << "[Control]: Start cutting cloud! Using start_z: " << minHeight << " and radius: " << radius << std::endl;
		Segment s;
		for(size_t i = 0; i < cloud.size(); i++)
		{
			const double x = cloud.at(i).x;
			const double y = cloud.at(i).y;
			const double z = cloud.at(i).z;
			if(z >= minHeight)
			{
				if(((x*x) + (y*y)) <= radius)
				{
					s.data.cloud->push_back(cloud.at(i));
				}
			}
		}
		//*s.data.cloud = cloud;
		s.data.createNormals();
		board->segments.push_back(s);
	}else
	{
		std::cout << "[Control]: Start segmentation!" << std::endl;
		board->segmentation->runSegmentation(boost::make_shared<pcl::PointCloud<PointType>>(cloud),&board->segments);
	}

	std::cout << "[Control]: Starting to recognize all "<< board->models.size() << " learned models.\n";

	for(unsigned int j = 0; j < board->segments.size(); j++)
	{
		//Run Segmentation to find objects
		Segment& segment = board->segments.at(j);
		segment.results.resize(board->models.size());

		//Create keypoints and reference frames for each object
		board->keypointCreator->run(segment.data.cloud,segment.data.keypoints);
		std::cout << "[Control]: Cloud: " << segment.data.cloud->size() << " keys: " << segment.data.keypoints->size() << std::endl;
		segment.data.createRF();

		//Create descriptors
		board->descriptorCreator->run(segment.data.cloud,segment.data.keypoints,segment.data.normals,segment.data.descriptors);

		//Match objects with all currently learned models
		board->matching->run(*segment.data.descriptors,segment.data.keypoints,&board->models,&segment.model_scene_corrs_vec,&segment.notFilteredCorresCountVec);

		std::shared_ptr<Blackboard> b = board;
		#pragma omp parallel for shared(b,segment)
		for(unsigned int i = 0; i < board->models.size(); i++)
		{
			if(segment.model_scene_corrs_vec.at(i)->size() == 0) continue;
			b->poseEstimator->run(segment.data.cloud,segment.data.rfs, segment.data.keypoints,b->models.at(i),segment.model_scene_corrs_vec.at(i),&segment.results.at(i));
		}
		//We have multiple possible guesses per object. Lets pick a guess for each object!
		Result finalGuess;
		bool success = board->resultEvaluation->run(segment,finalGuess,board->models);
		if(finalGuess.name != "UNKNOWN")
		{
			std::cout << "[Control]: Picked: " << finalGuess.name << " as winner." << std::endl;
			if (!success || !finalGuess.validPose)
			{
				std::cout << "[Control]: Pose is not valid. Guesser will run!" << std::endl;
				board->guesser->run (segment, &board->models, finalGuess);
			}
			board->segments.at (j).bestGuess = finalGuess;
			if(allowVisualization)
			{
				//tf::Transform visualizerOffset(tf::Quaternion(0,0,0,1),visualizer->getModelOffset());
				finalGuess.pose = finalGuess.pose;
			}
			results.push_back(finalGuess);
		}else
		{
			std::cout << "[Control::recognize]: Found no object in segment "<< j << std::endl;
		}
	}
	board->counter++;
	if(allowVisualization) visualizer->spinOnce();
	std::cout << "\n[Control::recognize]: DONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n";
}
/*Loards a scene from disk and runs recognize.*/
bool Control::recognize(const std::string file, std::vector<Result> &results)
{
	pcl::PointCloud<PointType> cloud;
	if (pcl::io::loadPCDFile (file, cloud) < 0)
	{
		ROS_ERROR("[Control::recognize]: Error loading cloud '%s'", file.c_str ());
		return false;
	}
	recognize(cloud,results);
	return true;
}

void Control::evaluateDescriptors(const pcl::PointCloud<PointType>& cloud, std::map<std::string,std::vector<double>> &results, unsigned long &numKeypoints)
{
	checkForRetrain();
	board->segments.clear();

	double radius = 1.0;
	double minHeight = 0.02;
	std::cout << "Start cutting cloud! Using start_z: " << minHeight << " and radius: " << radius << std::endl;
	Segment segment;
	for(size_t i = 0; i < cloud.size(); i++)
	{
		const double x = cloud.at(i).x;
		const double y = cloud.at(i).y;
		const double z = cloud.at(i).z;
		if(z >= minHeight)
		{
			if(((x*x) + (y*y)) <= radius)
			{
				segment.data.cloud->push_back(cloud.at(i));
			}
		}
	}
	//*s.data.cloud = cloud;
	segment.data.createNormals();
	//board->segmentation->run(boost::make_shared<pcl::PointCloud<PointType>>(cloud),&board->segments);

	std::cout << "[Control]: Starting to recognize all "<< board->models.size() << " learned models.\n";
	//Run Segmentation to find objects
	segment.results.resize(board->models.size());

	//Create keypoints and reference frames for each object
	board->keypointCreator->run(segment.data.cloud,segment.data.keypoints);
	numKeypoints = segment.data.keypoints->size();
	std::cout << "[Control]: Cloud: " << segment.data.cloud->size() << " keys: " << segment.data.keypoints->size() << std::endl;
	segment.data.createRF();

	//Create descriptors
	board->descriptorCreator->run(segment.data.cloud,segment.data.keypoints,segment.data.normals,segment.data.descriptors);
	std::cout << "[Control]: Start matching!\n";
	//Match objects with all currently learned models
	board->matching->run(*segment.data.descriptors,segment.data.keypoints,&board->models,&segment.model_scene_corrs_vec,&segment.notFilteredCorresCountVec);
	std::cout << "[Control]: Storing all correspondences:\n";
	for(unsigned int i = 0; i < board->models.size(); i++)
	{
		std::vector<double> corres;
		if(segment.model_scene_corrs_vec.size() <= i)
		{
			results.insert(std::pair<std::string,std::vector<double>>(board->models.at(i).name,corres));
			continue;
		}
		pcl::CorrespondencesPtr model_scene_corrs = segment.model_scene_corrs_vec.at(i);
		if(model_scene_corrs)
		{
			for(pcl::Correspondence c : *model_scene_corrs)
			{
				if(c.distance > 0)
				{
					corres.push_back(c.distance);
				}
			}
		}
		results.insert(std::pair<std::string,std::vector<double>>(board->models.at(i).name,corres));
	}
	board->counter++;
	if(allowVisualization) visualizer->spinOnce();
}
/*Called from database if the current profile is saved. Then also the features shoudl be saved.*/
bool Control::getFreaturesCB(handling_msgs::GetFeatures::Request &req, handling_msgs::GetFeatures::Response &res)
{
	std::cout << "\n############### Get Featrues CB\n";
	if((req.names.size() != req.descriptorFiles.size()) || (req.names.size() != req.keypointFiles.size()) || (req.names.size() != req.referenceFrameFiles.size()))
	{
		std::cout << "[Control::getFreaturesCB]: Can not store " << req.names.size() << " models if there are not the same amount of paths given!\n";
	}
	std::cout << "Req.names.size() == " << req.names.size() << std::endl;
	for(size_t i = 0; i < req.names.size(); i++) //Save all given models
	{
		size_t modelId;
		bool success = false;
		std::cout << "board->models.size() == " << board->models.size() << std::endl;
		for(modelId = 0; (!success) && (modelId < board->models.size()); modelId++) //Search for the model name
		{
			//Is this the model the database is looking for?
			if(board->models.at(modelId).name != req.names.at(i)) continue;

			std::cout << "Saving " << req.names.at(i) << "  to:\n";
			std::cout << req.keypointFiles.at(i) << "\n" << req.descriptorFiles.at(i) << "\n" << req.normalFiles.at(i) << "\n" << req.referenceFrameFiles.at(i) << "\n";
			//Save the individual model features at the given paths!
			board->models.at(modelId).data.saveAtLocation(req.keypointFiles.at(i),req.descriptorFiles.at(i),req.normalFiles.at(i),req.referenceFrameFiles.at(i));
			std::cout << "Saved " << board->models.at(modelId).name << std::endl;
			success = true;
		}
		if(!success)
		{
			std::cout << "[Control::getFreaturesCB]: Unable to find model: " << req.names.at(i) << ". Unable to save its features!\n";
		}
	}
	return true;
}

bool Control::updateParamCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::cout << "\n############### Update Parameters CB!\n";
	RosParameterManager::readParameters(nh); //Updates all RosParameter objects
	setAlgorithmClasses();
	//In the next detection process, all models will be learned again.
	for(size_t i = 0; i < board->models.size(); i++)
	{
		board->models.at(i).isValid = false;
	}
	return true;
}

/*Reads all parameters from the parameter list and updates the current system.*/
bool Control::settingsChangedCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	std::cout << "\n############### Settings changed CB2!\n";
	RosParameterManager::readParameters(nh); //Updates all RosParameter objects
	setAlgorithmClasses();
	std::cout << "Size: " << board->models.size() << std::endl;
	//In the next detection process, all models will be learned again.
	for(size_t i = 0; i < board->models.size(); i++)
	{
		board->models.at(i).isValid = false;
		std::cout << "Setting " << board->models.at(i).name << " to not valid!\n";
	}

	//Get the current profile id
	std::string newProfileId;
	if(!nh.getParam("/detection/database/profileName",newProfileId))
	{
		std::cout << "[Control::settingsChangedCB]: ERROR!! /detection/database/profileName is not set!!\n";
		latestProfileId = "";
		return false;
	}
	std::cout << "Still alive!\n";
	if(req.data) //We loaded a new profile!
	{
		std::cout << "[Control::settingsChangedCB]: loading new profile!\n";
		//Get new model list!
		std::vector<std::string> names;
		std::string modelNames;
		//Model names are comma separated string. We need to get each model into a vector.
		if(!nh.getParam("/detection/database/models",modelNames))
		{
			std::cout << "[Control::settingsChangedCB]: ERROR!! /detection/database/models is not set!!\n";
			return false;
		}
		std::stringstream lineStream(modelNames);
		std::string m;
		std::cout << "M: ";
		while(std::getline(lineStream,m,','))
		{
			names.push_back(m);
			std::cout << m;
		}
		std::cout << std::endl;

		readFeaturesFromDatabase(names,newProfileId);
		latestProfileId = newProfileId;
	}
	return true;
}
/* Reads the new features for the given models from database. If a models is unknown for the detection system, it will be added.*/
bool Control::readFeaturesFromDatabase(std::vector<std::string> models, std::string profileId)
{
	std::cout << "\n############### ReadFeaturesFromDatabase!\n";
	ros::ServiceClient client = nh.serviceClient<handling_msgs::ReadFeatures>("/detection/database/readFeatures");
	if(!client.exists())
	{
		std::cout << "[Control::readFeaturesFromDatabase]: Unable to connect to database via '/detection/database/readFeatures'!\n";
		return false;
	}
	handling_msgs::ReadFeatures srv;
	srv.request.name = models;
	srv.request.profileId = profileId;
	client.call(srv);

	for(auto m : models)
	{
		std::cout << m << std::endl;
	}
	for(auto m : srv.response.modelFiles)
	{
		std::cout << m << std::endl;
	}
	for(auto m : srv.response.keypointFiles)
	{
		std::cout << m << std::endl;
	}
	for(auto m : srv.response.referenceFrameFiles)
	{
		std::cout << m << std::endl;
	}

	bool success = true;
	for(size_t i = 0; i < srv.response.modelFiles.size(); i++)
	{
		bool exit = false;
		for(size_t modelId = 0; (!exit) && (modelId < board->models.size()); modelId++)//Search for the model name
		{
			//Is this the model the database is looking for?
			if(board->models.at(modelId).name != models.at(i)) continue;

			std::cout << "Loading features from database: " << board->models.at(modelId).name << std::endl;
			int result = board->models.at(modelId).data.loadFromLocation(srv.response.modelFiles.at(i),srv.response.keypointFiles.at(i),
					srv.response.descriptorFiles.at(i), srv.response.normalFiles.at(i),
					srv.response.referenceFrameFiles.at(i),false, board->descriptorCreator);

			//Some features could not be found!
			if(result == 0) board->models.at(modelId).isValid = true;
			else board->models.at(modelId).isValid = false;
			exit = true;
		}
		if(!exit)//Model is new and is not there in the blackboard-> So we have to create a new model and push it back into our list of learned models!
		{
			Model newModel(board->models.size(),models.at(i));

			std::cout << "[Control]: Creating model from database: " << newModel.name << std::endl;
			int result = newModel.data.loadFromLocation(srv.response.modelFiles.at(i),srv.response.keypointFiles.at(i),
					srv.response.descriptorFiles.at(i), srv.response.normalFiles.at(i),
					srv.response.referenceFrameFiles.at(i),true, board->descriptorCreator);
			std::cout << "[Control]: Done creating model!\n";
			//Model could not be loaded...
			if(result == -1)
			{
				std::cout << "[Control]: Model " << newModel.name << " not found. Ignoring it!\n";
				success = false;
				continue;
			}
			//Add an offset to the model, because it should not overlap with the other models
			tf::Vector3 offset = tf::Vector3(0,0,0);
			//if(allowVisualization) offset = visualizer->getModelOffset();
			newModel.positionOffset = tf::Vector3(offset.x() * newModel.id,offset.y(),offset.z());
			pcl::copyPointCloud(*newModel.data.cloud,*newModel.data.cloud);
			pcl::transformPointCloud(*newModel.data.cloud,*newModel.data.cloud, Eigen::Vector3f (newModel.positionOffset.x(),
									  newModel.positionOffset.y(),
									  newModel.positionOffset.z()), Eigen::Quaternionf (1, 0, 0, 0));
			//Succesfully loaded features from database?
			if(result == 0) newModel.isValid = true;
			board->models.push_back(newModel);
		}
	}
	return success;

}
/* Checks which models became inactive and deletes them. Checks which models became active and requests them from database.*/
bool Control::updateModelsCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::cout << "\n############### UpdateModelsCB\n";
	//Get new model list!
	std::vector<std::string> names;
	std::string modelNames;
	//Model names are comma separated string. We need to get each model into a vector.
	if(!nh.getParam("/detection/database/models",modelNames))
	{
		std::cout << "[Control::settingsChangedCB]: ERROR!! /detection/database/models is not set!!\n";
		return false;
	}
	std::stringstream lineStream(modelNames);
	std::string m;
	std::cout << "[Control]: Given model names: " << modelNames << std::endl;
	std::cout << "[Control]: Given model names converted: ";
	while(std::getline(lineStream,m,','))
	{
		std::cout << m << " ";
		names.push_back(m);
	}
	std::cout << std::endl;
	//Find all models which are not active anymore
	for(int i = 0; i < (int)board->models.size(); i++)
	{
		bool found = false;
		for(size_t j = 0; !found && (j < names.size()); j++)
		{
			if(names.at(j) == board->models.at(i).name) //Is the model still active and sould stay?
			{
				std::cout << "[Control]: " << board->models.at(i).name << " is still active!\n";
				found = true;
			}
		}
		if(!found) //Model is not in the list of active models. We have to forget it!
		{
			std::cout << "[Control]: " << board->models.at(i).name << " went to inactive! Detection will forget it!\n";
			board->models.at(i) = board->models.at(board->models.size()-1);
			board->models.resize(board->models.size()-1);
			board->models.at(i).id = i;
			//TODO visualization offset!!
			i--;
		}
	}

	//Find all models which became active and add them
	std::vector<std::string> newModels;
	for(size_t i = 0; i < names.size(); i++)
	{
		bool found = false;
		for(size_t j = 0; !found && (j < board->models.size()); j++)
		{
			if(names.at(i) == board->models.at(j).name) //Is the model still active and should stay?
			{
				std::cout << "[Control]: Found " << names.at(i) << ". It does not need to be added!\n";
				found = true;
			}
		}
		if(!found) //Model is not in the list of active models. We have to forget it!
		{
			std::cout << "[Control]: " << names.at(i) << " went to active! Detection will learn it!\n";
			newModels.push_back(names.at(i));
		}
	}
	return readFeaturesFromDatabase(newModels,latestProfileId);
}
bool Control::updateVisualsCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::cout << "\n############### Update visuals cB\n";
	RosParameterManager::readParameters(nh); //Updates all RosParameter objects, but do not retrain because only visual parameters changed.

	visualizer->forceRedraw = true;
	return true;
}
//Spin the cloud viewer

void Control::spinViewer()
{
	if(allowVisualization) visualizer->spinOnce();
}


