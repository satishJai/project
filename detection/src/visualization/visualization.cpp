/*
 * visualization.cpp
 *
 *  Created on: Dec 3, 2018
 *      Author: philipf
 */
#include <pcl/common/transforms.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Geometry>
#include <string>
#include "../../include/visualization/visualization.h"

Visualization::Visualization()
{
	showKeypoints = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showKeypoints");
	showNormals = RosParameterManager::createParam<bool> (false,ParamType::BOOL,"/visualization/showNormals");
	showDense = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showDense");
	showCorrespondences = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showCorrespondences");
	showEstimatedObjects = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showEstimatedObjects");
	showReferenceFrames = RosParameterManager::createParam<bool> (false,ParamType::BOOL,"/visualization/showReferenceFrames");
	showAllCorrespondences = RosParameterManager::createParam<bool> (false,ParamType::BOOL,"/visualization/showAllCorrespondences");
	showMatchingColor = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showMatchingColor");
	showOnlyWinner = RosParameterManager::createParam<bool> (true,ParamType::BOOL,"/visualization/showOnlyWinner");
	denseSize = RosParameterManager::createParam<double> (1.0,ParamType::DOUBLE,0.0,20.0,"/visualization/denseSize");
	keypointSize = RosParameterManager::createParam<double> (8.0,ParamType::DOUBLE,0.0,20.0,"/visualization/keypointSize");
	estimationSize = RosParameterManager::createParam<double> (4.0,ParamType::DOUBLE,0.0,20.0,"/visualization/estimationSize");
	colorCounter = 0;
	r = 0;
	g = 0;
	b = 0;
	sequendeIdModels = 0;
	sequendeIdScene = 0;
}
Visualization::~Visualization()
{
}
void Visualization::init (std::shared_ptr<Blackboard> gBoard, std::shared_ptr<ros::NodeHandle> nh_ptr, double gPublishRate)
{
	if(!gBoard)
	{
		std::cout << "[Visualization::init]: ERROR blackboard not valid!" << std::endl;
	}
	if(!nh_ptr)
	{
		std::cout << "[Visualization::init]: ERROR nodeHandle not valid!" << std::endl;
	}
	publishRate = gPublishRate;
	lastPublishTime = ros::Time::now();
	nh = nh_ptr;
	board = gBoard;
	visPub = nh->advertise<handling_msgs::VisualCloudUpdate>("/detection/visualUpdate",2);
}
void Visualization::spinOnce ()
{
	if(!board) return;
	if(!nh) return;
	if(forceRedraw || (counter != board->counter))//Have we already drawn the newest stuff? Then skip this.
	{
		forceRedraw = false;
		removeAllSegments();
		for(size_t segId = 0; segId < board->segments.size(); segId++)
		{
			Segment *seg = &board->segments.at(segId);
			drawCloud(&seg->data,false,segId);
			if (showCorrespondences->var)
			{
				if (showAllCorrespondences->var)
				{
					std::cout << "[Visualization]: Drawing all corres: " << std::endl;
					drawAllCorrespondences (seg,std::to_string(segId));
				}
				else if(showOnlyWinner->var)
				{
					std::cout << "[Visualization]: Drawing winner: " << seg->bestGuess.name << std::endl;
					drawCorrespondences (seg,&seg->bestGuess,&board->models.at(seg->bestGuess.modelId),"s" + std::to_string(segId));
				}else{
					for(size_t modelId = 0; modelId < seg->results.size(); modelId++)
					{
						std::cout << "[Visualization]: Drawing segment " << modelId << std::endl;
						for(size_t modelInstance = 0; modelInstance < seg->results.at(modelId).size(); modelInstance++)
						{
							std::cout << "[Visualization]: Drawing segment instance " << modelInstance << std::endl;
							drawCorrespondences (seg,&seg->results.at(modelId).at(modelInstance),&board->models.at(modelId),"s" + std::to_string(segId) + "m" + std::to_string(modelId) + "i" + std::to_string(modelInstance));
						}
					}
				}
			}
		}
		sequendeIdScene++;
		counter = board->counter;
	}
	checkAndPublishMessage();
}
void Visualization::checkAndPublishMessage()
{
	if(visPub.getNumSubscribers() == 0) return;
	if((ros::Time::now() - lastPublishTime).toSec() > (1.0/publishRate))
	{
		std::cout << "[Visualization::checkAndPublishMessage()]: Creating visualCloudUpdate message!\n";
		handling_msgs::VisualCloudUpdate msg;
		msg.LineColor = lineColor;
		msg.PointA = pointA;
		msg.PointB = pointB;
		pcl::toROSMsg(estimatedObjects,msg.estimatedObjects);
		pcl::toROSMsg(modelKeypoints,msg.modelKeypoints);
		pcl::toROSMsg(modelNormals,msg.modelNormals);
		pcl::toROSMsg(modelRFs,msg.modelRFs);
		pcl::toROSMsg(models,msg.models);
		pcl::toROSMsg(scene,msg.scene);
		pcl::toROSMsg(sceneKeypoints,msg.sceneKeypoints);
		pcl::toROSMsg(sceneNormals,msg.sceneNormals);
		pcl::toROSMsg(sceneRFs,msg.sceneRFs);
		msg.sequendeIdModels = sequendeIdModels;
		msg.sequendeIdScene = sequendeIdScene;
		visPub.publish(msg);
		std::cout << "[Visualization::checkAndPublishMessage()]: Done creating message!\n";
		lastPublishTime = ros::Time::now();
	}
}
void Visualization::drawModels()
{
	if(!board)
	{
		std::cout << "[Visualization::drawModels]: ERROR blackboard not valid!\n";
		return;
	}
	if(false)
	{
		std::cout << "[Visualization]: Visualization disabled by user.\n";
		return;
	}
	models.clear();
	modelKeypoints.clear();
	modelNormals.clear();
	modelRFs.clear();
	pointA.clear();
	pointB.clear();
	lineColor.clear();
	std::cout << "[Visualization]: Total number of models: " << board->models.size() << std::endl;
	modelOffsetCounter = 0;
	for(Model mod : board->models)
	{
		drawCloud(&mod.data,true,mod.id,mod.positionOffset);
		modelOffsetCounter++;
	}
	sequendeIdModels++;
}

void Visualization::removeAllSegments()
{
	scene.clear();
	estimatedObjects.clear();
	sceneKeypoints.clear();
	sceneNormals.clear();
	sceneRFs.clear();
	pointA.clear();
	pointB.clear();
	lineColor.clear();
}
void Visualization::drawCloud(ProcessedCloud *data, bool isModel, unsigned int id, tf::Vector3 modelOffset)
{
	if(data->cloud->size() == 0) return;
	if(showDense->var)
	{
		if(isModel)
		{
			pcl::PointCloud<PointType>::Ptr offsetedModel = boost::make_shared<pcl::PointCloud<PointType>>();
			pcl::transformPointCloud(*data->cloud,*offsetedModel, Eigen::Vector3f (modelOffset.x()*modelOffsetCounter,
																				   modelOffset.y(),
																				   modelOffset.z()), Eigen::Quaternionf (1, 0, 0, 0));
			models += *offsetedModel;
		}
		else scene += *data->cloud;
	}
	if(showNormals->var)
	{
		if(isModel)
		{
			modelNormals += *data->normals;
		}
		else sceneNormals += *data->normals;
	}
	if(showKeypoints->var)
	{
		if(isModel)
		{
			pcl::PointCloud<PointType>::Ptr offsetedModel = boost::make_shared<pcl::PointCloud<PointType>>();
			pcl::transformPointCloud(*data->keypoints,*offsetedModel, Eigen::Vector3f (modelOffset.x()*modelOffsetCounter,
																						   modelOffset.y(),
																						   modelOffset.z()), Eigen::Quaternionf (1, 0, 0, 0));
			modelKeypoints += *offsetedModel;
		}
		else sceneKeypoints += *data->keypoints;
	}
	if(showReferenceFrames->var)
	{
		if(isModel)
		{
			modelRFs += *data->rfs;
		}
		else sceneRFs += *data->rfs;
	}
}

void Visualization::drawAllCorrespondences(Segment *seg, std::string id)
{
	if(!seg->data.keypoints)
	{
		std::cout << "[Visualization]: seg->data.keypoints not valid." << std::endl;
		return;
	}
	for(size_t i = 0; i < seg->model_scene_corrs_vec.size(); i++) //For all models
	{
		for (size_t j = 0; j < seg->model_scene_corrs_vec.at(i)->size(); ++j) //For all correspondences with that model
		{
			PointType& model_point = board->models.at(i).data.keypoints->at (seg->model_scene_corrs_vec.at(i)->at(j).index_query);
			PointType& scene_point = seg->data.keypoints->at (seg->model_scene_corrs_vec.at(i)->at(j).index_match); // index_query
			geometry_msgs::Vector3 point;
			point.x = model_point.x;
			point.y = model_point.y;
			point.z = model_point.z;
			pointA.push_back(point);

			point.x = scene_point.x;
			point.y = scene_point.y;
			point.z = scene_point.z;
			pointB.push_back(point);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			if (showMatchingColor->var)
			{

				point.x = (double)model_point.r / 255.0;
				point.y = (double)model_point.g / 255.0;
				point.z = (double)model_point.b / 255.0;
				lineColor.push_back(point);
			}else
			{
				point.x = 0.0;
				point.y = 1.0;
				point.z = 0.0;
				lineColor.push_back(point);
			}
		}
	}
}
void Visualization::drawCorrespondences(Segment *seg, Result *result, Model *model, std::string id)
{
	if(result->name == "UNKNOWN")
	{
		return;
	}

	//Used so that each segemnt gets its own color, if showMatchingColor is disabled.
	r = colorCounter % 3;
	g = (colorCounter/2) % 3;
	b = (colorCounter/4) % 3;
	if(colorCounter == 0) b = 1;
	colorCounter++;

	//Draw guessed objects!
	if(showEstimatedObjects->var)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model = boost::make_shared<pcl::PointCloud<PointType>>();
		std::cout << "If segmentation fault occours right after this, Visualization::drawCorrespondences caused your problem...\n";

		Eigen::Affine3d t;
		tf::poseTFToEigen(result->pose,t);
		Eigen::Affine3f f = t.cast<float>();

		pcl::transformPointCloud(*model->data.keypoints, *rotated_model, f);
		//result->rototranslations.at(0);
		std::cout << "but this time, it did not. ;-)\n";

		int rs, gs, bs;
		rs = 255;
		gs = 0;
		bs = 0;
		if(!showMatchingColor->var)
		{
			rs = r * 255;
			gs = g * 255;
			bs = b * 255;
		}
		for(size_t i = 0; i < rotated_model->size(); i++)
		{
			rotated_model->at(i).r = rs;
			rotated_model->at(i).g = gs;
			rotated_model->at(i).b = bs;
		}
		estimatedObjects += *rotated_model;
		std::cout << "[Visualization::drawCorrespondences]: Adding estimated object (" << model->name << ") of size: " << rotated_model->size() << std::endl;
	}
	//Draw correspondences. Only correspondences which resulted into a guess will be drawn!
	if(showCorrespondences->var)
	{
		std::cout << "[Visualization::drawCorrespondences]: Drawing correspondences for " << model->name << ".\n";
		pcl::Correspondences *clustered_corrs= &result->model_scene_corrs;
		pcl::PointCloud<PointType>::Ptr model_points;
		pcl::PointCloud<PointType>::Ptr szene_points;
		model_points = model->data.keypoints;
		szene_points = seg->data.keypoints;

		for(size_t j = 0; j < clustered_corrs->size(); ++j)
		{
			PointType& model_point = model_points->at(clustered_corrs->at(j).index_query);
			PointType& scene_point = szene_points->at(clustered_corrs->at(j).index_match);

			geometry_msgs::Vector3 point;
			point.x = model_point.x;
			point.y = model_point.y;
			point.z = model_point.z;
			pointA.push_back(point);

			point.x = scene_point.x;
			point.y = scene_point.y;
			point.z = scene_point.z;
			pointB.push_back(point);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			if (showMatchingColor->var)
			{

				point.x = (double)model_point.r / 255.0;
				point.y = (double)model_point.g / 255.0;
				point.z = (double)model_point.b / 255.0;
				lineColor.push_back(point);
			}else
			{
				point.x = r;
				point.y = g;
				point.z = b;
				lineColor.push_back(point);
			}
		}
	}
}

tf::Vector3 Visualization::getModelOffset()
{
	return modelOffset;
}


