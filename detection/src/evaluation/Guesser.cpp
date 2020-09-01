/*
 * Guesser.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#include "../../include/evaluation/Guesser.h"

Guesser::Guesser()
{
	threshold = RosParameterManager::createParam<double> (20.0,ParamType::DOUBLE,0.0,100.0,"/guesser/threshold");
}
Guesser::~Guesser()
{
	RosParameterManager::deleteParam(threshold);
}

void Guesser::run(const Segment & segment, const std::vector<Model> * models, Result& guess)
{
	pcl::PointCloud<PointType>::ConstPtr cloud = segment.data.cloud;
	pcl::PointCloud<PointType>::ConstPtr keypoints = segment.data.keypoints;
	const std::vector<pcl::CorrespondencesPtr> * model_scene_corrs = &segment.model_scene_corrs_vec;
	double bestScore = 0.0;
	int mostCorres = 0;
	int secondMostCorres = 0;
	int bestModel = -1;
	double otherScore = 0.0;
	//Score == correspondences per keypoint -> 1.0 == all keypoints match. 0.0 == no match at all.
	for(int modelId = 0; modelId < model_scene_corrs->size(); modelId++)
	{
		int localCorres = model_scene_corrs->at(modelId)->size();
		double localScore = ((double)localCorres) / ((double)models->at(modelId).data.keypoints->size());
		/*
		if(localScore > bestScore && results->at(modelId).model_scene_corrs->size() > database->cg_thresh_){
			bestScore = localScore;
			mostCorres = results->at(modelId).model_scene_corrs->size();
			bestModel = modelId;
		}*/
		if((localCorres > mostCorres) && (model_scene_corrs->at(modelId)->size() > threshold->var))
		{
			if(mostCorres > secondMostCorres)
			{
				secondMostCorres = mostCorres;
			}
			mostCorres = localCorres;
			bestModel = modelId;
			bestScore = localScore;
		}else if(localCorres > secondMostCorres)
		{
			secondMostCorres = localCorres;
		}
		otherScore += localScore;
	}

	if(bestModel < 0)
	{
		std::cout << std::endl << "----------------------- No results.  ----------------------- " << std::endl;
		return;
	}


	//How many correspondences have been accepted? If the color often did not match, this will be very low...
	if(segment.notFilteredCorresCountVec.at(bestModel) != 0){
		double correspondenceRatio = ((double)model_scene_corrs->size()) / ((double)segment.notFilteredCorresCountVec.at(bestModel));
		std::cout << "(using lower threshold) Correspondence Ratio for best model " << models->at(bestModel).name <<": " << correspondenceRatio << std::endl;
		if(correspondenceRatio < 0.1){
			std::cout << "Correspondence Ratio to low.. Skipping!" << std::endl;
			std::cout << std::endl << "----------------------- No results.  ----------------------- " << std::endl;
			return;
		}
	}

	if(models->at(bestModel).data.keypoints->size() > keypoints->size()){
		std::cout << "Most corres: " << mostCorres << " second most corres: " << secondMostCorres << std::endl;
		std::cout << "BestScore: " << bestScore << "(model " << models->at(bestModel).name << ") otherScore: " << otherScore << std::endl;
		if(mostCorres > secondMostCorres * 2){
			guess.name = models->at(bestModel).name;
			guess.modelId = models->at(bestModel).id;
			guess.validPose = false;
			guess.probability = bestScore / (otherScore + bestScore);
			tf::Point center(0,0,0);
			if(guessObjectCenter(cloud,&center))
			{
				guess.pose.setOrigin(center);
				tf::Quaternion rot(1,0,0,0);
				guess.pose.setRotation(rot);
			}
		}
	}
}

bool Guesser::guessObjectCenter(pcl::PointCloud<PointType>::ConstPtr cloud, tf::Point* center)
{
	if(cloud->size() == 0) return false;

	float x = 0.0;
	float y = 0.0;
	float z = cloud->at(0).z;
	float size = cloud->size();

	for(unsigned int i = 0; i < cloud->size(); i++){
		x += cloud->at(i).x / size;
		y += cloud->at(i).y / size;
		if(cloud->at(i).z < z){
			z = cloud->at(i).z;
		}
	}
	center->setX(x);
	center->setY(y);
	center->setZ(z);
	return true;
}
