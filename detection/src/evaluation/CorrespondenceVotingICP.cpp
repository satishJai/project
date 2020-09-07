/*
 * CorrespondenceVoting.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: satishj
 */
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <eigen3/Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../../include/evaluation/CorrespondenceVotingICP.h"

CorrespondenceVotingICP::CorrespondenceVotingICP()
{
	correspondenceRatioThreshold = RosParameterManager::createParam<double> (0.15,ParamType::DOUBLE,0.0,1.0,"/CorrespondenceVoting/correspondenceRatioThreshold");
	maxIterations = RosParameterManager::createParam<int> (200,ParamType::INT,0,200,"/evaluation/ICP/maxIterations");
	resolution = RosParameterManager::createParam<double> (5.0,ParamType::DOUBLE,0.0,50.0,"/evaluation/ICP/resolution");
	transformationEpsilon = RosParameterManager::createParam<double> (0.00000001,ParamType::DOUBLE,0.0,0.001,"/evaluation/ICP/transformationEpsilon");
}
CorrespondenceVotingICP::~CorrespondenceVotingICP()
{
	RosParameterManager::deleteParam(correspondenceRatioThreshold);
}

bool CorrespondenceVotingICP::getICPPose(pcl::PointCloud<PointType>::Ptr source, pcl::PointCloud<PointType>::Ptr destination, Result& guess, tf::Pose& pose)
{
	std::cout << "Start icp!\n";
	pcl::PointCloud<PointType>::Ptr cloudInVox (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr parentInVox (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr tmp (new pcl::PointCloud<PointType>);

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(source);
	sor.setLeafSize(resolution->var*Blackboard::RESOLUTION, resolution->var*Blackboard::RESOLUTION, resolution->var*Blackboard::RESOLUTION);
	sor.filter(*cloudInVox);

	sor.setInputCloud(destination);
	sor.setLeafSize(resolution->var*Blackboard::RESOLUTION, resolution->var*Blackboard::RESOLUTION, resolution->var*Blackboard::RESOLUTION);
	sor.filter(*parentInVox);

	std::cout << "cloudIn: " << cloudInVox->size() << " parentIn: " << parentInVox->size() << std::endl;

	Eigen::Affine3d t1;
	tf::poseTFToEigen(pose,t1);
	Eigen::Affine3f f2 = t1.cast<float>();
	pcl::transformPointCloud(*cloudInVox, *cloudInVox, f2);

	*tmp = *cloudInVox;

	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setInputSource(cloudInVox);
	icp.setInputTarget(parentInVox);
	icp.setMaximumIterations(maxIterations->var);
	icp.setMaxCorrespondenceDistance(100000.0);
	icp.setTransformationEpsilon(transformationEpsilon->var);
	icp.align(*tmp);

	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerCustom<PointType> in(tmp, 0, 255, 0);
	viewer.addPointCloud(tmp, in, "in");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> in2(cloudInVox, 0, 0, 255);
	viewer.addPointCloud(cloudInVox, in2, "in2");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> in3(parentInVox, 255, 0, 0);
	viewer.addPointCloud(parentInVox, in3, "in3");
	viewer.spin();

	if(!icp.hasConverged())
	{
		std::cout << "ICP has NOT converged!\n";
		return false;
	}

	std::cout << "ICP Error: " <<  icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f t = icp.getFinalTransformation();
	Eigen::Affine3f a;
	a.matrix() = t;
	Eigen::Affine3d b = a.cast<double>();
	tf::Transform p;
	tf::poseEigenToTF(b,p);
	tf::Transform loc(tf::Quaternion(0,0,0,1),p.getOrigin());
	tf::Transform rot(p.getRotation(),tf::Vector3(0,0,0));
	pose = p;
	//pose = rot * loc * pose;
	//pose = loc * rot * pose;
	return true;
}

bool CorrespondenceVotingICP::run(Segment& segment, Result& solution, std::vector<Model> &models)
{
	const std::vector<std::vector<Result>>& results = segment.results;
	//Vote for most instances
	int maxInstanceModelId = -1;
	int bestInstancesInstanceId = -1;
	int maxInstancesCount = 0;
	int bestInstanceCorrespondencesCount = 0;

	//Vote for best instance which has most correspondences
	int maxCorrespondencesModelId = -1;
	int maxCorrespondencesCorrId = -1;
	int maxCorrespondencesCount = 0;

	//check correspondences which have been removed because of color
	double totalCorrespondences = 0;

	//Vote for highest correspondence average
	int maxAverageModelId = -1;
	int maxAverageCorrId = -1;
	float bestCorrespondenceAverage = 0.0;
	int totalCorrespondenceCount = 0;

	std::cout << std::endl << "----------------------- EVALUATING result ----------------------- " << std::endl;
	/*Lets pick the model which might be the best choice for this segment! This will check color rejection, most instances, best instance, average correspondences per instance.*/
	for(unsigned int modelId = 0; modelId < results.size(); modelId++)
	{
		const std::vector<Result>& modelResults = results.at(modelId);
		bool hasMostInstances = false;
		float correspondenceAverage = 0.0;
		int bestCorrespondenceCount = 0;
		int bestCorrespondenceId = -1;

		//How many correspondences have been accepted? If the color often did not match, this will be very low...
		if(segment.notFilteredCorresCountVec.at(modelId) != 0)
		{
			double correspondenceRatio = ((double)segment.model_scene_corrs_vec.at(modelId)->size()) / ((double)segment.notFilteredCorresCountVec.at(modelId));
			std::cout << "model_scene_corrs_vec: " << segment.model_scene_corrs_vec.at(modelId)->size() << " notFilteredCorrs: " << segment.notFilteredCorresCountVec.at(modelId) << std::endl;
			totalCorrespondences += correspondenceRatio;
			if(correspondenceRatio < correspondenceRatioThreshold->var)
			{
				std::cout << "Correspondence Ratio to low.. Skipping!" << correspondenceRatio << " < " << correspondenceRatioThreshold->var << std::endl;
				continue;
			}
		}
		//If the segment is bigger than our model, this cant be right!
		if(sizeChecking->var)
		{
			if(segment.data.getDiameter() > (models.at(modelId).data.getDiameter())*1.1)
			{
				std::cout << models.at(modelId).name << " diameter is to small! S="
						<< segment.data.getDiameter() << " M= " << models.at(modelId).data.getDiameter() << std::endl;
				continue;
			}
		}

		//store which model has more instances
		if(modelResults.size() > size_t(maxInstancesCount))
		{
			maxInstanceModelId = modelId;
			maxInstancesCount = modelResults.size();
			bestInstancesInstanceId = -1;
			bestInstanceCorrespondencesCount = 0;
			hasMostInstances = true;
			std::cout << "best current model: "<< maxInstanceModelId << std::endl;
		}

		//store the instance that has the most correspondences
		for(unsigned int guessId = 0; guessId < modelResults.size(); guessId++)
		{
			if(size_t(bestCorrespondenceCount) < modelResults.at(guessId).model_scene_corrs.size())
			{
				bestCorrespondenceCount = modelResults.at(guessId).model_scene_corrs.size();
				bestCorrespondenceId = guessId;
			}
			correspondenceAverage += ((float)modelResults.at(guessId).model_scene_corrs.size()) / ((float)modelResults.size()); //Might be a mistake. Might be results.size()
			totalCorrespondenceCount += modelResults.at(guessId).model_scene_corrs.size();
		}

		//Has this model the instance with most correspondences?
		if(maxCorrespondencesCount < bestCorrespondenceCount){
			maxCorrespondencesCount = bestCorrespondenceCount;
			maxCorrespondencesModelId = modelId;
			maxCorrespondencesCorrId = bestCorrespondenceId;
			std::cout << "best model: "<< maxCorrespondencesModelId << " has " << maxCorrespondencesCount << " corres." << std::endl;
		}

		//If this model has the most intances, store the best instance
		if(hasMostInstances){
			bestInstanceCorrespondencesCount = bestCorrespondenceCount;
			bestInstancesInstanceId = bestCorrespondenceId;
			std::cout << "Storing " << bestInstancesInstanceId << " as most corres." << std::endl;
		}

		//If this model has the best average correspondneces count, store this
		if(correspondenceAverage > bestCorrespondenceAverage){
			maxAverageModelId = modelId;
			maxAverageCorrId = bestCorrespondenceId;
			bestCorrespondenceAverage = correspondenceAverage;
			std::cout << "Model " << modelId << " has the best average correspondence count: " << bestCorrespondenceAverage << std::endl;
		}
	}
	std::cout << "Model: " << maxInstanceModelId << " has most guesses: " << maxInstancesCount << ". Its best guess has " << bestInstanceCorrespondencesCount << " correspondences at instance: " << bestInstancesInstanceId << std::endl;
	std::cout << "Model: " << maxCorrespondencesModelId << " has the most correspondences: " << maxCorrespondencesCount << " at instance: " << maxCorrespondencesCorrId << std::endl;

	if(maxAverageModelId == -1 && maxCorrespondencesModelId == -1){
		std::cout << std::endl << "----------------------- No results. ----------------------- " << std::endl;
		std::cout << std::endl << "----------------------- done EVALUATING result ----------------------- " << std::endl;
		return false;
	}

	std::cout << "Selecting pose:" << std::endl;

	//Select if maxCorrespondences or more guesses is better:
	if((bestCorrespondenceAverage) >= maxCorrespondencesCount)
	{
		solution.modelId = maxAverageModelId;
		solution.name = results.at(maxAverageModelId).at(0).name;

		double correspondenceRatio = ((double)results.at(maxAverageModelId).size()) / ((double)segment.notFilteredCorresCountVec.at(maxAverageModelId));
		solution.probability = correspondenceRatio / totalCorrespondences;
		//obj.probability = bestCorrespondenceAverage / ((float)totalCorrespondenceCount);
		solution.validPose = true;
		solution.pose = results.at(maxAverageModelId).at(maxAverageCorrId).pose;
		solution.rototranslations = results.at(maxAverageModelId).at(maxAverageCorrId).rototranslations;
		solution.model_scene_corrs = results.at(maxAverageModelId).at(maxAverageCorrId).model_scene_corrs;
		getICPPose(models.at(solution.modelId).data.cloud,segment.data.cloud,solution,solution.pose);
	}else
	{
		solution.modelId = maxCorrespondencesModelId;
		solution.name = results.at(maxCorrespondencesModelId).at(0).name;
		double correspondenceRatio = ((double)results.at(maxCorrespondencesModelId).size()) / ((double)segment.notFilteredCorresCountVec.at(maxCorrespondencesModelId));
		solution.probability = correspondenceRatio / totalCorrespondences;
		solution.validPose = true;
		solution.pose = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).pose;
		solution.rototranslations = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).rototranslations;
		solution.model_scene_corrs = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).model_scene_corrs;
		getICPPose(models.at(solution.modelId).data.cloud,segment.data.cloud,solution,solution.pose);
	}
	return true;
	std::cout << std::endl << "----------------------- done EVALUATING result ----------------------- " << std::endl;
}


