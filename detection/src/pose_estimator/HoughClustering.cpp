/*
 * HoughClustering.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: satishj
 */

#include "../../include/pose_estimator/HoughClustering.h"

HoughClustering::HoughClustering()
{
	hgThreshold = RosParameterManager::createParam<double> (1.0,ParamType::DOUBLE,0.0,100.0,"/poseEstimation/hough/hgThreshold");
	hgSize = RosParameterManager::createParam<double> (45.0,ParamType::DOUBLE,0.0,200.0,"/poseEstimation/hough/hgSize");
}

void HoughClustering::run(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<RFType>::ConstPtr rf, pcl::PointCloud<PointType>::ConstPtr keypoints, const Model& model,
		const pcl::CorrespondencesPtr& correspondences, std::vector<Result>* results)
{
	if(!correspondences) return;
	if(correspondences->size() == 0)
	{
		std::cout << "[HoughClustering]: No Correspondences given!\n";
		return;
	}
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
	clusterer.setHoughBinSize (hgSize->var * Blackboard::RESOLUTION);
	clusterer.setHoughThreshold (hgThreshold->var);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (true);
	clusterer.setInputCloud (model.data.keypoints);
	clusterer.setInputRf (model.data.rfs);
	clusterer.setSceneCloud (keypoints);
	clusterer.setSceneRf (rf);
	clusterer.setModelSceneCorrespondences (correspondences);
	clusterer.recognize (rototranslations, clustered_corrs);

	std::cout << model.name << " instances found: " << clustered_corrs.size () << std::endl;
	for (size_t i = 0; i < rototranslations.size (); ++i)
	{
		std::cout << "Instance " << (i + 1) << ":" << std::endl;
		std::cout << "\tHas " << clustered_corrs.at (i).size () << " correspondences." << std::endl;

		Eigen::Matrix3f rotation = rototranslations.at (i).block<3, 3> (0, 0);
		Eigen::Vector3f translation = rototranslations.at (i).block<3, 1> (0, 3);
		printf ("\t\t    | %6.3f %6.3f %6.3f | \n", rotation (0, 0), rotation (0, 1), rotation (0, 2));
		printf ("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation (1, 0), rotation (1, 1), rotation (1, 2));
		printf ("\t\t    | %6.3f %6.3f %6.3f | \n", rotation (2, 0), rotation (2, 1), rotation (2, 2));
		std::cout << std::endl;
		printf ("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	}

	//Convert the transform matrix, of each cluster of correspondences, from geometric consistency grouping to tf::Pose and put it into std::vector<Result>
	results->clear();
	results->resize(clustered_corrs.size());
	for(unsigned int i = 0; i < clustered_corrs.size(); i++)
	{
		results->at(i).modelId = model.id;
		results->at(i).name = model.name;
		results->at(i).probability = 1.0; //TODO
		results->at(i).rototranslations.clear();
		results->at(i).rototranslations.push_back(rototranslations.at(i));
		results->at(i).model_scene_corrs = clustered_corrs.at(i);

		//Split transform matrix into rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations.at(i).block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations.at(i).block<3, 1>(0, 3);

		//Copy from Eigen::Vector3f to tf::Vector3
		tf::Vector3 translationTf;
		translationTf.setValue(static_cast<double>(translation(0)),static_cast<double>(translation(1)),static_cast<double>(translation(2)));

		//Copy from Eigen::Matrix3f to tf::Matrix3x3
		tf::Matrix3x3 rotationTf;
		rotationTf.setValue(static_cast<double>(rotation(0,0)),static_cast<double>(rotation(0,1)),static_cast<double>(rotation(0,2)),
								static_cast<double>(rotation(1,0)),static_cast<double>(rotation(1,1)),static_cast<double>(rotation(1,2)),
								static_cast<double>(rotation(2,0)),static_cast<double>(rotation(2,1)),static_cast<double>(rotation(2,2)));

		//Set tf::pose by using tf::Vector3 and tf::Matrix3x3
		results->at(i).pose.setOrigin(translationTf);
		results->at(i).pose.setBasis(rotationTf);
		results->at(i).validPose = true;
	}
}



