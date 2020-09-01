/*
 * Segmentation.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: satishj
 */

#include "../../include/segmentation/Segmentation.h"
Segmentation::Segmentation()
{
	voxelLeafSize = RosParameterManager::createParam<double> (0.01,ParamType::DOUBLE,0.0,1.0,"/segmentation/voxelLeafSize");
	kdradius = RosParameterManager::createParam<int> (50,ParamType::INT,1,100,"/segmentation/kdradius");

	maxIterations = RosParameterManager::createParam<int> (100,ParamType::INT,1,100,"/segmentation/sac/maxIterations");
	distanceThreshold = RosParameterManager::createParam<double> (0.009,ParamType::DOUBLE,0.0,1.0,"/segmentation/sac/distanceThreshold");
	minClusterSize = RosParameterManager::createParam<int> (400,ParamType::INT,0,10000,"/segmentation/euclidean/minClusterSize");
	maxClusterSize = RosParameterManager::createParam<int> (10000,ParamType::INT,0,100000,"/segmentation/euclidean/maxClusterSize");
	clusterTolerance = RosParameterManager::createParam<double> (0.01,ParamType::DOUBLE,0,1,"/segmentation/euclidean/clusterTolerance");
	cloudPointSize = RosParameterManager::createParam<double> (0.3,ParamType::DOUBLE,0,1,"/segmentation/sac/cloudPointSize");
}

void Segmentation::voxelGridSampler (pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr sampledcloud)
{
	std::cout << "Segmentation starts with: " << cloud->points.size ()  << " data points." << std::endl;
	pcl::VoxelGrid<PointType> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (voxelLeafSize->var, voxelLeafSize->var, voxelLeafSize->var);
	vg.filter (*sampledcloud);
	std::cout << "PointCloud after filtering has: " << sampledcloud->points.size ()  << " data points." << std::endl;
}

void Segmentation::normalCreator (pcl::PointCloud<PointType>::Ptr sampledcloud, pcl::PointCloud<NormalType>::Ptr normals)
{
	pcl::search::KdTree<PointType>::Ptr tree;
	pcl::NormalEstimation<PointType, NormalType> ne;
	ne.setSearchMethod (tree);
	ne.setInputCloud (sampledcloud);
	ne.setKSearch (kdradius->var);
	ne.compute (*normals);
}

void Segmentation::sacSegmentation (pcl::PointCloud<PointType>::ConstPtr sampledcloud, pcl::PointCloud<PointType>::Ptr sacSegment)
{
	if(sampledcloud->size() == 0) return;
	std::cout << "Starting sacSegmentation "  << std::endl;
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::ExtractIndices<PointType> extract;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::PointCloud<PointType>::Ptr planeSegments (new pcl::PointCloud<PointType> ());
	pcl::PointCloud<PointType>::Ptr tmpCloud (new pcl::PointCloud<PointType> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations->var);
	seg.setDistanceThreshold (distanceThreshold->var);

	*tmpCloud = *sampledcloud;
	int nr_points = sampledcloud->points.size ();
	int exitCounter = 10;
	while ((tmpCloud->points.size () > (cloudPointSize->var * nr_points)) && (exitCounter > 0))
	{
		seg.setInputCloud (sampledcloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		extract.setInputCloud (sampledcloud);
		extract.setIndices (inliers);

		//extract.setNegative (false);
		// Get the points associated with the planar surface
		//extract.filter (*planeSegments);
		//std::cout << "PointCloud representing the planar component: " << planeSegments->points.size () << " data points." << std::endl;

		extract.setNegative (true);
		extract.filter (*sacSegment);
		*tmpCloud = *sacSegment;
		exitCounter--;
	}
	if(exitCounter == 0)
	{
		std::cout << "ERROR Segmentation::[sacSegmentation] infinite loop!\n";
	}
	*sacSegment = *tmpCloud;
}

void Segmentation::euclideanCluster (pcl::PointCloud<PointType>::ConstPtr sacSegment, std::vector < pcl::PointCloud<PointType>>& cloudVector)
{
	if(sacSegment->size() == 0) return;
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	pcl::EuclideanClusterExtraction<PointType> ec;
	std::vector<pcl::PointIndices> cluster_indices;
	tree->setInputCloud (sacSegment);
	ec.setClusterTolerance (clusterTolerance->var);
	ec.setMinClusterSize (minClusterSize->var);
	ec.setMaxClusterSize (maxClusterSize->var);
	ec.setSearchMethod (tree);
	ec.setInputCloud (sacSegment);
	ec.extract (cluster_indices);

	  std::cout << "Started clustering " << std::endl;
	int j = 0;
	cloudVector.clear ();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointType>::Ptr cloudCluster (new pcl::PointCloud<PointType>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloudCluster->points.push_back (sacSegment->points[*pit]); //*
		cloudCluster->width = cloudCluster->points.size ();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;
		std::cout << "Creating Cluster_" << j << std::endl;
		std::cout << "Cluster size is " << cloudCluster->points.size () << std::endl;
		pcl::PointCloud<PointType> cloud;
		pcl::copyPointCloud (*cloudCluster, cloud);
		cloudVector.push_back (cloud);
		j++;
	}
	std::cout << "Clustering Completed, with: " << cloudVector.size () << " clusters" << std::endl;
}

