/*
 * Segmentation.h
 *
 *  Created on: Nov 26, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTATION_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTATION_H_

#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../RosParameter.h"
#include "../RosParameterManager.h"
#include "../Blackboard.h"

class Segmentation
{
	public:
		Segmentation();
		/*for downsampling the input cloud*/
		void voxelGridSampler (pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr sampledcloud);
		/*Creting the normals for each points*/
		void normalCreator (pcl::PointCloud<PointType>::Ptr sampledcloud, pcl::PointCloud<NormalType>::Ptr normals);
		/*Estimating the plans*/
		void sacSegmentation (pcl::PointCloud<PointType>::ConstPtr cloud, pcl::PointCloud<PointType>::Ptr sacSegment);
		void regionGrowing (pcl::RegionGrowing<PointType, NormalType> regGrowing);
		/*Creating the clusters*/
		void euclideanCluster (pcl::PointCloud<PointType>::ConstPtr sacSegment, std::vector < pcl::PointCloud<PointType>>& cloudVector);

	private:
		RosParameter<double>::Ptr voxelLeafSize; //creating the ros parameters
		RosParameter<int>::Ptr kdradius;
		RosParameter<int>::Ptr maxIterations;
		RosParameter<double>::Ptr distanceThreshold;
		RosParameter<int>::Ptr minClusterSize;
		RosParameter<int>::Ptr maxClusterSize;
		RosParameter<double>::Ptr clusterTolerance;
		RosParameter<double>::Ptr cloudPointSize;

};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_SEGMENTATION_SEGMENTATION_H_ */
