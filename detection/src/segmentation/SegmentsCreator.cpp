/*
 * SegmentsCreator.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: satishj
 */

#include "../../include/segmentation/SegmentsCreator.h"

SegmentsCreator::SegmentsCreator()
{
	seg = std::make_shared<Segmentation>();
}

void SegmentsCreator::runSegmentation(pcl::PointCloud<PointType>::ConstPtr cloud, std::vector <Segment>* segmentVector)
{
	if(cloud->size() == 0) return;
	pcl::PointCloud<NormalType>::Ptr normals = boost::make_shared<pcl::PointCloud<NormalType>>();
	pcl::PointCloud<PointType>::Ptr sampledcloud = boost::make_shared<pcl::PointCloud<PointType>>();
	pcl::PointCloud<PointType>::Ptr sacSegment = boost::make_shared<pcl::PointCloud<PointType>>();
	std::vector<pcl::PointCloud<PointType>> cloudVector;

	seg->voxelGridSampler (cloud,  sampledcloud);
	seg->normalCreator (sampledcloud, normals);
	std::cout << "Start sacSegmentation "  << std::endl;
	seg->sacSegmentation (cloud, sacSegment);
	seg->euclideanCluster (sacSegment, cloudVector);

	segmentVector->resize(0);
	for(pcl::PointCloud<PointType> cloud : cloudVector){
		Segment seg;
		pcl::copyPointCloud(cloud,*seg.data.cloud);
		//pcl::copyPointCloud(*normals,*seg.data.normals);
		segmentVector->push_back(seg);
	}
}
