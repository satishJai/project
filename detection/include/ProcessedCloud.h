/*
 * ProcessedCloud.h
 *
 *  Created on: Nov 20, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_PROCESSEDCLOUD_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_PROCESSEDCLOUD_H_

#include "descriptor/Descriptors.h"
#include "descriptor/DescriptorCreator.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/board.h>
#include <pcl/io/pcd_io.h>
#include "Blackboard.h"
#include "RosParameter.h"
#include "RosParameterManager.h"

struct ProcessedCloud
{
	public:
		pcl::PointCloud<PointType>::Ptr cloud;
		pcl::PointCloud<PointType>::Ptr keypoints;
		pcl::PointCloud<RFType>::Ptr rfs;
		pcl::PointCloud<NormalType>::Ptr normals;
		boost::shared_ptr<Descriptors> descriptors;
	private:
		double diameter = -1.0;
	public:

		ProcessedCloud()
		{
			cloud = boost::make_shared<pcl::PointCloud<PointType>>();
			keypoints = boost::make_shared<pcl::PointCloud<PointType>>();
			rfs = boost::make_shared<pcl::PointCloud<RFType>>();
			normals = boost::make_shared<pcl::PointCloud<NormalType>>();
		}

		/*Returns  the diameter of the cloud.*/
		double getDiameter()
		{
			if(cloud->size() == 0) return 0.0;

			if(diameter < 0)
			{
				pcl::PointXYZ max;
				double maxSum = cloud->at(0).x + cloud->at(0).y + cloud->at(0).z;
				max.x = cloud->at(0).x;
				max.y = cloud->at(0).y;
				max.z = cloud->at(0).z;
				pcl::PointXYZ min;
				double minSum = cloud->at(0).x + cloud->at(0).y + cloud->at(0).z;
				min.x = cloud->at(0).x;
				min.y = cloud->at(0).y;
				min.z = cloud->at(0).z;

				for(size_t i = 1; i < cloud->size(); i++)
				{
					double sum = cloud->at(i).x + cloud->at(i).y + cloud->at(i).z;
					if(maxSum < sum)
					{
						maxSum = sum;
						max.x = cloud->at(i).x;
						max.y = cloud->at(i).y;
						max.z = cloud->at(i).z;
					}
					if(minSum > sum)
					{
						minSum = sum;
						min.x = cloud->at(i).x;
						min.y = cloud->at(i).y;
						min.z = cloud->at(i).z;
					}

					/*
					if(cloud->at(i).x > max.x) max.x = cloud->at(i).x;
					if(cloud->at(i).y > max.y) max.y = cloud->at(i).y;
					if(cloud->at(i).z > max.z) max.z = cloud->at(i).z;

					if(cloud->at(i).x < min.x) min.x = cloud->at(i).x;
					if(cloud->at(i).y < min.y) min.y = cloud->at(i).y;
					if(cloud->at(i).z < min.z) min.z = cloud->at(i).z;*/
				}
				diameter = std::sqrt(std::pow(max.x - min.x,2.0) + std::pow(max.y - min.y,2.0) + std::pow(max.z - min.z,2.0));
			}
			return diameter;
		}

		void createNormals(){
			pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
			norm_est.setInputCloud (cloud);
			pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
			norm_est.setSearchMethod (tree);
			norm_est.setKSearch(Blackboard::normalRadius->var);
			//norm_est.setRadiusSearch (RESOLUTION * 25.0);
			norm_est.compute (*normals);
		}

		void createRF()
		{
			pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
			rf_est.setFindHoles (true);
			rf_est.setRadiusSearch (Blackboard::rfRadius->var * Blackboard::RESOLUTION);
			rf_est.setInputCloud (keypoints); //TODO is this okay?
			if (cloud->size () >= 0 && normals->size () == 0)
			{
				createNormals ();
			}
			rf_est.setInputNormals (normals); //TODO is this okay?
			rf_est.setSearchSurface (cloud);
			rf_est.compute (*rfs);
		}

		bool saveAtLocation(std::string keypointFile, std::string descriptorFile, std::string normalFile, std::string referenceFrameFile)
		{
			bool error = false;

			if(keypointFile.size() == 0)
			{
				return false;
			}
			boost::filesystem::path p = boost::filesystem::path(keypointFile).parent_path();
			boost::filesystem::path profilePath = p.parent_path();
			if(!boost::filesystem::exists(profilePath))//creating profile folder
			{
				std::cout << "Creating directory " << profilePath.c_str() << std::endl;
				if(!boost::filesystem::create_directory(profilePath))
				{
					std::cout << "Unable to create directory " << profilePath.c_str() << std::endl;
					return false;
				}
			}
			//Create folders for each model if neccecary. then save the model!
			if(!boost::filesystem::exists(p))
			{
				std::cout << "Creating directory " << p.c_str() << std::endl;
				if(!boost::filesystem::create_directory(p))
				{
					std::cout << "Unable to create directory " << p.c_str() << std::endl;
					return false;
				}
			}
			if(pcl::io::savePCDFile(keypointFile,*keypoints) < 0)
			{
				std::cout << "Unable to save keypoints at " <<  keypointFile << std::endl;
				error = true;
			}

			if(normalFile.size() == 0)
			{
				return false;
			}
			p = boost::filesystem::path(normalFile).parent_path();
			if(!boost::filesystem::exists(p))
			{
				std::cout << "Creating directory " << p.c_str() << std::endl;
				if(!boost::filesystem::create_directory(p))
				{
					std::cout << "Unable to create directory " << p.c_str() << std::endl;
					return false;
				}
			}
			if(pcl::io::savePCDFile(normalFile,*normals) < 0)
			{
				std::cout << "Unable to save normals at " <<  normalFile << std::endl;
				error = true;
			}

			if(referenceFrameFile.size() == 0)
			{
				return false;
			}
			p = boost::filesystem::path(referenceFrameFile).parent_path();
			if(!boost::filesystem::exists(p))
			{
				std::cout << "Creating directory " << p.c_str() << std::endl;
				if(!boost::filesystem::create_directory(p))
				{
					std::cout << "Unable to create directory " << p.c_str() << std::endl;
					return false;
				}
			}
			if(pcl::io::savePCDFile(referenceFrameFile,*rfs) < 0)
			{
				std::cout << "Unable to save reference frames at " <<  referenceFrameFile << std::endl;
				error = true;
			}

			if(descriptorFile.size() == 0)
			{
				return false;
			}
			p = boost::filesystem::path(descriptorFile).parent_path();
			if(!boost::filesystem::exists(p))
			{
				std::cout << "Creating directory " << p.c_str() << std::endl;
				if(!boost::filesystem::create_directory(p))
				{
					std::cout << "Unable to create directory " << p.c_str() << std::endl;
					return false;
				}
			}
			if(!descriptors->savePCDFile(descriptorFile))
			{
				std::cout << "Unable to save descriptors at " <<  descriptorFile << std::endl;
				error = true;
			}
			return error;
		}

		int loadFromLocation(std::string modelFile, std::string keypointFile, std::string descriptorFile, std::string normalFile, std::string referenceFrameFile,
				bool skipModel, std::shared_ptr<DescriptorCreator> descrCreator)
		{
			if(!skipModel || (cloud->size() == 0))
			{
				std::cout << "Loading Model" << modelFile << std::endl;
				if(pcl::io::loadPCDFile(modelFile,*cloud) < 0)
				{
					std::cout << "Unable to load model at " <<  modelFile << std::endl;
					return -1;
				}
			}
			std::cout << "keypointFile: " << keypointFile << std::endl;
			if(pcl::io::loadPCDFile(keypointFile,*keypoints) < 0)
			{
				std::cout << "Unable to load keypoints at " <<  keypointFile << std::endl;
				return 1;
			}
			std::cout << "referenceFrameFile: " << referenceFrameFile << std::endl;
			if(pcl::io::loadPCDFile(referenceFrameFile,*rfs))
			{
				std::cout << "Unable to load reference frames at " <<  referenceFrameFile << std::endl;
				return 1;
			}
			std::cout << "normalFile: " << normalFile << std::endl;
			if(pcl::io::loadPCDFile(normalFile,*normals))
			{
				std::cout << "Unable to load normals at " <<  normalFile << std::endl;
				return 1;
			}
			std::cout << "descriptorFile: " << descriptorFile << std::endl;
			if(!descriptors)
			{
				descriptors = descrCreator->getEmptyDescriptor();
			}
			if(!descriptors->loadFromPCDFile(descriptorFile))
			{
				std::cout << "Unable to load descriptors at " <<  descriptorFile << std::endl;
				return 1;
			}
			std::cout << "Returning\n";
			return 0;
		}
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_PROCESSEDCLOUD_H_ */
