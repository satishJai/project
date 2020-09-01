/*
 * 3dscDescriptors.h
 *
 *  Created on: March 22, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_SHAPECONTEXTDESCRIPTORS_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_SHAPECONTEXTDESCRIPTORS_H_

#include "Descriptors.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/3dsc.h>

struct ShapeContextDescriptors : public Descriptors
{
		ShapeContextDescriptors ()
		{
			datatype = "ShapeContext";
			descriptors = boost::make_shared<pcl::PointCloud<pcl::ShapeContext1980>>();
		}

		pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors;
		pcl::KdTreeFLANN<pcl::ShapeContext1980> matching;
		bool kdTreeValid = false;

		pcl::Correspondence compare (const unsigned long id, const Descriptors* otherObject) override
		{
			//id is out of bounds
			if(id >= descriptors->size())
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			//Is the other descriptor the same type?
			const ShapeContextDescriptors* other = dynamic_cast<const ShapeContextDescriptors*>(otherObject);
			if(!other)
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			// Ignore NaNs.
			if(!pcl_isfinite(descriptors->at (id).descriptor[0]))
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			//KdTreeFLANN needs to be created by DescriptorCreator. Use function: matching.setInputCloud (descriptors);
			if(!other->kdTreeValid){
				return pcl::Correspondence(-1,-1,-1);
			}

			//Search nearest neighbors
			std::vector<int> neighbors;
			std::vector<float> squareDistances;
			int neighborCount = other->matching.nearestKSearch (descriptors->at (id), 1, neighbors, squareDistances);
			if(Blackboard::debugDescriptors->var) //Print debug messages if needed.
			{
				std::cout << "N= " << neighborCount << " M_ID= " << neighbors.at (0) << " Our_ID= " << static_cast<int> (id)
								<< " dist= " << squareDistances.at (0) << std::endl;
			}

			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squareDistances[0] < distanceThresh)
			{
				pcl::Correspondence corres;
				corres = pcl::Correspondence(neighbors.at(0),static_cast<int>(id), squareDistances.at(0));
				return corres;
			}
			return pcl::Correspondence(-1,-1,-1);
		}

		unsigned long getSize() override
		{
			return descriptors->size();
		}

		void clear() override
		{
			descriptors->clear();
		}

		bool savePCDFile(std::string fileName) override
		{
			if(pcl::io::savePCDFile(fileName,*descriptors) < 0) return false;
			return true;
		}

		bool loadFromPCDFile(std::string fileName) override
		{
			if(pcl::io::loadPCDFile(fileName,*descriptors) < 0) return false;
			return true;
		}
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_SHAPECONTEXTDESCRIPTORS_H_ */
