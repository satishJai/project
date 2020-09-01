/*
 * RSDDescriptors.h
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDDESCRIPTORS_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDDESCRIPTORS_H_

#include "Descriptors.h"
#include "../matching/ColorMatching.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/rsd.h>

struct RSDDescriptors : public Descriptors
{
		RSDDescriptors()
		{
			datatype = "RSD";
			descriptors = boost::make_shared<pcl::PointCloud<pcl::PrincipalRadiiRSD>>();
			descriptorWithColor = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		}

		pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr descriptors;
		pcl::PointCloud<pcl::PointXYZ>::Ptr descriptorWithColor;
		pcl::KdTreeFLANN<pcl::PointXYZ> matching;
		bool kdTreeValid = false;
		ColorMatching::hsv color;

		pcl::Correspondence compare (const unsigned long id, const Descriptors* otherObject) override
		{
			//id is out of bounds
			if (id >= descriptors->size ())
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			//Is the other descriptor the same type?
			const RSDDescriptors* other = dynamic_cast<const RSDDescriptors*>(otherObject);
			if (!other)
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			// Ignore NaNs.
			if ((descriptors->at (id).r_max != descriptors->at (id).r_max) ||
					(descriptors->at (id).r_min != descriptors->at (id).r_min))
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			//KdTreeFLANN needs to be created by DescriptorCreator. Use function: matching.setInputCloud (descriptors);
			if (!other->kdTreeValid)
			{
				return pcl::Correspondence(-1,-1,-1);
			}

			//Search nearest neighbors
			std::vector<int> neighbors;
			std::vector<float> squareDistances;
			int neighborCount = other->matching.nearestKSearch (descriptorWithColor->at (id), 1, neighbors, squareDistances);
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if(Blackboard::debugDescriptors->var) //Print debug messages if needed.
			{
				std::cout << "N= " << neighborCount << " M_ID= " << neighbors.at (0) << " Our_ID= " << static_cast<int> (id)
								<< " dist= " << squareDistances.at (0) << std::endl;
			}
			if (neighborCount == 1 && squareDistances[0] < distanceThresh)
			{
				pcl::Correspondence corres;
				corres = pcl::Correspondence (neighbors.at (0), static_cast<int> (id), squareDistances.at (0));
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

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTOR_RSDDESCRIPTORS_H_ */
