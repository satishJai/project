/*
 * ShotDescriptors.h
 *
 *  Created on: Nov 14, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_SHOTDESCRIPTORS_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_SHOTDESCRIPTORS_H_

#include "Descriptors.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot_omp.h>
#include<pcl/visualization/pcl_plotter.h>

struct ShotDescriptors : public Descriptors
{
		ShotDescriptors ()
		{
			datatype = "Shot";
			descriptors = boost::make_shared<pcl::PointCloud<pcl::SHOT352>>();
		}

		pcl::PointCloud<pcl::SHOT352>::Ptr descriptors;
		pcl::KdTreeFLANN<pcl::SHOT352> matching;
		bool kdTreeValid = false;

		pcl::Correspondence compare (const unsigned long id, const Descriptors* otherObject) override
		{
			//id is out of bounds
			if(id >= descriptors->size())
			{
				return pcl::Correspondence(-1,-1,-1);
			}
			//Is the other descriptor the same type?
			const ShotDescriptors* other = dynamic_cast<const ShotDescriptors*>(otherObject);
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

			//Plot descriptor histogram
			/*
			if((neighborCount >= 1) && false)
			{
				std::vector<double> x;
				for(int i = 0; i < 352; i++)
				{
					x.push_back((double)i);
				}
				pcl::visualization::PCLPlotter plotter;
				std::vector<float> hist1 (std::begin(descriptors->at(id).descriptor),std::end(descriptors->at(id).descriptor));
				std::vector<double> hist1d (hist1.begin(),hist1.end());
				//plotter.addHistogramData(hist1d,352,"Scene",std::vector<char>{(char)255,(char)0,(char)0});
				std::vector<float> hist2 (std::begin(other->descriptors->at(neighbors.at (0)).descriptor),std::end(other->descriptors->at(neighbors.at (0)).descriptor));
				std::vector<double> hist2d (hist2.begin(),hist2.end());
				//plotter.addHistogramData(hist2d, 352,"Model",std::vector<char>{(char)0,(char)255,(char)0});
				plotter.addPlotData(x,hist1d,"scene",2,std::vector<char>{(char)255,(char)0,(char)0,(char)255});
				plotter.addPlotData(x,hist2d,"model",2,std::vector<char>{(char)0,(char)255,(char)0,(char)255});
				plotter.setWindowSize(1680,1050);
				plotter.spin();
				plotter.close();
			}*/
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

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_SHOTDESCRIPTORS_H_ */
