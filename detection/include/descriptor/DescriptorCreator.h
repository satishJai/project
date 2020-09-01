/*
 * DescriptorCreator.h
 *
 *  Created on: Nov 14, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORCREATOR_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORCREATOR_H_
#include "Blackboard.h"
#include "Descriptors.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class DescriptorCreator
{
	public:
		DescriptorCreator()
		{
			radius = RosParameterManager::createParam<double> (75.0,ParamType::DOUBLE,0.0,200.0,"/descriptor/radius");
		}
		virtual ~DescriptorCreator ()
		{
			RosParameterManager::deleteParam(radius);
		}
		virtual void run(const pcl::PointCloud<PointType>::ConstPtr cloud, const pcl::PointCloud<PointType>::ConstPtr keypoints,
								 const pcl::PointCloud<NormalType>::ConstPtr normals, boost::shared_ptr<Descriptors>& descriptors) = 0;
		virtual std::string getName()
		{
			return "Unknown";
		};
		virtual boost::shared_ptr<Descriptors> getEmptyDescriptor() = 0;

	protected:
		RosParameter<double>::Ptr radius;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORCREATOR_H_ */
