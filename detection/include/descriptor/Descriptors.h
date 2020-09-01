/*
 * Descriptors.h
 *
 *  Created on: Nov 14, 2018
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORS_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORS_H_

#include "Blackboard.h"
#include <pcl/correspondence.h>
#include <iostream>
#include <memory>

struct Descriptors
{
		Descriptors ():datatype("Unknown")
		{
		}

		public: virtual ~Descriptors () = default;

		std::string datatype;
		//square distance threshold is used for matching. Is individual for each descriptor type.
		float distanceThresh = 0.25;

		void setSquareDistanceThreshold(float threshold)
		{
			distanceThresh = threshold;
		}

		virtual pcl::Correspondence compare(const unsigned long id, const Descriptors* otherObject) = 0;

		virtual void clear() = 0;

		virtual unsigned long getSize() = 0;

		virtual bool savePCDFile(std::string fileName) = 0;

		virtual bool loadFromPCDFile(std::string fileName) = 0;
};

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_DESCRIPTORS_H_ */
