/*
 * ColorMatching.h
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_COLORMATCHING_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_COLORMATCHING_H_

#include "Matching.h"
#include "../descriptor/Descriptors.h"
#include "../RosParameter.h"
#include "../RosParameterManager.h"

class ColorMatching : public Matching
{
	public:
		ColorMatching();
		void run(Descriptors& descriptors, pcl::PointCloud<PointType>::ConstPtr keypoints, const std::vector<Model>* models,
			 std::vector<pcl::CorrespondencesPtr>* model_scene_corrs_vec, std::vector<unsigned long> *notFilteredCorresCountVec) override;
		static bool isSimilarColor(const PointType scene, const PointType model, double nBlackThreshold, double nGrayThreshold, double nAngleThresh, double nValueThresh);

	protected:
		RosParameter<double>::Ptr blackThreshold;
		RosParameter<double>::Ptr grayThreshold;
		RosParameter<double>::Ptr angleThreshold;
		RosParameter<double>::Ptr valueThreshold;

	public:
		/*-------------------- Structures for color matching --------------------*/
		struct rgb
		{
				double r;       // a fraction between 0 and 1
				double g;       // a fraction between 0 and 1
				double b;       // a fraction between 0 and 1
				rgb ()
				{
					r = 0.0;
					g = 0.0;
					b = 0.0;
				}
				rgb (double sr, double sg, double sb)
				{
					r = sr;
					g = sg;
					b = sb;
				}
		};

		struct hsv
		{
				double h;       // angle in degrees
				double s;       // a fraction between 0 and 1
				double v;       // a fraction between 0 and 1
				hsv ()
				{
					h = 0.0;
					s = 0.0;
					v = 0.0;
				}
				hsv (double sh, double ss, double sv)
				{
					h = sh;
					s = ss;
					v = sv;
				}
		};

		/*-------------------- Functions for color matching --------------------*/
		//https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both#
		static hsv rgb2hsv (rgb in);

		static rgb hsv2rgb (hsv in);
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_MATCHING_COLORMATCHING_H_ */
