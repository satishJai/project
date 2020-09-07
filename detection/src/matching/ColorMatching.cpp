/*
 * ColorMatching.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: satishj
 */
#include "../../include/matching/ColorMatching.h"
#include <pcl/correspondence.h>

ColorMatching::ColorMatching()
{
	blackThreshold = RosParameterManager::createParam<double> (0.175,ParamType::DOUBLE,0.0,1.0,"/matching/colorMatching/blackThreshold");
	grayThreshold = RosParameterManager::createParam<double> (0.3,ParamType::DOUBLE,0.0,1.0,"/matching/colorMatching/grayThreshold");
	angleThreshold = RosParameterManager::createParam<double> (20.0,ParamType::DOUBLE,0.0,360.0,"/matching/colorMatching/angleThreshold");
	valueThreshold = RosParameterManager::createParam<double> (0.25,ParamType::DOUBLE,0.0,1.0,"/matching/colorMatching/valueThreshold");
}

void ColorMatching::run(Descriptors& descriptors, pcl::PointCloud<PointType>::ConstPtr keypoints, const std::vector<Model>* models,
		 std::vector<pcl::CorrespondencesPtr>* model_scene_corrs_vec, std::vector<unsigned long> *notFilteredCorresCountVec)
{
	model_scene_corrs_vec->clear();
	model_scene_corrs_vec->resize(models->size ());
	notFilteredCorresCountVec->clear();
	notFilteredCorresCountVec->resize(models->size());

	double blackThresholdVar = blackThreshold->var;
	double grayThresholdVar = grayThreshold->var;
	double angleThresholdVar = angleThreshold->var;
	double valueThresholdVar = valueThreshold->var;
	// Check every model.
	#pragma omp parallel for shared(models, descriptors, keypoints, model_scene_corrs_vec, notFilteredCorresCountVec, blackThresholdVar, grayThresholdVar, angleThresholdVar, valueThresholdVar)
	for (unsigned long j = 0; j < models->size (); j++)
	{
		//All correspondences with the current model are stored here
		pcl::CorrespondencesPtr model_scene_corrs = boost::make_shared<pcl::Correspondences>();
		Model model = models->at(j);
		unsigned long notFilteredCorresCount = 0;

		// Check every descriptor computed for the scene.
		for (unsigned long i = 0; i < descriptors.getSize (); ++i)
		{
			//Check for correspondences with model
			pcl::Correspondence corres = descriptors.compare(i,model.data.descriptors.get());
			if(corres.index_match != corres.index_match) continue;
			if(corres.index_match < 0) continue;
			notFilteredCorresCount++;
			if(isSimilarColor(keypoints->at(i),model.data.keypoints->at(corres.index_query),blackThresholdVar,grayThresholdVar,angleThresholdVar,valueThresholdVar))
			{
				model_scene_corrs->push_back (corres);
			}
		}

		std::cout << "Found " << notFilteredCorresCount << " correspondences! Found " << model_scene_corrs->size() << " color corres with "<< models->at(j).name << "!" << std::endl;
		//#pragma omp critical
		notFilteredCorresCountVec->at(j) = notFilteredCorresCount;
		//All correspondences with the current model will be put into an array. E.g. if you access this array at position 0, you can get all correspondences with model0
		//#pragma omp critical
		model_scene_corrs_vec->at(j) = model_scene_corrs;
	}
}
bool ColorMatching::isSimilarColor(const PointType scene, const PointType model, double nBlackThreshold, double nGrayThreshold, double nAngleThresh, double nValueThresh){
	rgb scene_rgb = rgb(((double)scene.r)/255.0,((double)scene.g)/255.0,((double)scene.b)/255.0);
	rgb model_rgb = rgb(((double)model.r)/255.0,((double)model.g)/255.0,((double)model.b)/255.0);

	hsv scene_hsv = rgb2hsv(scene_rgb);
	hsv model_hsv = rgb2hsv(model_rgb);

	//is it black or gray?
	bool sceneBlack = scene_hsv.v <= nBlackThreshold;
	bool modelBlack = model_hsv.v <= nBlackThreshold;
	bool sceneGrayImage = (scene_hsv.s <= nGrayThreshold) && !sceneBlack;
	bool modelGrayImage = (model_hsv.s <= nGrayThreshold) && !modelBlack;
	bool result = false;
	if(sceneBlack || sceneGrayImage)
	{
		//both is black
		if(sceneBlack && modelBlack) return true;

		//both is gray color
		if(sceneGrayImage && modelGrayImage)
		{
			result = std::fabs(scene_hsv.v - model_hsv.v) < nValueThresh;
			return result;
		}
		//Scene should be gray/black, but is not. Or Model is not gray/black, but scene is. -> No correspondence
		return false;
	}

	//is the saturation difference to great? Might cause problems in light and dark scenes...
	if(std::fabs(scene_hsv.s - model_hsv.s) > 0.6) return false;

	//Get shortest angular distance
	//https://stackoverflow.com/questions/9505862/shortest-distance-between-two-degree-marks-on-a-circle
	double raw_diff = scene_hsv.h > model_hsv.h ? scene_hsv.h - model_hsv.h : model_hsv.h - scene_hsv.h;
	double mod_diff = std::fmod(raw_diff, 360.0);
	double dist = mod_diff > 180.0 ? 360.0 - mod_diff : mod_diff;

	result = std::fabs(dist) <= nAngleThresh;
	return result;
}
ColorMatching::hsv ColorMatching::rgb2hsv (rgb in)
{
	hsv out;
	double min, max, delta;

	min = in.r < in.g ? in.r : in.g;
	min = min < in.b ? min : in.b;

	max = in.r > in.g ? in.r : in.g;
	max = max > in.b ? max : in.b;

	out.v = max;                                // v
	delta = max - min;
	if (delta < 0.00001)
	{
		out.s = 0;
		out.h = 0; // undefined, maybe nan?
		return out;
	}
	if (max > 0.0)
	{ // NOTE: if Max is == 0, this divide would cause a crash
		out.s = (delta / max);                  // s
	}
	else
	{
		// if max is 0, then r = g = b = 0
		// s = 0, h is undefined
		out.s = 0.0;
		out.h = NAN;                            // its now undefined
		return out;
	}
	if (in.r >= max)                           // > is bogus, just keeps compilor happy
		out.h = (in.g - in.b) / delta;        // between yellow & magenta
	else if (in.g >= max)
		out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
	else
		out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

	out.h *= 60.0;                              // degrees

	if (out.h < 0.0)
		out.h += 360.0;

	return out;
}
ColorMatching::rgb ColorMatching::hsv2rgb (hsv in)
{
	double hh, p, q, t, ff;
	long i;
	rgb out;

	if (in.s <= 0.0)
	{       // < is bogus, just shuts up warnings
		out.r = in.v;
		out.g = in.v;
		out.b = in.v;
		return out;
	}
	hh = in.h;
	if (hh >= 360.0)
		hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = in.v * (1.0 - in.s);
	q = in.v * (1.0 - (in.s * ff));
	t = in.v * (1.0 - (in.s * (1.0 - ff)));

	switch (i)
	{
		case 0:
			out.r = in.v;
			out.g = t;
			out.b = p;
			break;
		case 1:
			out.r = q;
			out.g = in.v;
			out.b = p;
			break;
		case 2:
			out.r = p;
			out.g = in.v;
			out.b = t;
			break;

		case 3:
			out.r = p;
			out.g = q;
			out.b = in.v;
			break;
		case 4:
			out.r = t;
			out.g = p;
			out.b = in.v;
			break;
		case 5:
		default:
			out.r = in.v;
			out.g = p;
			out.b = q;
			break;
	}
	return out;
}
