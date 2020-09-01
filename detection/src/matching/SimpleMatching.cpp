/*
 * ColorMatching.cpp
 *
 *  Created on: Nov 16, 2018
 *      Author: philipf
 */
#include "../../include/matching/SimpleMatching.h"
#include <pcl/correspondence.h>

void SimpleMatching::run(Descriptors& descriptors, pcl::PointCloud<PointType>::ConstPtr keypoints, const std::vector<Model>* models,
		 std::vector<pcl::CorrespondencesPtr>* model_scene_corrs_vec, std::vector<unsigned long> *notFilteredCorresCountVec)
{
	model_scene_corrs_vec->clear();
	model_scene_corrs_vec->resize(models->size ());
	notFilteredCorresCountVec->clear();
	notFilteredCorresCountVec->resize(models->size());
	// Check every model.
	#pragma omp parallel for shared(models, descriptors, keypoints, model_scene_corrs_vec, notFilteredCorresCountVec)
	for (unsigned long j = 0; j < models->size (); j++)
	{
		std::cout << "Matching with: " << models->at(j).name << std::endl;
		std::cout << "There are " << descriptors.getSize() << " descriptors of type " << descriptors.datatype << std::endl;
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
			model_scene_corrs->push_back (corres);
		}
		std::cout << "Found " << notFilteredCorresCount << " correspondences! Found " << model_scene_corrs->size() << " color corres with "<< models->at(j).name << "!" << std::endl;
		notFilteredCorresCountVec->at(j) = notFilteredCorresCount;
		//All correspondences with the current model will be put into an array. E.g. if you access this array at position 0, you can get all correspondences with model0
		model_scene_corrs_vec->at(j) = model_scene_corrs;
	}
}
