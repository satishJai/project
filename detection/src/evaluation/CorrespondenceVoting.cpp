/*
 * CorrespondenceVoting.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: philipf
 */

#include "../../include/evaluation/CorrespondenceVoting.h"

CorrespondenceVoting::CorrespondenceVoting()
{
	correspondenceRatioThreshold = RosParameterManager::createParam<double> (0.15,ParamType::DOUBLE,0.0,1.0,"/CorrespondenceVoting/correspondenceRatioThreshold");
}
CorrespondenceVoting::~CorrespondenceVoting()
{
	RosParameterManager::deleteParam(correspondenceRatioThreshold);
}

bool CorrespondenceVoting::run(Segment& segment, Result& solution, std::vector<Model> &models)
{
	const std::vector<std::vector<Result>>& results = segment.results;
	//Vote for most instances
	int maxInstanceModelId = -1;
	int bestInstancesInstanceId = -1;
	int maxInstancesCount = 0;
	int bestInstanceCorrespondencesCount = 0;

	//Vote for best instance which has most correspondences
	int maxCorrespondencesModelId = -1;
	int maxCorrespondencesCorrId = -1;
	int maxCorrespondencesCount = 0;

	//check correspondences which have been removed because of color
	double totalCorrespondences = 0;

	/*
	//Vote for most unclustered correspondences
	int maxCorrespondencesModelId = -1;
	int maxCorrespondencesCorrId = -1;
	int mostUnclusteredCorrespondencesCount = 0;*/

	//Vote for highest correspondence average
	int maxAverageModelId = -1;
	int maxAverageCorrId = -1;
	float bestCorrespondenceAverage = 0.0;

	std::cout << std::endl << "----------------------- EVALUATING result ----------------------- " << std::endl;

	for(unsigned int modelId = 0; modelId < results.size(); modelId++)
	{
		const std::vector<Result>& modelResults = results.at(modelId);
		bool hasMostInstances = false;
		float correspondenceAverage = 0.0;
		int bestCorrespondenceCount = 0;
		int bestCorrespondenceId = -1;

		//How many correspondences have been accepted? If the color often did not match, this will be very low...
		if(segment.notFilteredCorresCountVec.at(modelId) != 0)
		{
			double correspondenceRatio = ((double)segment.model_scene_corrs_vec.at(modelId)->size()) / ((double)segment.notFilteredCorresCountVec.at(modelId));
			std::cout << "Model: " << models.at(modelId).name << " correspondenes: " << segment.model_scene_corrs_vec.at(modelId)->size() << " notFilteredCorrs: " << segment.notFilteredCorresCountVec.at(modelId) << std::endl;
			totalCorrespondences += correspondenceRatio;
			if(correspondenceRatio < correspondenceRatioThreshold->var)
			{
				std::cout << models.at(modelId).name << ": Correspondence Color/WithoutColor ratio too low, skipping!" << correspondenceRatio << " < " << correspondenceRatioThreshold->var << std::endl;
				continue;
			}
		}
		//If the segment is bigger than our model, this cant be right!
		if(sizeChecking->var)
		{
			if(segment.data.getDiameter() > (models.at(modelId).data.getDiameter())*1.35)
			{
				std::cout << models.at(modelId).name << " diameter is to small! S="
						<< segment.data.getDiameter() << " M= " << models.at(modelId).data.getDiameter() << std::endl;
				continue;
			}
		}

		//store which model has more instances
		if(modelResults.size() > (unsigned int)maxInstancesCount)
		{
			maxInstanceModelId = modelId;
			maxInstancesCount = modelResults.size();
			bestInstancesInstanceId = -1;
			bestInstanceCorrespondencesCount = 0;
			hasMostInstances = true;

		}

		//store which instance has the most correspondences
		for(unsigned int guessId = 0; guessId < modelResults.size(); guessId++)
		{
			if((unsigned int)bestCorrespondenceCount < modelResults.at(guessId).model_scene_corrs.size())
			{
				bestCorrespondenceCount = modelResults.at(guessId).model_scene_corrs.size();
				bestCorrespondenceId = guessId;
			}
			correspondenceAverage += ((float)modelResults.at(guessId).model_scene_corrs.size()) / ((float)modelResults.size()); //Might be a mistake. Might be results.size()
		}
		std::cout <<  models.at(modelId).name << "'s best instance " << bestCorrespondenceId << " has " << bestCorrespondenceCount << " correspondences!\n";

		//Has this model the instance with most correspondences?
		if(maxCorrespondencesCount < bestCorrespondenceCount)
		{
			maxCorrespondencesCount = bestCorrespondenceCount;
			maxCorrespondencesModelId = modelId;
			maxCorrespondencesCorrId = bestCorrespondenceId;
			std::cout << "Currently best model: "<< models.at(modelId).name << " has " << maxCorrespondencesCount << " correspondences." << std::endl;
		}

		//If this model has the most intances, store the number of correspondences from the best instance
		if(hasMostInstances)
		{
			bestInstanceCorrespondencesCount = bestCorrespondenceCount;
			bestInstancesInstanceId = bestCorrespondenceId;
			std::cout << models.at(modelId).name << " currently has the most instances: Best instance has: " << bestCorrespondenceCount << " correspondences!\n";
			std::cout << "Storing " << bestInstancesInstanceId << " as most corres." << std::endl;
		}

		//If this model has the best average correspondneces count, store this
		if(correspondenceAverage > bestCorrespondenceAverage)
		{
			maxAverageModelId = modelId;
			maxAverageCorrId = bestCorrespondenceId;
			bestCorrespondenceAverage = correspondenceAverage;
			std::cout << models.at(modelId).name << " has the best average correspondence count: " << bestCorrespondenceAverage << std::endl;
		}
	}

	if(maxInstanceModelId >= 0)
		std::cout << models.at(maxInstanceModelId).name << " has most instances: " << maxInstancesCount << ". Its best instance has " << bestInstanceCorrespondencesCount << " correspondences and is named: " << bestInstancesInstanceId << std::endl;
	if(maxCorrespondencesModelId >= 0)
		std::cout << models.at(maxCorrespondencesModelId).name << " has the most correspondences: " << maxCorrespondencesCount << " at instance: " << maxCorrespondencesCorrId << std::endl;
	if(maxAverageModelId >= 0)
		std::cout << models.at(maxAverageModelId).name << " has the most average amount of correspondences per instance: " << bestCorrespondenceAverage << std::endl;


	if(maxAverageModelId == -1 && maxCorrespondencesModelId == -1){
		std::cout << std::endl << "----------------------- No results. ----------------------- " << std::endl;
		std::cout << std::endl << "----------------------- Result has to be guessed! ----------------------- " << std::endl;
		solution.name = "UNKNOWN";
		return false;
	}

	std::cout << "Selecting pose:" << std::endl;
	//Select if maxCorrespondences or more guesses is better:
	if((bestCorrespondenceAverage) >= maxCorrespondencesCount)
	{
		solution.modelId = maxAverageModelId;
		solution.name = results.at(maxAverageModelId).at(0).name;

		double correspondenceRatio = ((double)results.at(maxAverageModelId).size()) / ((double)segment.notFilteredCorresCountVec.at(maxAverageModelId));
		solution.probability = correspondenceRatio / totalCorrespondences;
		//obj.probability = bestCorrespondenceAverage / ((float)totalCorrespondenceCount);
		solution.validPose = true;
		solution.pose = results.at(maxAverageModelId).at(maxAverageCorrId).pose;

		solution.rototranslations = results.at(maxAverageModelId).at(maxAverageCorrId).rototranslations;
		solution.model_scene_corrs = results.at(maxAverageModelId).at(maxAverageCorrId).model_scene_corrs;
	}else
	{
		solution.modelId = maxCorrespondencesModelId;
		solution.name = results.at(maxCorrespondencesModelId).at(0).name;
		double correspondenceRatio = ((double)results.at(maxCorrespondencesModelId).size()) / ((double)segment.notFilteredCorresCountVec.at(maxCorrespondencesModelId));
		solution.probability = correspondenceRatio / totalCorrespondences;
		solution.validPose = true;
		solution.pose = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).pose;
		solution.rototranslations = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).rototranslations;
		solution.model_scene_corrs = results.at(maxCorrespondencesModelId).at(maxCorrespondencesCorrId).model_scene_corrs;
	}
	std::cout << std::endl << "----------------------- done EVALUATING result ----------------------- " << std::endl;
	return true;
}


