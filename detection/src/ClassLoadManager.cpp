/*
 * ClassLoadManager.cpp
 *
 *  Created on: Feb 12, 2019
 *      Author: satishj
 */
#include <ClassLoadManager.h>

#include <descriptor/CShotCreator.h>
#include <descriptor/ShotCreator.h>
#include <descriptor/RSDCreator.h>
#include <descriptor/FPFHCreator.h>
#include <descriptor/ShapeContextCreator.h>
#include <descriptor/UShapeContextCreator.h>
#include <descriptor/RSDCreator.h>
#include <keypoint/ISSCreator.h>
#include <keypoint/NDCreator.h>
#include <keypoint/VoxelGridCreator.h>
#include <matching/ColorMatching.h>
#include <matching/KColorMatching.h>
#include <matching/KSColorMatching.h>
#include <matching/SimpleMatching.h>
#include <pose_estimator/GCClustering.h>
#include <pose_estimator/HoughClustering.h>
#include <evaluation/CorrespondenceVoting.h>
#include <evaluation/CorrespondenceVotingICP.h>

const std::map<std::string,std::shared_ptr<DescriptorCreator>> ClassLoadManager::descriptorCreators =
{
		{"CShotCreator",std::make_shared<CShotCreator>()},
		{"RSDCreator",std::make_shared<RSDCreator>()},
		{"ShotCreator",std::make_shared<ShotCreator>()},
		{"FPFHCreator",std::make_shared<FPFHCreator>()},
		{"ShapeContextCreator",std::make_shared<ShapeContextCreator>()},
		{"UShapeContextCreator",std::make_shared<UShapeContextCreator>()}
};

const std::map<std::string,std::shared_ptr<KeypointCreator>> ClassLoadManager::keypointCreators =
{
		{"ISSCreator",std::make_shared<ISSCreator>()},
		{"NDCreator",std::make_shared<NDCreator>()},
		{"VoxelGridCreator",std::make_shared<VoxelGridCreator>()}
};

const std::map<std::string,std::shared_ptr<Matching>> ClassLoadManager::matchings =
{
		{"ColorMatching",std::make_shared<ColorMatching>()},
		{"KColorMatching",std::make_shared<KColorMatching>()},
		{"KSColorMatching",std::make_shared<KSColorMatching>()},
		{"SimpleMatching",std::make_shared<SimpleMatching>()}
};

const std::map<std::string,std::shared_ptr<PoseEstimator>> ClassLoadManager::poseEstimators =
{
		{"GCClustering",std::make_shared<GCClustering>()},
		{"HoughClustering",std::make_shared<HoughClustering>()}
};

const std::map<std::string,std::shared_ptr<ResultEvaluation>> ClassLoadManager::resultEvaluations =
{
		{"CorrespondenceVoting",std::make_shared<CorrespondenceVoting>()},
		{"CorrespondenceVotingICP",std::make_shared<CorrespondenceVotingICP>()}
};

std::shared_ptr<DescriptorCreator> ClassLoadManager::getDescriptorCreator(std::string s)
{
	if (descriptorCreators.count (s) > 0)
	{
		std::cout << "[ClassLoadManager]: Loading " << s << std::endl;
		return descriptorCreators.at(s);
	}
	std::cout << "[ClassLoadManager]: Unable to find " << s << "!\n";
	return nullptr;
}

std::shared_ptr<KeypointCreator> ClassLoadManager::getKeypointCreator(std::string s)
{
	if (keypointCreators.count (s) > 0)
	{
		std::cout << "[ClassLoadManager]: Loading " << s << std::endl;
		return keypointCreators.at(s);
	}
	std::cout << "[ClassLoadManager]: Unable to find " << s << "!\n";
	return nullptr;
}

std::shared_ptr<Matching> ClassLoadManager::getMatching(std::string s)
{
	if (matchings.count (s) > 0)
	{
		std::cout << "[ClassLoadManager]: Loading " << s << std::endl;
		return matchings.at(s);
	}
	std::cout << "[ClassLoadManager]: Unable to find " << s << "!\n";
	return nullptr;
}

std::shared_ptr<PoseEstimator> ClassLoadManager::getPoseEstimator(std::string s)
{
	if (poseEstimators.count (s) > 0)
	{
		std::cout << "[ClassLoadManager]: Loading " << s << std::endl;
		return poseEstimators.at(s);
	}
	std::cout << "[ClassLoadManager]: Unable to find " << s << "!\n";
	return nullptr;
}

std::shared_ptr<ResultEvaluation> ClassLoadManager::getResultEvaluation(std::string s)
{
	if (resultEvaluations.count (s) > 0)
	{
		std::cout << "[ClassLoadManager]: Loading " << s << std::endl;
		return resultEvaluations.at(s);
	}
	std::cout << "[ClassLoadManager]: Unable to find " << s << "!\n";
	return nullptr;
}

