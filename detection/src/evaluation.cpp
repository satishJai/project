/*
 * main.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: philipf
 */

#include <iostream>
#include <stdlib.h>
#include <memory>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../include/Control.h"

struct GTObject
{
	tf::Transform t;
	std::string name;
	std::string filepath;
};

std::vector<std::string> listOfScenes;
std::vector<std::string> listOfGTs;
std::vector<std::string> listOfProfiles;

std::string localEvaluWorkspace = "default";

std::map<std::string, unsigned long> objectsCounter; //Total number of each object from all scenes.
unsigned long totalNumObjects = 0;					//Total number of all objects from all scenes
unsigned long totalNumDetectedObjects = 0;			//How many right detections did we have?
unsigned long totalNumberOfMistakes = 0;			//How many mistakes did we have?
unsigned long doubleClassifiedCounter = 0;			//How many did we classify an object twice? Happens if segmentation does make multiple segments out of one object.
double globalPositionError = 0;
double globalPositionError2D = 0;
double globalRotationError = 0;
double currentAverageRuntime = 0;

std::vector<std::string> allActiveModels;

std::map<std::string,int> mistakes;
std::map<std::string,int> badClassified;
std::map<std::string,int> correct;

std::vector<std::map<std::string, double>> precisionList;
std::vector<std::map<std::string, double>> recallList;
std::vector<std::map<std::string, double>> fOneList;

std::vector<double> globalPrecisionList;
std::vector<double> globalRecallList;
std::vector<double> globalFOneList;
std::vector<double> positionErrorList;
std::vector<double> positionError2dList;
std::vector<double> rotationErrorList;
std::vector<double> averageRuntime;

std::string bestProfileName = "none";
unsigned long mostDetectedProfile = 0;

double secondsBetweenScenes = 0.0;
ros::Time startTime;

bool loadClouds(std::string fileName, std::vector<GTObject>& objects)
{
	std::cout << "Loading ground truth file: " << fileName << std::endl;
	try{
		std::ifstream file(fileName);
		if(file.fail())
		{
			std::cout << "Failed to load file!\n";
			return false;
		}
		std::string text;
		tf::Vector3 loc(0,0,0);
		tf::Quaternion rot(0,0,0,0);
		tf::Transform t;
		GTObject currentObject;
		getline(file,text);
		while(text.size() > 0)
		{
			std::stringstream ss(text);
			int i = 0;
			while(ss.good())
			{
				std::string substr;
				getline(ss,substr,',');
				switch(i)
				{
					case 0:
					{
						double value = std::stof(substr.c_str());
						loc.setX(value); break;
					}
					case 1:
					{
						double value = std::stof(substr.c_str());
						loc.setY(value); break;
					}
					case 2:
					{
						double value = std::stof(substr.c_str());
						loc.setZ(value); break;
					}

					case 3:
					{
						double value = std::stof(substr.c_str());
						rot.setX(value); break;
					}
					case 4:
					{
						double value = std::stof(substr.c_str());
						rot.setY(value); break;
					}
					case 5:
					{
						double value = std::stof(substr.c_str());
						rot.setZ(value); break;
					}
					case 6:
					{
						double value = std::stof(substr.c_str());
						rot.setW(value); break;
					}
					case 7:
					{
						t.setOrigin(loc);
						t.setRotation(rot);
						currentObject.t = t;
						currentObject.filepath = substr;
						currentObject.name = boost::filesystem::basename(substr);
						objects.push_back(currentObject);
						break;
					}
					default:;
				}
				i++;
			}
			getline(file,text);
		}
		return true;
	}catch(std::ifstream::failure &readErr)
	{
		std::cerr << "\n\nException occured when reading a file\n"
				  << readErr.what()
				  << std::endl;
	}
	return false;
}
/*There might be multiple models of the same type, only generated with different parameters. If the string contains a +, this indicates that
 * everything bevore the + is the object name and everything after describes the different used parameters.*/
std::string getObjectBaseType(std::string name)
{
	std::size_t found = name.find("+");
	if(found != std::string::npos)
	{
		return name.substr(0,found);
	}
	return name;
}

void compareResults(std::vector<Result> results,std::vector<GTObject> gts, std::string sceneName, std::string path)
{
	std::vector<int> scenesAndGuessesCount;
	scenesAndGuessesCount.resize(gts.size());
	std::cout << "Results for " << sceneName << ":\n";
	for(Result r : results) //For all results, find nearest object inside ground truht file
	{
		std::string name = r.name;
		int best = -1;
		double smallestError = -1.0;
		double smallestError2D = -1.0;
		double rotError = -1.0;
		for(size_t i = 0; i < gts.size(); i++) //Find corresponding object in the ground truth file
		{
			double error = gts.at(i).t.getOrigin().distance(r.pose.getOrigin());
			if(best == -1 || error < smallestError)
			{
				smallestError = error;
				smallestError2D = std::sqrt(std::pow(gts.at(i).t.getOrigin().x() - r.pose.getOrigin().x(),2.0) + std::pow(gts.at(i).t.getOrigin().y() - r.pose.getOrigin().y(),2.0));
				best = i;
				rotError = gts.at(i).t.getRotation().angleShortestPath(r.pose.getRotation());
			}
		}
		if(best == -1)
		{
			std::cout << r.name << " is not in the scene!\n";
		}
		else
		{
			++scenesAndGuessesCount.at(best);
			std::string isRight;
			if(getObjectBaseType(r.name) == getObjectBaseType(gts.at(best).name))
			{
				isRight = "1";
				totalNumDetectedObjects++;
				globalPositionError += smallestError;
				globalPositionError2D += smallestError2D;
				globalRotationError += rotError;
				correct.at(r.name)++;
			}else{
				isRight = "-1";
				totalNumberOfMistakes++;
				mistakes.at(r.name)++;
				if(badClassified.find(getObjectBaseType(gts.at(best).name)) != badClassified.end())
				{
					badClassified.at(getObjectBaseType(gts.at(best).name))++;
				}else
				{
					badClassified.insert(std::pair<std::string,int>(getObjectBaseType(gts.at(best).name),1));
				}
			}
			std::cout << r.name << " -> " << getObjectBaseType(gts.at(best).name) << " eP: " << smallestError << " rP: " << rotError << "\n";
		}
		//Remember seen object. Maybe we classify the same object multiple times...
	}
	//Check if some objcts have been classified multiple times. then segmentation is not working properly.
	for(int c : scenesAndGuessesCount)
	{
		if(c > 1)
		{
			doubleClassifiedCounter++;
		}
	}
	for(GTObject o : gts)
	{
		if(o.name != "default")
		{
			totalNumObjects++;
			if(objectsCounter.count(getObjectBaseType(o.name)) == 1)//Count how many times our learned models could be actually detected
			{
				objectsCounter.at(getObjectBaseType(o.name))++;
			}
		}
	}
}

void printStatistics(std::string profilePath)
{
	averageRuntime.push_back(currentAverageRuntime / double(listOfScenes.size()));

	/*Precision, Recall, F1 per object!*/
	std::map<std::string,double> precisions;
	std::map<std::string,double> recalls;
	std::map<std::string,double> fOnes;
	for(std::string m : allActiveModels)
	{
		std::pair<std::string,double> pair;
		pair.first = m;
		double p = double(correct.at(m)) / double((mistakes.at(m) + correct.at(m)));
		pair.second = p;
		precisions.insert(pair);

		double r = double(correct.at(m)) / objectsCounter.at(m);
		pair.second = r;
		recalls.insert(pair);

		pair.second = 2 * (p*r) / (p+r);
		fOnes.insert(pair);
	}
	precisionList.push_back(precisions);
	recallList.push_back(recalls);
	fOneList.push_back(fOnes);

	/*Global precision, recall, F1.*/
	double recall = double(totalNumDetectedObjects) / double(totalNumObjects);
	globalRecallList.push_back(recall);
	double precision = double(totalNumDetectedObjects) / double(totalNumberOfMistakes + totalNumDetectedObjects);
	globalPrecisionList.push_back(precision);
	double fOneScore = 2 * (precision * recall) / (precision + recall);
	globalFOneList.push_back(fOneScore);

	positionErrorList.push_back(globalPositionError/double(totalNumDetectedObjects));
	positionError2dList.push_back(globalPositionError2D/double(totalNumDetectedObjects));
	rotationErrorList.push_back(globalRotationError/double(totalNumDetectedObjects));


	std::cout << "DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_\n";
	std::cout << "HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_HAPPY_\n";
	std::cout << "DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_DONE_\n\n\n";
	for(std::string m : allActiveModels)
	{
		std::cout << "M="<< m << " p=" << precisions.at(m) << " r=" << recalls.at(m) << " f1=" << fOnes.at(m) << std::endl;
	}
	std::cout << "\nTime per cloud: " << currentAverageRuntime / double(listOfScenes.size()) << "s" << std::endl;
	std::cout << "Detected " << totalNumDetectedObjects << " out of " << totalNumObjects << std::endl;
	std::cout << "But did " << totalNumberOfMistakes << " mistakes.\n";
	std::cout << "Double classificated objects: " << doubleClassifiedCounter << std::endl;
	std::cout << "Precision: " << precision << " recall: " << recall << " F1: " << fOneScore << std::endl;
	std::cout << "Average position3d error: " << globalPositionError/double(totalNumDetectedObjects)
			  << " position2d error: " << globalPositionError2D/double(totalNumDetectedObjects)
			  << " rotation error: " << globalRotationError/double(totalNumDetectedObjects) << std::endl;

	std::string dir = boost::filesystem::path(profilePath).parent_path().string();
	std::string name = boost::filesystem::basename(profilePath);

	std::cout << "\n\nBad Guesses:\n";
	for(auto const& x : mistakes)
	{
		std::cout << x.first << ": " << x.second << std::endl;
	}
	std::cout << "\n\nBad Classified:\n";
	for(auto const& x : badClassified)
	{
		std::cout << x.first << ": " << x.second << std::endl;
	}
	std::cout << "\n\nCorrect Classified:\n";
	for(auto const& x : correct)
	{
		std::cout << x.first << ": " << x.second << std::endl;
	}

	std::ofstream file;
	try
	{
		file.open(dir + std::string("/") + name + std::string(".result"));
		file << "Time per cloud: " << currentAverageRuntime / double(listOfScenes.size()) << "s" << std::endl;
		file << "Detected " << totalNumDetectedObjects << " out of " << totalNumObjects << std::endl;
		file << "But did " << totalNumberOfMistakes << " mistakes.\n";
		file << "Double classificated objects: " << doubleClassifiedCounter << std::endl;
		file << "Precision: " << precision << " recall: " << recall << " F1: " << fOneScore << std::endl;
		file << "Average position3d error: " << globalPositionError/double(totalNumDetectedObjects)
					  << " position2d error: " << globalPositionError2D/double(totalNumDetectedObjects)
					  << " rotation error: " << globalRotationError/double(totalNumDetectedObjects) << std::endl;
		file << "\n";
		for(std::string m : allActiveModels)
		{
			file << "M="<< m << " p=" << precisions.at(m) << " r=" << recalls.at(m) << " f1=" << fOnes.at(m) << std::endl;
		}
		file << "\nBad Guesses:\n";
		for(auto const& x : mistakes)
		{
			file << x.first << ": " << x.second << std::endl;
		}
		file << "\n\nBad Classified:\n";
		for(auto const& x : badClassified)
		{
			file << x.first << ": " << x.second << std::endl;
		}
		file << "\n\nCorrect Classified:\n";
		for(auto const& x : correct)
		{
			file << x.first << ": " << x.second << std::endl;
		}
		file.close();
	}catch(std::ofstream::failure &writeErr)
	{
		std::cerr << "\n\nException occured when writing to a file\n"
		          << writeErr.what()
		          << std::endl;
	}
}

void printFinalResult(std::string dir, std::string name)
{
	std::ofstream file;
	try
	{
		std::cout << "Creating Final Result: " << dir + "/" + name + ".csv" << std::endl;
		file.open(dir + std::string("/") + name + std::string(".csv"));
		/*Write header for .csv file!*/
		file << "ProfileId, Precision, Recall, FOne, Runtime, PositionError, PositionError2d, RotationError";
		for(std::string m : allActiveModels)
		{
			file << ", " << m << "_F1";
		}
		for(std::string m : allActiveModels)
		{
			file << ", " << m << "_R";
		}
		for(std::string m : allActiveModels)
		{
			file << ", " << m << "_P";
		}
		file << std::endl;

		/*Write data for .csv file!*/
		for(size_t i = 0; i < globalPrecisionList.size(); i++)
		{
			file << boost::filesystem::basename(listOfProfiles.at(i)) << ", "
					<< globalPrecisionList.at(i) << ", " << globalRecallList.at(i) << ", " << globalFOneList.at(i) << ", " << averageRuntime.at(i)
					<< ", " << positionErrorList.at(i) << ", " << positionError2dList.at(i) << ", " << rotationErrorList.at(i);
			for(std::string m : allActiveModels)
			{
				file << ", " << fOneList.at(i).at(m);
			}
			for(std::string m : allActiveModels)
			{
				file << ", " << recallList.at(i).at(m);
			}
			for(std::string m : allActiveModels)
			{
				file << ", " << precisionList.at(i).at(m);
			}
			file << std::endl;
		}
		file.close();
	}catch(std::ofstream::failure &writeErr)
	{
		std::cerr << "\n\nException occured when writing to a file\n"
				  << writeErr.what()
				  << std::endl;
	}
}

int main (int argc, char* argv[])
{
	bool allowVisualization = false;
	if(argc > 1)
	{
		localEvaluWorkspace = argv[1];
		if(argc > 2)
		{
			std::string visualization(argv[2]);
			if(visualization == "true")
			{
				allowVisualization = true;
			}
			if(argc > 3)
			{
				secondsBetweenScenes = std::atof(argv[3]);
			}
		}
	}else
	{
		std::cout << "Usage: evaluation WORKSPACE [ALLOW_VISUALIZATION=false] [SECONDS_BETWEEN_SCENES=0.0]\n";
		return 0;
	}
	std::cout << "Using evalu workspace: " << localEvaluWorkspace << std::endl;
	std::cout << "Visualization = " << allowVisualization << " secondsBeweenScenes= " << secondsBetweenScenes << std::endl;
	ros::init (argc, argv, localEvaluWorkspace + "Evaluation");

	ros::NodeHandle nh;
	nh.setParam("/detection/visualization/allowVisualization",allowVisualization);
	Control detectionControl(allowVisualization,false,true);
	std::string packagePath = ros::package::getPath("handling_recognition");
	size_t id = 0;

	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(packagePath + "/evalu/" + localEvaluWorkspace),{}))
	{
		std::string file = entry.path().string();
		std::string extension = boost::filesystem::extension(file);
		if(extension != ".txt") continue;
		listOfProfiles.push_back(file);
	}

	std::sort(listOfProfiles.begin(),listOfProfiles.end());

	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(packagePath + "/objects/models"),{}))
	{
		std::string path = entry.path().string();
		std::string extension = boost::filesystem::extension(path);
		if(extension != ".pcd") continue;
		std::string name = boost::filesystem::basename(path);
		detectionControl.learnModel(path,name,id);
		mistakes.insert(std::pair<std::string,int>(name,0));
		correct.insert(std::pair<std::string,int>(name,0));
		objectsCounter.insert(std::pair<std::string,unsigned long>(name,0));
		allActiveModels.push_back(name);
		id++;
	}

	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(packagePath + "/objects/scenes"),{}))
	{
		std::string path = entry.path().string();
		std::string extension = boost::filesystem::extension(path);
		if(extension != ".pcd") continue;
		std::string gtName = path.substr(0,path.size()-4) + ".gt";
		if(!boost::filesystem::exists(gtName))
		{
			std::cout << gtName << " does not exist. Skipping..\n";
			continue;
		}

		listOfScenes.push_back(path);
		listOfGTs.push_back(path.substr(0,path.size()-4) + ".gt");
		std::cout << "ListOfScene: " << path << " listOfGT: " << path.substr(0,path.size()-4) << ".gt" << std::endl;
	}

	ros::Duration(0.5).sleep();
	startTime = ros::Time::now();
	double remainingTime = -60.0;
	std::cout << "Starting loop." << std::endl;
	for(size_t p = 0; ((p < listOfProfiles.size()) && ros::ok()); p++)
	{
		std::cout << "\n\nLoading profile: " << listOfProfiles.at(p) << std::endl;
		std::string cmd = "rosparam load " + listOfProfiles.at(p);
		std::cout << cmd << std::endl;
		system(cmd.c_str());
		std_srvs::Empty::Request req;
		std_srvs::Empty::Response res;
		detectionControl.updateParamCB(req,res);
		detectionControl.checkForRetrain(true);
		ros::Time profileStartTime = ros::Time::now();
		for(size_t i = 0; ((i < listOfScenes.size()) && ros::ok()); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr newScene = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			std::cout << "Loading scene: " << listOfScenes.at(i) << std::endl;
			if (pcl::io::loadPCDFile (listOfScenes.at(i), *newScene) < 0)
			{
				ROS_ERROR("[evaluation]: Error loading scene '%s'",listOfScenes.at(i).c_str());
			}
			else
			{
				std::vector<Result> results;
				std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
				std::cout << "++++++++++++++++++" << " Profile " << (p+1) << " of " << (unsigned long)listOfProfiles.size() << " +++++++++++++++++++++++++++++++++++++++++++++++\n";
				std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
				ros::Time start = ros::Time::now();
				detectionControl.recognize (*newScene, results);
				currentAverageRuntime += (ros::Time::now()-start).toSec();
				double minutesToEnd = (remainingTime-(ros::Time::now() - profileStartTime).toSec())/60.0; //Estimate runtime:
				std::string output = (minutesToEnd < 0)? "-1.0min" : std::to_string((int)minutesToEnd) + "min" + std::to_string(int((minutesToEnd - int(minutesToEnd))*60.0)) + "s";
				std::cout << "----------------------------------------------------------------------------------------------------------------------------------\n";
				std::cout << "-------------------------------------- Remaining time: " << output << " ---------------------------------------\n";
				std::cout << "----------------------------------------------------------------------------------------------------------------------------------\n";
				std::cout << "[evaluation]: Found " << results.size () << " matches!\n";
				std::vector<GTObject> gtList;
				loadClouds(listOfGTs.at(i),gtList);
				compareResults(results,gtList,boost::filesystem::basename(listOfProfiles.at(p)) + "/" + boost::filesystem::basename(listOfScenes.at(i)),
						packagePath + "/evalu/" + localEvaluWorkspace);
			}
			if(allowVisualization)
			{
				std::cout << "Start visualization!\n";
				ros::Time start = ros::Time::now();
				while((ros::Time::now() - start).toSec() < secondsBetweenScenes)
				{
					detectionControl.spinViewer ();
					ros::spinOnce();
				}
				std::cout << "Continue Detection!\n";
			}
		}
		printStatistics(listOfProfiles.at(p));
		if(mostDetectedProfile < totalNumDetectedObjects)
		{
			mostDetectedProfile = totalNumDetectedObjects;
			bestProfileName = boost::filesystem::basename(listOfProfiles.at(p));
		}

		//Reset variables
		totalNumObjects = 0;
		totalNumDetectedObjects = 0;
		totalNumberOfMistakes = 0;
		globalPositionError = 0;
		globalPositionError2D = 0;
		globalRotationError = 0;
		doubleClassifiedCounter = 0;
		currentAverageRuntime = 0;
		for(auto const& x : mistakes)
		{
			mistakes.at(x.first) = 0;
		}
		for(auto const& x : correct)
		{
			correct.at(x.first) = 0;
		}
		for(auto const& x: badClassified)
		{
			badClassified.at(x.first) = 0;
		}
		for(auto const& x: objectsCounter)
		{
			objectsCounter.at(x.first) = 0;
		}

		remainingTime = ((ros::Time::now() - startTime).toSec() / (p + 1)) * (listOfProfiles.size() - (p + 1));

	}
	printFinalResult(packagePath + "/evalu/" + localEvaluWorkspace,"result");
	std::cout << "Best Profile with " << mostDetectedProfile << " detected objects: " << bestProfileName << std::endl;

	//Print best FOne score!
	double bestFOne = 0.0;
	int bestFOneId = 0;
	for(size_t j = 0; j < globalFOneList.size(); j++)
	{
		if(globalFOneList.at(j) > bestFOne)
		{
			bestFOne = globalFOneList.at(j);
			bestFOneId = j;
		}
	}
	std::cout << "Best Profile F1 with " << bestFOne << " detected objects: " << boost::filesystem::basename(listOfProfiles.at(bestFOneId)) << std::endl;
	std::cout << "Done! Press CTL + C to quit!\n";
	while(ros::ok())
	{
		ros::Duration(0.5).sleep();
		detectionControl.spinViewer ();
		ros::spinOnce();
	}
	return 0;
}

