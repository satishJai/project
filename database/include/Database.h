/*
 * database.h
 *
 *  Created on: Jan 22, 2019
 *      Author: satishj
 */

#ifndef SRC_DEXTERITY_DATABASE_INCLUDE_DATABASE_H_
#define SRC_DEXTERITY_DATABASE_INCLUDE_DATABASE_H_

#include <iostream>
#include <istream>
#include <ostream>
#include <string>
#include <exception>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <handling_msgs/SaveParameter.h>
#include <handling_msgs/AddModel.h>
#include <handling_msgs/GetModels.h>
#include <soci/soci.h>


class Database
{
	public:
		Database (const std::string& databasePath);
		/*Used for opening database connection*/
		void open (const std::string& backend, const std::string& param);

		bool createTable ();
		/*Saves new profile to the database.
		 * Takes profileName and the list of current available models from the ros parameter server
		 * Provides the location where the features(.pcd) files should to stored in the hard disk */
		bool saveProfile (std::string profileName, std::string modelNames, std::vector<std::string> &name, std::vector<std::string> &keypointFiles,
				std::vector<std::string> &descriptorFiles, std::vector<std::string> &normalFiles, std::vector<std::string> &referenceFrameFiles);
		/*Loads a specific profile from the database
		 * takes profile name as input
		 * @param profileName is the profile name
		 * @param yamlPath is the path to save, where database creates a .yaml file from the stored ros parameters
		 * @param models list of the models stored in the DB
		 * @return Returns true if success
		 */
		bool loadProfile (std::string profileName, std::string &yamlPath, std::string &models);
		/*Adding new model to DB*/
		bool addModel (const std::string name, const std::string location);
		/*Deleting a model from  DB*/
		bool deleteModel (const std::string name);
		/*Loads all the currently available models from the database*/
		bool getModels(std::vector<std::string> &modelName);
		/*To get the location where the models pcd files are stored*/
		bool getModelFile (std::vector<std::string> &modelName, std::vector<std::string> &modelFile);
		/*gets the list the profile from the DB*/
		bool getProfiles(std::vector<std::string> &profileName);
		/*Reads the features of the specific models from the DB
		 * @param profileName is the profile name
		 * @param models is the list of model
		 * @param modelFile is the location of the model pcd file
		 * @param keypoint,descriptor, normal, refFrame are the location where the features of a particular model is stored*/
		bool readFeatures(std::string profileName, std::vector<std::string> &models,std::vector<std::string> &modelFile, std::vector<std::string> &keypoint,
				std::vector<std::string> &descriptor, std::vector<std::string> &normal, std::vector<std::string> &refFrame);
		bool getActions(std::string actionType, std::string modelName, std::vector<std::string> &actionPoint, std::vector<std::string> &actionName);
		bool getObjectMesh(std::string actionType,std::vector<std::string> &modelName, std::vector<std::string> &meshPath);
		bool deleteProfile(std::string name);
	private:
		soci::session sql;
		std::string databasePath;
};

#endif /* SRC_DEXTERITY_DATABASE_INCLUDE_DATABASE_H_ */
