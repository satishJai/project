/*
 * database.cpp
 *
 *  Created on: Jan 22, 2019
 *      Author: satishj
 */

#include <Database.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

using namespace soci;
using namespace std;

Database::Database (const std::string& databasePath)
		: databasePath (databasePath)
{
}

void Database::open (const std::string& backend, const std::string& param)
{
	try
	{
		sql.open (backend, param);
		std::cout << "Connection established successfully. \n";
	}
	catch (const std::exception& e)
	{
		std::cerr << "error opening database: " << e.what () << '\n';
	}
}

bool Database::createTable ()
{
	try
	{
		sql << "create table profile ("
				" id serial unique,"
				" name varchar(100) PRIMARY KEY,"
				" parameters text,"
				" date date NOT NULL DEFAULT CURRENT_DATE"
				") ";

		sql << "create table feature ("
				" featureId serial PRIMARY KEY,"
				" modelName varchar(100) ,"
				" descriptor text ,"
				" keypoint text ,"
				" normal text ,"
				" refFrame text ,"
				" profileId INTEGER REFERENCES profile(id)"
				") ";

		sql << "create table Models ("
				" id serial ,"
				" name varchar(100) PRIMARY KEY,"
				" location text "
				") ";

	}

	catch (soci_error const &e1)
	{
		cerr << "table already exist " << e1.what () << '\n';

	}
	return true;
}

bool Database::saveProfile (std::string profileName, std::string modelNames, std::vector<std::string> &model, std::vector<std::string> &keypointFiles, std::vector<std::string> &descriptorFiles, std::vector<std::string> &normalFiles, std::vector<
		std::string> &referenceFrameFiles)
{
	int count = 0;
	sql << "select count(*) from profile where name = :profileName", use (profileName), into (count);
	if (count != 0)
	{
		deleteProfile (profileName);
	}
	//Reading the file from the hard disk
	std::ifstream file (std::string (databasePath + "/dump.yaml").c_str ());
	std::string parameterValue;
	std::string value;
	if (file.fail ())
	{
		std::cout << "[database]: file '" << databasePath + "/dump.yaml" << "' does not exist." << std::endl;
	}
	else
	{
		while (file.good ())
		{
			getline (file, value);
			if (value.length () > 0)
			{
				parameterValue.append (value);
				parameterValue.append ("\n");
			}
		}
	}

	try
	{
		sql << "insert into profile (name, parameters, modelNames) values(:profileName, :value, :modelNames)", use (profileName), use (parameterValue), use (modelNames);
		int profileId;
		sql << "select id from profile where name = :profileName", use (profileName), into (profileId);
		//creating a string vector'model' from string modelNames
		std::string word = "";
		char dl = ',';
		modelNames = modelNames + dl;
		model.clear ();
		for (std::size_t i = 0; i < modelNames.size (); i++)
		{

			if (modelNames[i] != dl)
			{
				word = word + modelNames[i];
			}
			else
			{
				if (word.size () != 0)
					model.push_back (word);
				word = "";
			}
		}
		//Creating the feature storage path for each model
		for (auto i : model)
		{

			std::string descriptor = '/' + profileName + '/' + i + '/' + "descriptor.pcd";
			std::string keypoint = '/' + profileName + '/' + i + '/' + "keypoint.pcd";
			std::string normal = '/' + profileName + '/' + i + '/' + "normal.pcd";
			std::string refFrame = '/' + profileName + '/' + i + '/' + "refFrame.pcd";
			keypointFiles.push_back (databasePath + keypoint);
			descriptorFiles.push_back (databasePath + descriptor);
			normalFiles.push_back (databasePath + normal);
			referenceFrameFiles.push_back (databasePath + refFrame);
			sql << "insert into feature (modelName, descriptor, keypoint, normal, refFrame, profileid ) values(:i, :descriptor, :keypoint, :normal, :refFrame, :profileId )", use (i), use (descriptor), use (keypoint), use (normal), use (refFrame), use (profileId);
		}
		return true;
	}
	catch (soci_error const &e)
	{
		std::cerr << "Exception: " << e.what () << '\n';
		return false;

	}
	return true;
}

bool Database::loadProfile (std::string profileName, std::string &yamlPath, std::string &models)
{
	try
	{
		std::string parameterValue = "";
		std::string modelList = "";
		int profileId = 0;
		/*storing all the parameters from the profile table in outfile.yaml*/
		sql << "select parameters from profile where name = :profileName", use (profileName), into (parameterValue);
		std::remove ((databasePath + "/outfile.yaml").c_str ());
		ofstream outfile;
		outfile.open ((databasePath + "/outfile.yaml").c_str ());
		yamlPath = (databasePath + "/outfile.yaml").c_str ();
		outfile << parameterValue;
		outfile.close ();
		/*creating the model list for a particular profileID using rowset vector*/
		sql << "select id from profile where name = :profileName", use (profileName), into (profileId);
		rowset<row> rs = (sql.prepare << "select modelName from feature where profileId = :profileId", use (profileId));
		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			modelList.append (row.get<string> (0));
			modelList.append (",");
		}
		size_t pos = modelList.find_last_of (",");
		models = modelList.substr (0, pos);
		std::cout << "The models are: " << models << "\n" << std::endl;
		return true;
	}
	catch (soci_error const &e)
	{
		std::cerr << "Exception: " << e.what () << '\n';
		return false;

	}

}

bool Database::readFeatures (std::string profileName, std::vector<std::string> &models, std::vector<std::string> &modelFile, std::vector<std::string> &keypoint, std::vector<std::string> &descriptor, std::vector<std::string> &normal, std::vector<
		std::string> &refFrame)
{
	int id = 0;
	std::string modellocation;
	sql << "select id from profile where name = :profileName", use (profileName), into (id);
	if (profileName.empty ())
	{
		for (auto i : models)
		{
			sql << "select location from Models where name = :i", use (i), into (modellocation);
			modelFile.push_back (databasePath + modellocation);
			descriptor.push_back ("");
			keypoint.push_back ("");
			normal.push_back ("");
			refFrame.push_back ("");
		}
	}
	else
	{
		for (auto i : models)
		{
			sql << "select location from Models where name = :i", use (i), into (modellocation);
			modelFile.push_back (databasePath + modellocation);
		}
		rowset<row> rs = (sql.prepare << "select modelName, descriptor, keypoint, normal, refFrame from feature where profileId = :id", use (id));
		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			models.push_back (row.get<string> (0));
			descriptor.push_back (databasePath + row.get<string> (1));
			keypoint.push_back (databasePath + row.get<string> (2));
			normal.push_back (databasePath + row.get<string> (3));
			refFrame.push_back (databasePath + row.get<string> (4));
		}
	}

	return true;
}

bool Database::addModel (const std::string name, const std::string location)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (location, cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}
	std::string storeLocation = '/' + name + ".pcd";
	if (pcl::io::savePCDFile ((databasePath + storeLocation), cloud) < 0)
	{
		std::cerr << "Adding model to database failed! File does not exist." << std::endl;
		return false;
	}
	std::cout << "Saved " << cloud.points.size () << " data points" << std::endl;

	try
	{
		std::cout << "Connection established successfully. \n";
		std::cout << "The database path is: " << databasePath << std::endl;
		sql << "insert into Models (name, location) values(:name, :storeLocation)", use (name), use (storeLocation);

	}
	catch (exception const &e1)
	{
		std::cout << "Connection to database failed " << e1.what () << '\n';
		return false;
	}

	return true;
}

bool Database::getModels (std::vector<std::string> &modelName)
{
	try
	{
		std::cout << "Connection established successfully. \n";
		int count;
		sql << "select count(*) from Models", into (count);
		std::cout << "We have " << count << " entries in the models table.\n";
		rowset<row> rs = (sql.prepare << "select name from models");

		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			modelName.push_back (row.get<string> (0));
			std::cout << row.get<string> (0) << "\n" << std::endl;
		}
		return true;
	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;
	}
}

bool Database::getModelFile (std::vector<std::string> &modelName, std::vector<std::string> &modelFile)
{
	try
	{
		std::cout << "Connection established successfully. \n";
		std::string loaction;
		for (auto model : modelName)
		{
			sql << "select location from Models where name = :model", use (model), into (loaction);
			modelFile.push_back (databasePath + loaction);
		}

		return true;
	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;
	}
}

bool Database::getProfiles (std::vector<std::string> &profileName)
{
	try
	{
		std::cout << "Connection established successfully. \n";
		int count;
		sql << "select count(*) from profile", into (count);
		std::cout << "We have " << count << " entries in the profile table.\n";
		rowset<row> rs = (sql.prepare << "select name from profile");

		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			profileName.push_back (row.get<string> (0));
			std::cout << row.get<string> (0) << "\n" << std::endl;
		}
		return true;
	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;
	}
}

bool Database::deleteModel (const std::string modelName)
{
	try
	{
		std::string modelLocation = "";
		sql << "select location from Models where name = :modelName", use (modelName), into (modelLocation);

		if (std::remove ((databasePath + modelLocation).c_str ()) != 0)
		{
			cout << "Error while deleting file at location" << modelLocation << '\n';
		}

		else
		{
			sql << "delete from Models where name = :modelName", use (modelName);
			std::cout << "The model is deleted from: " << databasePath + modelLocation << '\n';
			sql << "delete from feature where modelName = :modelName", use (modelName);
		}
		rowset<row> rs = (sql.prepare << "select descriptor, keypoint, normal, refFrame from feature where modelName = :modelName", use (modelName));
		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			std::remove ((databasePath + row.get<string> (0)).c_str ());
			std::remove ((databasePath + row.get<string> (1)).c_str ());
			std::remove ((databasePath + row.get<string> (2)).c_str ());
			std::remove ((databasePath + row.get<string> (3)).c_str ());
		}
	}
	catch (soci_error const &e)
	{
		std::cerr << "Exception: " << e.what () << '\n';
		return false;

	}
	return true;
}

bool Database::getActions (std::string actionType, std::string modelName, std::vector<std::string> &actionPoint, std::vector<std::string> &actionName)
{
	rowset<row> rs = (sql.prepare << "select actionPoint, actionName from actions where type = :actionType and model =:modelName", use (actionType), use (modelName));

	for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
	{
		row const& row = *it;
		actionPoint.push_back (row.get<string> (0));
		actionName.push_back (row.get<string> (1));
	}
	return true;
}
bool Database::getObjectMesh (std::string actionType, std::vector<std::string> &modelName, std::vector<std::string> &meshPath)
{
	rowset<row> rs = (sql.prepare << "select distinct model from actions where type = :actionType", use (actionType));

	for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
	{
		row const& row = *it;
		modelName.push_back (row.get<string> (0));
	}
	std::string path = "";
	for(auto i : modelName)
	{
		sql << "select meshpath from Models where name = :modelName", use (i), into (path);
		meshPath.push_back(path);
	}
	return true;
}
bool Database::deleteProfile (std::string name)
{
	int profileId = 0;
	sql << "select id from profile where name = :name", use (name), into (profileId);
	sql << "delete from profile where name = :name", use (name);
	sql << "delete from feature where profileId = :profileId", use (profileId);
	std::remove ((databasePath + '/' + name).c_str ());
	return true;
}
