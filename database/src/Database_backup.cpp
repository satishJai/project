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
using namespace soci::details;
using namespace std;

Database::Database (const std::string& databasePath)
		: databasePath (databasePath)
{
}

bool Database::createTable ()
{
	try
	{
		std::cout << "Connection established successfully. \n";
		try
		{
			//sql << "drop table profile";
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
	catch (exception const &e)
	{
		std::cerr << "Connection to database failed " << e.what () << '\n';
		return false;

	}

}

/*bool Database::saveProfile_backup (std::string profileName, std::string comment, Features feature)
 {
 //Reading the file from the harddisk
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
 std::cout << "The parameters are " << parameterValue << std::endl;
 }
 //Connecting to the database
 try
 {
 std::cout << "Connection established successfully. \n";
 //Instering parameter in the Profile table
 try
 {
 std::string descriptor = databasePath + '/' + profileName + '/' + "descriptor.pcd";
 std::string keypoint = databasePath + '/' + profileName + '/' + "keypoint.pcd";
 std::string normal =  + '/' + profileName + '/' + "normal.pcd";
 std::string refFrame = databasePath + '/' + profileName + '/' + "refFrame.pcd";

 sql << "insert into profile (name, parameter) values(:profileName, :value)", use (profileName), use (parameterValue);
 //sql << "insert into profile (name, parameter) values(\'detection/keypoint/ramjia\', :value)", use(profileName),use (parameterValue);
 int profileId;
 sql << "select id from profile where name = :profileName", use (profileName), into (profileId);
 sql << "insert into feature (id, descriptor, keypoint, normal, refFrame ) values(:profileId, :descriptor, :keypoint, :normal, :refFrame )", use (profileId), use (descriptor), use (keypoint), use (normal), use (refFrame);
 return true;
 }
 catch (soci_error const &e)
 {
 std::cerr << "Exception: " << e.what () << '\n';
 return false;

 }
 }
 catch (exception const &e1)
 {
 std::cerr << "Connection to database failed " << e1.what () << '\n';
 return false;

 }

 }*/

bool Database::saveProfile (std::string profileName, std::string modelNames, std::vector<std::string> &model, std::vector<std::string> &keypointFiles, std::vector<std::string> &descriptorFiles, std::vector<std::string> &normalFiles, std::vector<
		std::string> &referenceFrameFiles)
{

	//Reading the file from the harddisk
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
		std::cout << "The parameters are " << parameterValue << std::endl;

	}
	//Connecting to the database
	try
	{
		std::cout << "Connection established successfully. \n";
		//Instering parameter in the Profile table
		try
		{
			sql << "insert into profile (name, parameters, modelNames) values(:profileName, :value, :modelNames)", use (profileName), use (parameterValue), use (modelNames);
			int profileId;
			sql << "select id from profile where name = :profileName", use (profileName), into (profileId);
			//creating a string vector for models
			std::string word = "";
			char dl = ',';
			modelNames = modelNames + dl;
			//std::vector<std::string> model;
			model.clear();
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

			for (auto i : model)
			{
				std::string descriptor = databasePath + '/' + profileName + '/' + i + '/' + "descriptor.pcd";
				std::string keypoint = databasePath + '/' + profileName + '/' + i + '/' + "keypoint.pcd";
				std::string normal = databasePath + '/' + profileName + '/' + i + '/' + "normal.pcd";
				std::string refFrame = databasePath + '/' + profileName + '/' + i + '/' + "refFrame.pcd";
				keypointFiles.push_back (keypoint);
				descriptorFiles.push_back (descriptor);
				normalFiles.push_back (normal);
				referenceFrameFiles.push_back (refFrame);
				//sql << "insert into profile (name, parameter) values(\'detection/keypoint/ramjia\', :value)", use(profileName),use (parameterValue);

				sql << "insert into feature (modelName, descriptor, keypoint, normal, refFrame, profileid ) values(:i, :descriptor, :keypoint, :normal, :refFrame, :profileId )", use (i), use (descriptor), use (keypoint), use (normal), use (refFrame), use (profileId);
			}
			return true;
		}
		catch (soci_error const &e)
		{
			std::cerr << "Exception: " << e.what () << '\n';
			return false;

		}
	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;

	}
	return true;
}

bool Database::loadProfile (std::string profileName, std::string &yamlPath, std::string &models)
{
	try
	{
		std::cout << "Connection established successfully. \n";
		try
		{
			std::string parameterValue = "";
			std::string modelList = "";
			int profileId = 0;
			sql << "select parameters from profile where name = :profileName", use (profileName), into (parameterValue);
			std::remove ((databasePath + "/outfile.yaml").c_str ());
			ofstream outfile;
			outfile.open ((databasePath + "/outfile.yaml").c_str ());
			yamlPath = (databasePath + "/outfile.yaml").c_str ();
			outfile << parameterValue;
			outfile.close ();
			sql << "select id from profile where name = :profileName", use (profileName), into (profileId);
			//sql << "select profile.parameters, feature.modelName from profile, feature where profile.id = :profileId", use(profileId), into(parameterValue1), into(models);
			//sql << "select parameters, modelNames from profile where profile.id = :profileId", use (profileId), into (parameterValue1), into (models);
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
			std::cout << "The parameters are: " << parameterValue << std::endl;
			return true;
		}
		catch (soci_error const &e)
		{
			std::cerr << "Exception: " << e.what () << '\n';
			return false;

		}

	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;

	}

}

bool Database::loadModel (std::string modelName)
{
	std::cout << "Connection established successfully. \n";
	sql << "create table blobTable ("
			"    id integer,"
			"    pcd oid"
			")";
	return true;
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
			modelFile.push_back (modellocation);
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
			modelFile.push_back (modellocation);
		}
		rowset<row> rs = (sql.prepare << "select modelName, descriptor, keypoint, normal, refFrame from feature where profileId = :id", use (id));
		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
		{
			row const& row = *it;
			models.push_back (row.get<string> (0));
			descriptor.push_back (row.get<string> (1));
			keypoint.push_back (row.get<string> (2));
			normal.push_back (row.get<string> (3));
			refFrame.push_back (row.get<string> (4));
			//cout << "Id: " << row.get<int> (0) << '\n' << "descriptor: " << row.get<string> (1) << " normal:" << row.get<string> (2) << '\n' << "refFrame: " << row.get<string> (3) << endl;
		}
	}

	return true;
}
/*

 bool Database::saveAllModels ()
 {
 std::cout << "Connection established successfully. \n";

 std::ifstream file (std::string (databasePath + "/example.yaml").c_str ());
 std::string paramvector;
 for (std::string value; getline (file, value);)
 {
 paramvector.append (value);
 paramvector.append ("\n");
 }
 std::cout << "the parameters are: \n" << paramvector << std::endl;

 std::clog << "1 \n";
 std::string data = "ABC\0\0\0AAA";
 std::clog << "2 \n";
 // sql << "create table soci_test1 (id int, text_value text, " "blob_value blob, longblob_value longblob)";
 //blob buffer (sql);

 std::clog << "3 \n";
 //soci::session sql << "insert into blobTable(id,pcd) values(1 , :data)", use (data);
 std::clog << "4 \n";
 //	buffer.append(buf, sizeof (buf));
 //	std::clog << "5 \n";
 string buff;
 //sql << "select pcd from blobtable where id = 1", into (buffer);
 //cout << "the blobtable input is" << buffer << endl;


 blob b(sql);
 sql << "select pcd from blobTable where id = 1", into(b);
 char buf2[100];
 b.read(0,buf2,b.get_len());
 std::cout << "The size of blob is  " << buf2 << std::endl;

 return true;
 }

 */

bool Database::addModel (const std::string name, const std::string location)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (location, cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}
	std::string storeLocation = databasePath + name + ".pcd";
	if (pcl::io::savePCDFile (storeLocation, cloud) < 0)
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

//bool Database::getModel (int profileId, std::vector<int> &id, std::vector<std::string> &name, Cloud &cloudVector, Features &feature)
//{
//	try
//	{
//		std::cout << "Connection established successfully. \n";
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//		std::vector<std::string> file;
//
//		sql << "select id(*) from models", into (id);
//		sql << "select name(*) from models", into (name);
//		sql << "select location(*) from models", into (file);
//
//		rowset<row> rs = (sql.prepare << "select id, descriptor, keypoint, normal, refFrame from feature where id = :profileId");
//		int i = 0;
//		// iteration through the resultset:
//		for (rowset<row>::const_iterator it = rs.begin (); it != rs.end (); ++it)
//		{
//			row const& row = *it;
//
//			// dynamic data extraction from each row:
//			feature.id[i] = row.get<int> (0);
//			feature.descriptor[i] = row.get<string> (1);
//			feature.keypoint[i] = row.get<string> (2);
//			feature.refFrame[i] = row.get<string> (3);
//			i++;
//			//cout << "Id: " << row.get<int> (0) << '\n' << "descriptor: " << row.get<string> (1) << " normal:" << row.get<string> (2) << '\n' << "refFrame: " << row.get<string> (3) << endl;
//		}
//
//		for (auto i : file)
//		{
//			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (i, *cloud) == -1) //* load the file
//			{
//				PCL_ERROR("Couldn't read the pcd file \n");
//				return (false);
//			}
//			cloudVector.push_back (*cloud);
//
//		}
//		return (true);
//	}
//	catch (exception const &e1)
//	{
//		std::cerr << "Connection to database failed " << e1.what () << '\n';
//		return false;
//	}
//}

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
			modelFile.push_back (loaction);
		}

		return true;
	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;
	}
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
		std::cout << "Connection established successfully. \n";
		try
		{
			std::string location;
			sql << "select location from Models where name = :modelName", use (modelName), into (location);
			//std::remove((location ).c_str ());

			//const char *des = location.data ();
			if (std::remove ((location).c_str ()) != 0)
			{
				cout << "Error while deleting file at location" << location << '\n';
			}

			else
			{
				sql << "delete from Models where name = :modelName", use (modelName);
				cout << "The model is deleted from: " << location << '\n';
				//sql << "select profile.parameters, feature.modelName from profile, feature where profile.id = :profileId", use(profileId), into(parameterValue1), into(models);
				sql << "delete from feature where modelName = :modelName", use (modelName);
			}

		}
		catch (soci_error const &e)
		{
			std::cerr << "Exception: " << e.what () << '\n';
			return false;

		}

	}
	catch (exception const &e1)
	{
		std::cerr << "Connection to database failed " << e1.what () << '\n';
		return false;
	}
	return true;
}
