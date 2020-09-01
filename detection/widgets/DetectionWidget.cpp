/* * ParameterWidget.cpp *
 *  Created on: Feb 22, 2019
 *      Author: satishj
 */

#include <iostream>
#include <stdio.h>
#include "DetectionWidget.h"
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QListWidget>
#include <handling_msgs/AddModel.h>
#include <handling_msgs/DeleteModel.h>
#include <handling_msgs/SaveProfile.h>
#include <handling_msgs/GetModels.h>
#include <handling_msgs/GetProfiles.h>
#include <handling_msgs/LoadProfile.h>
#include <handling_msgs/SetSource.h>
#include <handling_msgs/DeleteProfile.h>
#include <handling_msgs/UpdateParameters.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vtkRenderWindow.h>


struct ParamType
{
		std::string name = "";
		std::string value = "";
		double valueDouble = 0;
		double valueInt = 0;
		std::string type = "";
		std::string options = "";
		std::string minimum = "";
		std::string maximum = "";
		double min = 0;
		double max = 0;
		bool boolvalue = true;
};

std::vector<ParamType> paramVector;
bool paramSet = false;
bool newModel = false;
bool newParam = false;

std::map<std::string, ParamType> mapOfParameters;
std::string strInt = "int";
std::string strDouble = "double";
std::string strBool = "bool";
std::string strEnum = "enum";
std::string strFloat = "float";

const std::string addModelService = "/detection/database/addModel";
const std::string deleteModelService = "/detection/database/deleteModel";
const std::string saveProfileService = "/detection/database/saveProfile";
const std::string getModelService = "/detection/database/getModels";
const std::string getProfileService = "/detection/database/getProfiles";
const std::string loadProfileService = "/detection/database/loadProfile";
const std::string updateParametersService = "/detection/database/updateParameters";
const std::string updateModelsService = "/detection/database/updateModels";
const std::string setSourceService = "/detection/setSource";
const std::string triggerService = "/detection/trigger";
const std::string continuousService = "/detection/continuous";
const std::string updateVisuals = "/detection/database/updateVisuals";
const std::string deleteProfileService = "/detection/database/deleteProfile";

DetectionWidget::DetectionWidget ()
{
	newItem = new QListWidgetItem;
	boolItem = new QTableWidgetItem;
	doubleItem = new QTableWidgetItem;
	intItem = new QTableWidgetItem;
	QPalette palette;

	setupUi (this);
	constructViewer ();
	palette.setColor (QPalette::Window, Qt::yellow);
	palette.setColor (QPalette::WindowText, Qt::blue);

	label_7->setAutoFillBackground (true);
	label_7->setPalette (palette);
	label_7->setText ("All the messages will be displayed here");
	//pushButtonSaveProfile->setEnabled (false);
	pushButtonAddModel_2->setEnabled (false);
	pushButtonStopDetection->setEnabled (false);
	pushButtonStartDetection->setEnabled (true);

}

DetectionWidget::~DetectionWidget ()
{
}

QWidget * DetectionWidget::createInstance ()
{
	return new DetectionWidget;
}

void DetectionWidget::init ()
{
	addServiceClient<handling_msgs::AddModel> (addModelService, addModelService);
	addServiceClient<handling_msgs::GetModels> (getModelService, getModelService);
	addServiceClient<handling_msgs::GetProfiles> (getProfileService, getProfileService);
	addServiceClient<handling_msgs::DeleteModel> (deleteModelService, deleteModelService);
	addServiceClient<handling_msgs::DeleteProfile> (deleteProfileService, deleteProfileService);
	addServiceClient<handling_msgs::SaveProfile> (saveProfileService, saveProfileService);
	addServiceClient<handling_msgs::LoadProfile> (loadProfileService, loadProfileService);
	addServiceClient<std_srvs::SetBool> (updateParametersService, updateParametersService);
	addServiceClient<std_srvs::SetBool> (continuousService, continuousService);
	addServiceClient<std_srvs::Empty> (updateModelsService, updateModelsService);
	addServiceClient<std_srvs::Empty> (triggerService, triggerService);
	addServiceClient<handling_msgs::SetSource> (setSourceService, setSourceService);
	addServiceClient<std_srvs::Empty>(updateVisuals,updateVisuals);
	addSubscriber ("/detection/visualUpdate", "/detection/visualUpdate", 1, &DetectionWidget::cloudViewerCallback, this);
	loadActveModels ();
	loadParameter ();
	getCloudVisualizer ();
	loadTopics ();
}


void DetectionWidget::loadTopics ()
{
	ros::master::V_TopicInfo vInfo;
	ros::master::getTopics (vInfo);
	QStringList list;
	for (ros::master::TopicInfo i : vInfo)
	{
		if (i.datatype.find ("PointCloud2") != std::string::npos)
		{
			list.push_back (QString::fromStdString (i.name));
			std::cout << "Adding available topic: '" << i.name << "'\n";
		}
	}
	comboBoxTopics->clear ();
	comboBoxTopics->addItems (list);
}

void DetectionWidget::loadActveModels ()
{
	listActiveModels->clear ();
	std::string models = "";
	std::string profile = "";
	getNodeHandle ()->getParam ("detection/database/models", models);
	//creating a string vector for models
	std::string word = "";
	char dl = ',';
	models = models + dl;
	std::vector<std::string> modelVector;

	for (std::size_t i = 0; i < models.size (); i++)
	{

		if (models[i] != dl)
		{
			word = word + models[i];
		}
		else
		{
			if (word.size () != 0)
				modelVector.push_back (word);
			word = "";
		}
	}
	for (auto i : modelVector)
	{
		listActiveModels->addItem (QString::fromStdString (i));
		std::cout << "the model list are...................." << i << std::endl;
	}
	getNodeHandle ()->getParam ("/detection/database/profileName", profile);
	//comboBoxLoadProfile
	loadParameter ();
}

void DetectionWidget::moveToActive ()
{
	std_srvs::Empty srv;
	if (!listAvaliableModels->currentItem ())
		return;
	auto item = listAvaliableModels->selectedItems ();
	auto list = listAvaliableModels->currentItem ()->text ();
	listActiveModels->addItem (list);
	QString allNamesInOneString;
	//pushButtonSaveProfile->setEnabled (true);
	comboBoxLoadProfile->setStyleSheet ("QComboBox { background-color: yellow; }");
	getNodeHandle ()->setParam ("/detection/database/profileName", "");
	int names = listActiveModels->count ();
	for (int i = 0; i < names; i++)
	{
		if(i < (names-1))
		{
			allNamesInOneString = allNamesInOneString + listActiveModels->item (i)->text () + ",";
		}

		else
		{
			allNamesInOneString = allNamesInOneString + listActiveModels->item (i)->text () ;
		}
	}
	std::cout << "The output string is: " << allNamesInOneString.toStdString () << std::endl;
	getNodeHandle ()->setParam ("detection/database/models", allNamesInOneString.toStdString ());

	for (auto l : item)
	{
		listAvaliableModels->removeItemWidget (l);

		delete l;
	}
	enableWaitCursor(true);
	callService (updateModelsService, srv);
	enableWaitCursor(false);
}

void DetectionWidget::removeItem ()
{
	if (!listActiveModels->currentItem ())
		return;
	auto item = listActiveModels->selectedItems ();
	auto list = listActiveModels->currentItem ()->text ();
	listAvaliableModels->addItem (list);

	//pushButtonSaveProfile->setEnabled (true);
	comboBoxLoadProfile->setStyleSheet ("QComboBox { background-color: yellow; }");
	getNodeHandle ()->setParam ("/detection/database/profileName", "");

	for (auto l : item)
	{
		listActiveModels->removeItemWidget (l);
		delete l;
	}

	int names = listActiveModels->count ();
	QString allNamesInOneString = "";
	if(names > 0)
	{
		allNamesInOneString = listActiveModels->item (1)->text ();
		for (int i = 1; i < names; i++)
		{
			allNamesInOneString = allNamesInOneString + "," + listActiveModels->item (i)->text ();
		}
	}
	std::cout << "The output string is: " << allNamesInOneString.toStdString () << std::endl;
	getNodeHandle ()->setParam ("detection/database/models", allNamesInOneString.toStdString ());
	std_srvs::Empty srvUpdateModels;
	callService (updateModelsService, srvUpdateModels);
}

void DetectionWidget::setFolder ()
{
	std::cout << "setFolder clicked..........." << "\n";
	QString filename = QFileDialog::getSaveFileName (this, tr ("Add Model"), labelModelPath->text (), tr ("PointCloudData (*.pcd);; All Files(*)"));
	std::string basename = boost::filesystem::basename (filename.toStdString ());
	if (filename.size () == 0)
		return;
	labelModelPath->setText (filename);
	lineEditModelName->setText (QString::fromStdString (basename));
	pushButtonAddModel_2->setEnabled (true);
}

void DetectionWidget::addModel ()
{
	handling_msgs::AddModel srv;
	if (lineEditModelName->text ().toStdString ().empty ())
	{
		label_7->setText ("Model name not provided");
	}
	else
	{
		label_7->setText ("");
		std::cout << " addModel to the database..........." << lineEditModelName->text ().toStdString () << "\n";
		srv.request.name = lineEditModelName->text ().toStdString ();
		std::string storeLocation = labelModelPath->text ().toStdString ();
		std::cout << " store loaction is..........." << labelModelPath->text ().toStdString () << "\n";
		srv.request.model = labelModelPath->text ().toStdString ();
		enableWaitCursor(true);
		callService (addModelService, srv);
		enableWaitCursor(false);
		if (srv.response.success == true)
		{
			lineEditModelName->clear ();
			label_7->setText ("Model added successfully");
			pushButtonAddModel_2->setEnabled (false);
		}
		refresh ();
	}
}

void DetectionWidget::deleteModel ()
{
	handling_msgs::DeleteModel srv;
	comboBoxDeleteModel->itemText (comboBoxDeleteModel->currentIndex ());
	std::cout << " deleting model from the database..........." << comboBoxDeleteModel->currentText ().toStdString () << "\n";
	srv.request.name = comboBoxDeleteModel->currentText ().toStdString ();
	enableWaitCursor(true);
	callService (deleteModelService, srv);
	enableWaitCursor(false);
	if (srv.response.success == true)
	{
		label_7->setText ("Model deleted successfully");
	}
	refresh ();
}

void DetectionWidget::publishParameter ()
{
	std::cout << "Publishing the parameters " << "\n";
	int j = 0;
	for (auto param : paramVector)
	{
		//ParamType paramType = param.type;
		std::string type = param.type;

		if (type.compare (strInt) == 0)
		{
			std::cout << "The parameters type: " << type << "\n";
			std::string paramName = param.name + "/" + "value";
			intBox = new QSpinBox;
			tableCell = new QWidget;
			tableCell = tableWidget->cellWidget (j, 1);
			intBox = static_cast<QSpinBox*> (tableCell);
			int val = intBox->value ();
			std::cout << "The name is :" << paramName << "\n";
			std::cout << " The value is : " << val << "\n";
			getNodeHandle ()->setParam (paramName, val);
			j++;
		}
		else if (type.compare (strDouble) == 0 || type.compare (strFloat) == 0)
		{
			std::cout << "The parameters type: " << type << "\n";
			std::string paramName = param.name + "/" + "value";
			std::cout << "The name is :" << paramName << "\n";
			doubleBox = new QDoubleSpinBox;
			tableCell = new QWidget;
			tableCell = tableWidget->cellWidget (j, 1);
			doubleBox = static_cast<QDoubleSpinBox*> (tableCell);
			double val = doubleBox->value ();
			std::cout << " The value is : " << val << "\n";
			getNodeHandle ()->setParam (paramName, val);
			j++;
		}
		else if (type.compare (strBool) == 0)
		{
			std::cout << "The parameters type: " << type << "\n";
			std::string paramName = param.name + "/" + "value";
			std::cout << "The name is :" << paramName << "\n";
			boolBox = new QCheckBox;
			tableCell = new QWidget;
			tableCell = tableWidget->cellWidget (j, 1);
			boolBox = static_cast<QCheckBox*> (tableCell);
			if (boolBox->isChecked ())
			{

				getNodeHandle ()->setParam (paramName, true);
				std::cout << " The value is : " << "true" << "\n";
			}
			else
			{
				std::cout << " The value is : " << "false" << "\n";
				getNodeHandle ()->setParam (paramName, false);
			}
			j++;
		}
		else if (type.compare (strEnum) == 0)
		{
			std::cout << "The parameters type: " << type << "\n";
			std::string paramName = param.name + "/" + "value";
			std::cout << "The name is :" << paramName << "\n";
			comboBox = new QComboBox;
			tableCell = new QWidget;
			tableCell = tableWidget->cellWidget (j, 1);
			comboBox = static_cast<QComboBox*> (tableCell);
			std::cout << " The value is : " << comboBox->currentText ().toStdString () << "\n";
			getNodeHandle ()->setParam (paramName, comboBox->currentText ().toStdString ());
			j++;
		}
		else
		{
			std::cout << "********************error********************** " << std::endl;
		}
	}
	std::cout << "The size of param vector is " << paramVector.size () << std::endl;
	paramSet = true;
	pushButtonSaveProfile->setEnabled (true);
	std_srvs::SetBool srvUpdateParameters;
	srvUpdateParameters.request.data = false;
	enableWaitCursor(true);
	callService (updateParametersService, srvUpdateParameters);
	enableWaitCursor(false);
	label_7->setText ("Parameters Published");
	comboBoxLoadProfile->setStyleSheet ("QComboBox { background-color: yellow; }");
	getNodeHandle ()->setParam ("/detection/database/profileName", "");
	std::string newProfile = comboBoxLoadProfile->currentText ().toStdString () + "*";
	std::cout << "The newProfile text is" << newProfile << std::endl;
	QString qstr = QString::fromStdString (newProfile);
	comboBoxLoadProfile->currentTextChanged (qstr);
	newParam = true;

}

void DetectionWidget::loadProfile ()
{
	handling_msgs::LoadProfile srvLoadProfile;
	srvLoadProfile.request.profile = comboBoxLoadProfile->currentText ().toStdString ();
	std::cout << "The loadProfile service invoked " << "\n" << std::endl;
	comboBoxLoadProfile->setStyleSheet ("QComboBox { background-color: white; }");
	//pushButtonSaveProfile->setEnabled (false);
	enableWaitCursor(true);
	callService (loadProfileService, srvLoadProfile);
	enableWaitCursor(false);
	listActiveModels->clear ();
	std::string models = "";
	getNodeHandle ()->setParam ("/detection/database/profileName", comboBoxLoadProfile->currentText ().toStdString ());
	getNodeHandle ()->getParam ("detection/database/models", models);
	lineEditSaveProfile->setText (comboBoxLoadProfile->currentText ());
	std_srvs::Empty srvUpdateModels;
	//creating a string vector for models
	std::string word = "";
	char dl = ',';
	models = models + dl;
	std::vector<std::string> modelVector;

	for (std::size_t i = 0; i < models.size (); i++)
	{

		if (models[i] != dl)
		{
			word = word + models[i];
		}
		else
		{
			if (word.size () != 0)
				modelVector.push_back (word);
			word = "";
		}
	}
	for (auto i : modelVector)
	{
		listActiveModels->addItem (QString::fromStdString (i));
	}
	loadParameter ();
	label_7->setText (QString::fromStdString ((comboBoxLoadProfile->currentText ().toStdString () + " profile has been loaded")));
	std_srvs::SetBool srvUpdateParameters;
	srvUpdateParameters.request.data = true;
	enableWaitCursor(true);
	callService (updateParametersService, srvUpdateParameters);
	callService (updateModelsService, srvUpdateModels);
	enableWaitCursor(false);
}

void DetectionWidget::saveProfile ()
{
	if (lineEditSaveProfile->text ().toStdString ().empty ())
	{
		label_7->setText ("Please provide the profile name");
	}
	else
	{
		handling_msgs::SaveProfile srv;
		srv.request.profileName = lineEditSaveProfile->text ().toStdString ();
		std::cout << "SaveProfile : " << lineEditSaveProfile->text ().toStdString () << "\n";
		enableWaitCursor(true);
		callService (saveProfileService, srv);
		enableWaitCursor(false);
		//pushButtonSaveProfile->setEnabled (false);
		lineEditSaveProfile->clear ();
		comboBoxLoadProfile->setStyleSheet ("QComboBox { background-color: white; }");
		label_7->setText ("Profile saved successfully");
		getNodeHandle ()->setParam ("/detection/database/profileName", lineEditSaveProfile->text ().toStdString ());
	}
	refresh();
}

void DetectionWidget::refresh ()
{
	listAvaliableModels->clear ();
	comboBoxDeleteModel->clear ();
	comboBoxLoadProfile->clear ();
	//listActiveModels->clear ();
	//lineEditSaveProfile->clear ();
	lineEditModelName->clear ();
	//label_7->setText ("Refreshed");
	//loadParameter();
	loadTopics ();
	handling_msgs::GetModels srvGetModels;
	std::cout << ".......Calling the getModel service......" << std::endl;
	enableWaitCursor(true);
	callService (getModelService, srvGetModels);
	enableWaitCursor(false);
	for (auto model : srvGetModels.response.models)
	{
		comboBoxDeleteModel->addItem (QString::fromStdString (model));

		//Only add the model to the list of available models if it is not active!
		bool alreadyActive = false;
		for(size_t i = 0; (i < listActiveModels->count()) && !alreadyActive; i++)
		{
			if(listActiveModels->item(i)->text().toStdString() == model)
			{
				alreadyActive = true; //Model is already active. So skip it.
			}
		}
		if(alreadyActive) continue;

		listAvaliableModels->addItem (QString::fromStdString (model));
	}

	handling_msgs::GetProfiles srvGetProfile;
	std::cout << ".......Calling the getProfile service......" << std::endl;
	enableWaitCursor(true);
	callService (getProfileService, srvGetProfile);
	enableWaitCursor(false);
	for (auto i : srvGetProfile.response.profiles)
	{
		comboBoxLoadProfile->addItem (QString::fromStdString (i));

	}
	std::cout << "refreshed" << std::endl;
}

void DetectionWidget::enableWaitCursor(bool enabled)
{
	if(enabled)
	{
		QApplication::setOverrideCursor(Qt::WaitCursor);
		QApplication::processEvents();
	}else
	{
		QApplication::restoreOverrideCursor();
		QApplication::processEvents();
	}
}

void DetectionWidget::loadParameter ()
{
	std::string package_path = ros::package::getPath ("handling_recognition");
	std::remove ((package_path + "/test.txt").c_str ());
	system (("rosparam list /detection | grep /type >>" + package_path + "/test.txt").c_str ());
	//Reading the file from the harddisk
	std::ifstream file (std::string (package_path + "/test.txt"));
	std::string parameter;

	//int rowCount = mapOfParameters.size ();
	tableWidget->setColumnCount (2);
	tableWidget->setRowCount (100);
	tableWidget->setColumnWidth (0, 250);

	std::string header0 = "ParameterName";
	std::string header1 = "ParameterValue";
	std::string header2 = "ParameterSlider";
	header_0 = new QTableWidgetItem (QString::fromStdString (header0));
	header_1 = new QTableWidgetItem (QString::fromStdString (header1));
	header_2 = new QTableWidgetItem (QString::fromStdString (header2));
	tableWidget->setHorizontalHeaderItem (0, new QTableWidgetItem (QString::fromStdString (header0)));
	tableWidget->setHorizontalHeaderItem (1, new QTableWidgetItem (QString::fromStdString (header1)));
	tableWidget->setHorizontalHeaderItem (2, new QTableWidgetItem (QString::fromStdString (header2)));

	int row = 0;
	ParamType paramtype;
	paramVector.clear ();
	if (file.fail ())
	{
		std::cout << "[database]: file '" << "/home/pg/rrs_ss18/satishj/ros/src/dexterity/test.txt" << "' does not exist." << std::endl;
	}
	else
	{
		if (file.is_open ())
		{
			while (getline (file, parameter))
			{
				if (parameter.length () > 0)
				{
					getNodeHandle ()->getParam (parameter, paramtype.type);
					size_t pos = parameter.find ("/type");
					if (pos != std::string::npos)
					{
						paramtype.name = parameter.substr (0, pos);

						if (paramtype.type.compare (strInt) == 0)
						{
							getNodeHandle ()->getParam (paramtype.name + "/" + "value", paramtype.valueInt);
							getNodeHandle ()->getParam (paramtype.name + "/" + "min", paramtype.min);
							getNodeHandle ()->getParam (paramtype.name + "/" + "max", paramtype.max);
							intBox = new QSpinBox;
							intItem = new QTableWidgetItem (QString::fromStdString (header0));
							std::string name = paramtype.name.substr (10, paramtype.name.size () - 10);
							tableWidget->setItem (row, 0, new QTableWidgetItem (QString::fromStdString (name)));
							intBox->setMaximum (paramtype.max);
							intBox->setMinimum (paramtype.min);
							intBox->setValue (paramtype.valueInt);
							tableWidget->setCellWidget (row, 1, intBox);
							paramVector.push_back (paramtype);

							row++;
						}
						else if (paramtype.type.compare (strDouble) == 0 || paramtype.type.compare (strFloat) == 0)
						{
							getNodeHandle ()->getParam (paramtype.name + "/" + "value", paramtype.valueDouble);
							getNodeHandle ()->getParam (paramtype.name + "/" + "min", paramtype.min);
							getNodeHandle ()->getParam (paramtype.name + "/" + "max", paramtype.max);
							doubleBox = new QDoubleSpinBox;
							doubleItem = new QTableWidgetItem (QString::fromStdString (header0));
							std::string name = paramtype.name.substr (10, paramtype.name.size () - 10);
							tableWidget->setItem (row, 0, new QTableWidgetItem (QString::fromStdString (name)));
							doubleBox->setMaximum (paramtype.max);
							doubleBox->setMinimum (paramtype.min);
							doubleBox->setValue (paramtype.valueDouble);
							tableWidget->setCellWidget (row, 1, doubleBox);
							paramVector.push_back (paramtype);
							row++;
						}
						else if (paramtype.type.compare (strBool) == 0)
						{
							getNodeHandle ()->getParam (paramtype.name + "/" + "value", paramtype.boolvalue);

							boolBox = new QCheckBox;
							boolItem = new QTableWidgetItem (QString::fromStdString (header0));
							std::string name = paramtype.name.substr (10, paramtype.name.size () - 10);
							tableWidget->setItem (row, 0, new QTableWidgetItem (QString::fromStdString (name)));
							if (paramtype.boolvalue)
							{
								boolBox->setChecked (paramtype.boolvalue);
							}
							else
							{
								boolBox->setChecked (paramtype.boolvalue);
							}
							tableWidget->setCellWidget (row, 1, boolBox);
							paramVector.push_back (paramtype);
							row++;
						}
						else if (paramtype.type.compare (strEnum) == 0)
						{
							getNodeHandle ()->getParam (paramtype.name + "/" + "value", paramtype.value);
							getNodeHandle ()->getParam (paramtype.name + "/" + "options", paramtype.options);
							comboBox = new QComboBox;
							enumItem = new QTableWidgetItem (QString::fromStdString (header0));
							std::string name = paramtype.name.substr (10, paramtype.name.size () - 10);
							tableWidget->setItem (row, 0, new QTableWidgetItem (QString::fromStdString (name)));
							std::stringstream ss (paramtype.options);
							while (ss.good ())
							{
								std::string subString;
								getline (ss, subString, ',');
								comboBox->addItem (QString::fromStdString (subString));
							}

							comboBox->setCurrentText (QString::fromStdString (paramtype.value));
							tableWidget->setCellWidget (row, 1, comboBox);
							paramVector.push_back (paramtype);
							row++;
						}
					}
				}
			}
			file.close ();
			std::cout << "the row count is: " << row << std::endl;
			std::cout << "The size of param vector is " << paramVector.size () << std::endl;
		}

	}
}

void DetectionWidget::cloudViewerCallback (const handling_msgs::VisualCloudUpdate& latestMsg)
{
	bool update = false;
	if (redrawModels || (latestMsg.sequendeIdModels != lastModelSequendeId))
	{
		redrawModels = false;
		double denseSize = 1.0;
		double keySize = 8.0;
		getNodeHandle ()->getParam ("/detection/visualization/denseSize/value", denseSize);
		getNodeHandle ()->getParam ("/detection/visualization/keypointSize/value", keySize);


		std::cout << "Starting drawing models!\n";
		viewer->removePointCloud ("models", 0);
		viewer->removePointCloud ("modelNormals", 0);
		if (latestMsg.models.data.size () > 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr models = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
			pcl::fromROSMsg (latestMsg.models, *models);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (models);
			viewer->addPointCloud (models, rgb, "models");
			std::cout << "dense: " << denseSize << std::endl;
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, denseSize, "models");

			if (latestMsg.modelNormals.data.size () > 0)
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>> ();
				pcl::fromROSMsg (latestMsg.modelNormals, *normals);
				viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal> (models, normals, 10, 0.007, "modelNormals");
			}
		}

		viewer->removePointCloud ("modelKeypoints", 0);
		for (std::string s : modelRFs)
		{
			viewer->removeCoordinateSystem (s, 0);
		}
		modelRFs.clear ();
		if (latestMsg.modelKeypoints.data.size () > 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
			pcl::fromROSMsg (latestMsg.modelKeypoints, *keypoints);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (keypoints);
			viewer->addPointCloud (keypoints, rgb, "modelKeypoints");
			std::cout << "Key: " << keySize << std::endl;
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, keySize, "modelKeypoints");
			if (latestMsg.modelRFs.data.size () > 0)
			{
				pcl::PointCloud<pcl::ReferenceFrame>::Ptr rfs = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>> ();
				pcl::fromROSMsg (latestMsg.modelRFs, *rfs);
				Eigen::Affine3f t;
				for (size_t i = 0; i < rfs->size (); i++)
				{
					Eigen::Map < Eigen::Matrix3f > map = rfs->at (i).getMatrix3fMap ();

					t.matrix ().block<3, 3> (0, 0) = map.block<3, 3> (0, 0);
					t.matrix ().block<3, 1> (0, 3) << keypoints->at (i).x, keypoints->at (i).y, keypoints->at (i).z;
					t.matrix ().block<1, 4> (3, 0) << 0, 0, 0, 1;
					viewer->addCoordinateSystem (0.005, t, std::to_string (i) + "model_ref", 0);
					modelRFs.push_back (std::to_string (i) + "model_ref");
				}
			}
		}

		lastModelSequendeId = latestMsg.sequendeIdModels;
		std::cout << "Done drawing models!\n";
		update = true;
	}
	if (latestMsg.sequendeIdScene != lastSceneSequenceId)
	{
		double denseSize = 1.0;
		double keySize = 8.0;
		double estimationSize = 3.0;
		getNodeHandle ()->getParam ("/detection/visualization/denseSize/value", denseSize);
		getNodeHandle ()->getParam ("/detection/visualization/estimationSize/value", estimationSize);
		getNodeHandle ()->getParam ("/detection/visualization/keypointSize/value", keySize);

		std::cout << "Starting drawing scene!\n";
		viewer->removePointCloud ("scene", 0);
		viewer->removePointCloud ("sceneNormals", 0);
		if (latestMsg.scene.data.size () > 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
			pcl::fromROSMsg (latestMsg.scene, *scene);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (scene);
			viewer->addPointCloud (scene, rgb, "scene");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, denseSize, "scene");
			if (latestMsg.sceneNormals.data.size () > 0)
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>> ();
				pcl::fromROSMsg (latestMsg.sceneNormals, *normals);
				viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal> (scene, normals, 10, 0.007, "sceneNormals");
			}
		}
		viewer->removePointCloud ("keypoints", 0);
		for (std::string s : sceneRFs)
		{
			viewer->removeCoordinateSystem (s, 0);
		}
		sceneRFs.clear ();
		if (latestMsg.sceneKeypoints.data.size () > 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
			pcl::fromROSMsg (latestMsg.sceneKeypoints, *keypoints);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (keypoints);
			viewer->addPointCloud (keypoints, rgb, "keypoints");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, keySize, "keypoints");

			if (latestMsg.sceneRFs.data.size () > 0)
			{
				pcl::PointCloud<pcl::ReferenceFrame>::Ptr rfs = boost::make_shared<pcl::PointCloud<pcl::ReferenceFrame>> ();
				pcl::fromROSMsg (latestMsg.sceneRFs, *rfs);
				Eigen::Affine3f t;
				for (size_t i = 0; i < rfs->size (); i++)
				{
					Eigen::Map < Eigen::Matrix3f > map = rfs->at (i).getMatrix3fMap ();

					t.matrix ().block<3, 3> (0, 0) = map.block<3, 3> (0, 0);
					t.matrix ().block<3, 1> (0, 3) << keypoints->at (i).x, keypoints->at (i).y, keypoints->at (i).z;
					t.matrix ().block<1, 4> (3, 0) << 0, 0, 0, 1;
					viewer->addCoordinateSystem (0.005, t, std::to_string (i) + "scene_ref", 0);
					modelRFs.push_back (std::to_string (i) + "scene_ref");
				}
			}
		}
		viewer->removePointCloud ("detectedObjects", 0);
		if (latestMsg.estimatedObjects.data.size () > 0)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectedObjects = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> ();
			pcl::fromROSMsg (latestMsg.estimatedObjects, *detectedObjects);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (detectedObjects);
			viewer->addPointCloud (detectedObjects, rgb, "detectedObjects");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, estimationSize, "detectedObjects");
		}
		//Add all correspondences
		//viewer->removeAllShapes(0);
		std::cout << latestMsg.PointA.size () << std::endl;
		std::cout << latestMsg.PointB.size () << std::endl;
		std::cout << latestMsg.LineColor.size () << std::endl;
		for (size_t i = 0; i < linesA.size (); i++)
		{
			//std::cout << "Starting with: " << "corres_" + std::to_string (i) << std::endl;
			viewer->removeShape ("corres_" + std::to_string (i), 0);
		}
		linesA.clear ();
		linesB.clear ();
		for (size_t i = 0; i < latestMsg.PointA.size (); i++)
		{
			pcl::PointXYZ pA;
			pA.x = latestMsg.PointA.at (i).x;
			pA.y = latestMsg.PointA.at (i).y;
			pA.z = latestMsg.PointA.at (i).z;
			linesA.push_back (pA);
			pcl::PointXYZ pB;
			pB.x = latestMsg.PointB.at (i).x;
			pB.y = latestMsg.PointB.at (i).y;
			pB.z = latestMsg.PointB.at (i).z;
			linesB.push_back (pB);
			viewer->addLine<pcl::PointXYZ, pcl::PointXYZ> (linesA.at (i), linesB.at (i), latestMsg.LineColor.at (i).x, latestMsg.LineColor.at (i).y, latestMsg.LineColor.at (i).z, "corres_" + std::to_string (i), 0);
		}
		lastSceneSequenceId = latestMsg.sequendeIdScene;
		std::cout << "Done drawing scene!\n";
		update = true;
	}
	if (update)
	{
		viewerWidget->update ();
	}
}

void DetectionWidget::constructViewer ()
{
	viewerWidget = new QVTKWidget;
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer> ("viewer", false);
	horizontalLayout_15->addWidget (viewerWidget, 1);
	viewerWidget->SetRenderWindow (viewer->getRenderWindow ());
	viewer->setupInteractor (viewerWidget->GetInteractor (), viewerWidget->GetRenderWindow ());
	viewerWidget->update ();
}

void DetectionWidget::loadScene ()
{
	std::cout << "setLoadScene clicked" << "\n";
	QString filename = QFileDialog::getSaveFileName (this, tr ("Add Model"), labelModelPath->text (), tr ("PointCloudData (*.pcd);; All Files(*)"));
	if (filename.size () == 0)
		return;
	label_8->setText (filename);
	handling_msgs::SetSource srv;
	srv.request.name = label_8->text ().toStdString ();
	srv.request.isTopic = false;
	srv.request.frame_id = lineEdit->text ().toStdString ();
	enableWaitCursor(true);
	callService (setSourceService, srv);
	enableWaitCursor(false);
}

void DetectionWidget::changeCameraTopic (QString Topic)
{
	std::cout << "SelectTopic Invoked" << "\n";
	handling_msgs::SetSource srv;
	srv.request.name = comboBoxTopics->itemText (comboBoxTopics->currentIndex ()).toStdString ();
	srv.request.isTopic = true;
	srv.request.frame_id = lineEdit->text ().toStdString ();
	enableWaitCursor(true);
	callService (setSourceService, srv);
	enableWaitCursor(false);
}

void DetectionWidget::triggerDetection ()
{
	std::cout << "Detection is triggered" << "\n";
	std_srvs::Empty srv;
	enableWaitCursor(true);
	callService (triggerService, srv);
	enableWaitCursor(false);

}

void DetectionWidget::startDetection ()
{
	std::cout << "Continuous Detection started" << "\n";
	std_srvs::SetBool srv;
	srv.request.data = true;
	enableWaitCursor(true);
	callService (continuousService, srv);
	enableWaitCursor(false);
	pushButtonStopDetection->setEnabled (true);
	pushButtonStartDetection->setEnabled (false);
}

void DetectionWidget::stopDetection ()
{
	std::cout << "Continuous Detection stopped" << "\n";
	std_srvs::SetBool srv;
	srv.request.data = false;
	enableWaitCursor(true);
	callService (continuousService, srv);
	enableWaitCursor(false);
	pushButtonStopDetection->setEnabled (false);
	pushButtonStartDetection->setEnabled (true);
}

void DetectionWidget::setCloudVisualizer ()
{
	bool checked = true;
	checked = checkBox->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showCorrespondences/value", checked);
	checked = checkBox_2->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showDense/value", checked);
	checked = checkBox_3->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showEstimatedObjects/value", checked);
	checked = checkBox_4->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showMatchingColor/value", checked);
	checked = checkBox_5->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showKeypoints/value", checked);
	checked = checkBox_6->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showNormals/value", checked);
	checked = checkBox_7->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showReferenceFrames/value", checked);
	checked = checkBox_8->isChecked ();
	getNodeHandle ()->setParam ("/detection/visualization/showOnlyWinner/value", checked);

	getNodeHandle ()->setParam ("/detection/visualization/denseSize/value", doubleSpinBox->value ());
	getNodeHandle ()->setParam ("/detection/visualization/estimationSize/value", doubleSpinBox_2->value ());
	getNodeHandle ()->setParam ("/detection/visualization/keypointSize/value", doubleSpinBox_3->value ());

	std_srvs::Empty srv;
	callService(updateVisuals,srv);
	redrawModels = true;
}

void DetectionWidget::getCloudVisualizer ()
{
	bool checked;
	double value = 0;
	getNodeHandle ()->getParam ("/detection/visualization/showCorrespondences/value", checked);
	std::cout << "The bool value is........." << checked << std::endl;
	checkBox->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showDense/value", checked);
	checkBox_2->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showEstimatedObjects/value", checked);
	checkBox_3->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showMatchingColor/value", checked);
	checkBox_4->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showKeypoints/value", checked);
	checkBox_5->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showNormals/value", checked);
	checkBox_6->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showReferenceFrames/value", checked);
	checkBox_7->setChecked (checked);
	getNodeHandle ()->getParam ("/detection/visualization/showOnlyWinner/value", checked);
	checkBox_8->setChecked (checked);

	getNodeHandle ()->getParam ("/detection/visualization/denseSize/value", value);
	std::cout << "The value is........." << value << std::endl;
	doubleSpinBox->setValue (value);
	getNodeHandle ()->getParam ("/detection/visualization/estimationSize/value", value);
	doubleSpinBox_2->setValue (value);
	getNodeHandle ()->getParam ("/detection/visualization/keypointSize/value", value);
	doubleSpinBox_3->setValue (value);
}

void DetectionWidget::deleteProfile()
{
	std::cout << "Delete profile invoked" << std::endl;
	handling_msgs::DeleteProfile srvDeleteProfile;
	srvDeleteProfile.request.profileName = comboBoxLoadProfile->currentText ().toStdString ();
	callService (deleteProfileService, srvDeleteProfile);
	getNodeHandle ()->setParam ("detection/database/models", "");
	listActiveModels->clear();
	refresh();
}

