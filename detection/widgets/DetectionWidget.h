/*
 * ParameterWidget.h
 *
 *  Created on: Feb 22, 2019
 *      Author: satishj
 */

#ifndef SRC_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_PARAMETERWIDGET_H_
#define SRC_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_PARAMETERWIDGET_H_
/*
 * Include ROS-Headers inside Q_MOC_RUN guard to avoid problems with boost library
 */
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#endif
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <QtCore/QtPlugin>
#include <QtWidgets/QWidget>
#include <QVTKWidget.h>
#include <handling_msgs/VisualCloudUpdate.h>
#include <core/WidgetPluginInterface.h>
#include <core/BaseWidget.h>
#include <ui_DetectionWidget.h>
#include <std_msgs/Int8MultiArray.h>

class DetectionWidget : public BaseWidget, private Ui_DetectionWidget
{
		Q_OBJECT
		Q_INTERFACES (WidgetPluginInterface)
		Q_PLUGIN_METADATA(IID "DetectionWidget")

	public:
		DetectionWidget ();
		virtual ~DetectionWidget ();
		QWidget* createInstance () override;
		void init () override;
	private:
		/*Callback from detection to gui. New clouds and lines for visualization will be received here.*/
		void cloudViewerCallback (const handling_msgs::VisualCloudUpdate& latestMsg);
		/*Initializes the viewer. Has to be called in constructor.*/
		void constructViewer ();
		/*Visual feedback for the user that a processing step has started.*/
		void enableWaitCursor(bool enabled);
		QListWidgetItem * newItem;
		QTableWidgetItem * header_0;
		QTableWidgetItem * header_1;
		QTableWidgetItem * header_2;
		QTableWidgetItem * intItem;
		QTableWidgetItem * doubleItem;
		QTableWidgetItem * boolItem;
		QTableWidgetItem * enumItem;
		QSpinBox *intBox;
		QDoubleSpinBox *doubleBox;
		QComboBox *comboBox;
		QCheckBox *boolBox;
		QWidget *tableCell;

		//Pcl visualizer
		unsigned long lastModelSequendeId = 0;
		unsigned long lastSceneSequenceId = 0;
		bool dirtyViewer = false;
		bool redrawModels = false;
		pcl::visualization::PCLVisualizer::Ptr viewer;
		QVTKWidget* viewerWidget;
		std::vector<std::string> sceneRFs;
		std::vector<std::string> modelRFs;
		std::vector<pcl::PointXYZ> linesA;
		std::vector<pcl::PointXYZ> linesB;

	private Q_SLOTS:
		void moveToActive (); //Moves the models from available model region to active region
		void removeItem (); //Moves the models from  model active region to available region
		void setFolder (); //set the folder path adding new model
		void addModel (); //invoke add model service to adds the model to the DB
		void deleteModel (); //invoke delete model service to delete model from db
		void publishParameter (); //Pushes all the parameters currently available in the table to the ros server
		void loadProfile (); //Loads the models and parameters for a specific profile
		void saveProfile (); //Saves the models from the active list and the currently published parameters as a profile to the DB
		void refresh () override;
		void loadParameter (); //Creates the table view and loads the parameters from the ros parameter server
		void loadActveModels(); //creates the active model list from the active models present on the ros parameter server
		void loadScene();
		void changeCameraTopic(QString Topic); //used to switch between the camera and gazebo
		void triggerDetection(); //run detection once
		void startDetection(); //runs detection continuously
		void stopDetection(); //stops the running detection
		void setCloudVisualizer(); //sets the ros parameters for the Visualizer
		void getCloudVisualizer(); //gets the parameters from the ros server for the Visualizer
		void loadTopics();
		void deleteProfile();

};

#endif /* SRC_GETBOT_SRC_OPERATOR_INTERFACE_WIDGETS_PARAMETERWIDGET_H_ */
