/*
 * RosParameter.h
 *
 *  Created on: Feb 5, 2019
 *      Author: philipf
 *
 *  This class is used to create a variable which will be present on the parameter server and can be read by the gui and visualized.
 *  Please use RosParameterManager for creating RosParameter and only give other classes access to a pointer to RosParameter.
 *
 *
 *  Usage:
 *	//Create a static object of parameter which can be accessed anywhere!
 *	static ParameterManager myParameterManager;
 *
 *	//When ros is available and all your parameters should be created, call this. Parameters which have not yet been added to the parameter Manager will not be visible!
 *	myParameterManager.initParameters(nh);
 *
 *	//When you have changed your parameters on the ros parameter server, call the following to update all parameters:
 *	RosParameterManager::readParameters(nh);
 *
 *
 *	//Somewhere in your class which uses parameters:
 *	//Create your parameter globally inside your class!
 *	RosParameter<double>::Ptr radius;
 *
 *	//Let ParameterManager create a param! Use this inside your constructor!
 *	KeypointCreator()
 *	{
 *		radius = RosParameterManager::createParam<double> (20.0,ParamType::DOUBLE,0.0,100.0,"/keypoints/radius");
 *	}
 *
 *	//You can now access/modify the value by the following:
 *	radius->var = radius->var * 2.0;
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETER_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETER_H_

#include <memory>
#include <ros/ros.h>
//#include "../include/Blackboard.h"

/* Can be any type. Stores the value of wished type and the default value.
 * Therefore you do not need to create a variable and a const default version of that variable in your header file.*/
class RosParameterBase
{
	public:
		virtual ~RosParameterBase () = default;
		virtual void initParam(ros::NodeHandle &nh) = 0;
		virtual void reset (ros::NodeHandle &nh) = 0;
		virtual void readFromServer(ros::NodeHandle &nh) = 0;
};

enum class ParamType
{
	BOOL,DOUBLE,ENUM,FLOAT,INT,STRING
};

template<typename T>
struct RosParameter: public RosParameterBase
{
	using Ptr = std::shared_ptr<RosParameter<T>>;

	T var;
	const std::string topic;

	private:
		T defaultVar;
		const ParamType type;
		const double min;
		const double max;
		const std::string options;

	public:

	RosParameter(T d, const ParamType t, const std::string paramTopic);
	RosParameter(T d, const ParamType t, const double min, const double max, const std::string paramTopic);
	RosParameter(T d, const ParamType t, const std::string options, const std::string paramTopic);

	void initParam(ros::NodeHandle &nh) override;

	//Resets value to defaultVar. Also resets the ros parameters servers value to defaultVar.
	void reset(ros::NodeHandle &nh) override;

	//Reads value from the ros parameter server. If there is no value on the server, defaultVar will be used and a value will be created on the parameter server.
	void readFromServer(ros::NodeHandle &nh) override;
};

/*Constructor for using bool or string. Also other values without min/max limit can be set here.*/
template <typename T>
RosParameter<T>::RosParameter(T d, const ParamType t, const std::string paramTopic)
:var(d),topic(paramTopic),defaultVar(d),type(t),min(0),max(0),options("")
{
}
/*Constructor for using int, double, float. Min and Max can be set to 0, then gui will not create a limitation for the value.*/
template <typename T>
RosParameter<T>::RosParameter(T d, const ParamType t, const double min, const double max, const std::string paramTopic)
:var(d),topic(paramTopic),defaultVar(d),type(t),min(min),max(max),options("")
{
}
/*Constructor for using enums, which are actually strings. You can give all options as comma separated values.*/
template <typename T>
RosParameter<T>::RosParameter(T d, const ParamType t, const std::string options, const std::string paramTopic)
:var(d),topic(paramTopic),defaultVar(d),type(t),min(0),max(0),options(options)
{
}

/*Resets value to defaultVar. Also resets the ros parameters servers value to defaultVar.*/
template <typename T>
void RosParameter<T>::initParam(ros::NodeHandle &nh)
{
	if(nh.param<T>(topic + "/value",var,defaultVar)) //Does the parameter already exist? Get value and we are happy. :-)
	{
		defaultVar = var;
	}else
	{
		switch(type)
		{
			case ParamType::BOOL:
			{
				std::cout << "bool:\n";
				nh.setParam(topic + "/type","bool");
				break;
			}
			case ParamType::DOUBLE:
			{
				std::cout << "double:\n";
				nh.setParam(topic + "/type","double");
				nh.setParam(topic + "/min",min);
				nh.setParam(topic + "/max",max);
				break;
			}
			case ParamType::ENUM:
			{
				std::cout << "enum:\n";
				nh.setParam(topic + "/type","enum");
				nh.setParam(topic + "/options",options);
				break;
			}
			case ParamType::FLOAT:
			{
				std::cout << "float:\n";
				nh.setParam(topic + "/type","float");
				nh.setParam(topic + "/min",min);
				nh.setParam(topic + "/max",max);
				break;
			}
			case ParamType::INT:
			{
				std::cout << "int:\n";
				nh.setParam(topic + "/type","int");
				nh.setParam(topic + "/min",min);
				nh.setParam(topic + "/max",max);
				break;
			}
			case ParamType::STRING:
			{
				std::cout << "string:\n";
				nh.setParam(topic + "/type","string");
				break;
			}
		}
		std::cout << "Creating parameterV2: " << topic << std::endl; //We have to create parameter and other values which help gui.
		nh.setParam(topic + "/value",defaultVar);
	}
}

/*Resets value to defaultVar. Also resets the ros parameters servers value to defaultVar.*/
template <typename T>
void RosParameter<T>::reset(ros::NodeHandle &nh)
{
	var = defaultVar;
	nh.setParam(topic + "/value",defaultVar);
}

/*Reads value from the ros parameter server. If there is no value on the server, defaultVar will be used and a value will be created on the parameter server.*/
template <typename T>
void RosParameter<T>::readFromServer(ros::NodeHandle &nh)
{
	if(!nh.param<T>(topic + "/value",var,defaultVar))
	{
		nh.setParam(topic + "/value",defaultVar);
	}
}

#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETER_H_ */
