/*
 * Configurable.h
 *
 *  Created on: Jan 23, 2019
 *      Author: philipf
 *
 *  RosParameterManager will manage all RosParameter objects. Other objects which uses RosParameters should only get pointers, which are created by createParam()!
 *  If readParameters() is called, all parameters will be updated to the value on the ros parameter server.
 *  Call initParameters() to create parameters on the parameter server.
 *
 *  Read usage in RosParameter.h
 */

#ifndef SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETERMANAGER_H_
#define SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETERMANAGER_H_

#include <memory>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "RosParameter.h"
#include <handling_msgs/Trigger.h>

class RosParameterManager
{
	public:
	/* Inits each parameter with values from the ros parameter server. If value does not exist on parameter server, default value will be put on the server.*/
	void initParameters(ros::NodeHandle &nh);

	/* Reads parameters from parameter server. If there are no parameters, default parameters will be used and put onto the ros parameter server.*/
	static void readParameters(ros::NodeHandle &nh);

	/* Creates/Overwrites all parameters from the parameter server with its default value.*/
	void setDefaultParameters(ros::NodeHandle &nh);

	static bool deleteParam(std::shared_ptr<RosParameterBase> ptr)
	{
		for(size_t i = 0; i < RosParameterManager::listOfParameters.size(); i++)
		{
			auto p = RosParameterManager::listOfParameters.at(i);
			if(p.get() == ptr.get())
			{
				RosParameterManager::listOfParameters.at(i) = RosParameterManager::listOfParameters.at(RosParameterManager::listOfParameters.size()-1);
				RosParameterManager::listOfParameters.resize(RosParameterManager::listOfParameters.size()-1);
				return true;
			}
		}
		return false;
	}

	/* If a new RosParameter struct is created, it will register itself here.*/
	template<typename T>
	static std::shared_ptr<RosParameter<T>> createParam(T value, const ParamType type, const std::string topic)
	{
		auto param = std::make_shared<RosParameter<T>>(value, type, "/detection" + topic);
		RosParameterManager::listOfParameters.push_back(param);
		return param;
	}

	/* If a new RosParameter struct is created, it will register itself here.*/
	template<typename T>
	static std::shared_ptr<RosParameter<T>> createParam(T value, const ParamType type, const double min, const double max, const std::string topic)
	{
		auto param = std::make_shared<RosParameter<T>>(value, type, min, max, "/detection" + topic);
		RosParameterManager::listOfParameters.push_back(param);
		return param;
	}

	/* If a new RosParameter struct is created, it will register itself here.*/
	template<typename T>
	static std::shared_ptr<RosParameter<T>> createParam(T value, const ParamType type, const std::string options, const std::string topic)
	{
		auto param = std::make_shared<RosParameter<T>>(value, type, options, "/detection" + topic);
		RosParameterManager::listOfParameters.push_back(param);
		return param;
	}

	private:
	static std::vector<std::shared_ptr<RosParameterBase>> listOfParameters; //All RosParameter structs will be stored here.
};



#endif /* SRC_DEXTERITY_DETECTION_INCLUDE_ROSPARAMETERMANAGER_H_ */
