/*
 * Configurable.cpp
 *
 *  Created on: Feb 5, 2019
 *      Author: philipf
 */
#include "../include/RosParameterManager.h"

std::vector<std::shared_ptr<RosParameterBase>> RosParameterManager::listOfParameters;

void RosParameterManager::initParameters(ros::NodeHandle &nh)
{
	for(std::shared_ptr<RosParameterBase> p : listOfParameters)
	{
		p->initParam(nh);
	}
}

/* Reads parameters from parameter server. If there are no parameters, default parameters will be used and put onto the ros parameter server.*/
void RosParameterManager::readParameters(ros::NodeHandle &nh)
{
	for(std::shared_ptr<RosParameterBase> p : listOfParameters)
	{
		p->readFromServer(nh);
	}
}
/* Creates/Overwrites all parameters from the parameter server with its default values.*/
void RosParameterManager::setDefaultParameters(ros::NodeHandle &nh)
{
	for(std::shared_ptr<RosParameterBase> p : listOfParameters)
	{
		p->reset(nh);
	}
}

