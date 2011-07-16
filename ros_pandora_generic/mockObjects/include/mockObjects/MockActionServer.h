#ifndef MOCK_ACTION_SERVER_H
#define MOCK_ACTION_SERVER_H


#include "gmock/gmock.h"  // Brings in Google Mock.
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#define MOCK_ACTION_SERVER(name,type) 									\
class name##ActionServer { 												\
																		\
	private:															\
																		\
	ros::NodeHandle _nh;												\
																		\
	std::string _actionName;											\
																		\
	actionlib::SimpleActionServer<type##Action> _action;				\
																		\
	public:																\
	name##ActionServer(std::string name);								\
	virtual ~name##ActionServer() {};									\
																		\
	virtual void ActionServerCallback(const type##GoalConstPtr &goal) = 0; \
																		\
};																		\
																		\
name##ActionServer::name##ActionServer(std::string name) : _action(name, false)	\
{																		\
	_actionName = name;													\
	_action.start();													\
}																		\
																		\
class Mock##name##ActionServer : public name##ActionServer {			\
																		\
	public:																\
																		\
	Mock##name##ActionServer(std::string actionName) : name##ActionServer(actionName) {}; \
																\
	MOCK_METHOD1(ActionServerCallback, void(const type##GoalConstPtr &goal)); \
																\
};																

#endif




	
