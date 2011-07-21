/*
 *	Copyright (C) 2011 by Pandora Robotics Team, Aristotle Univeristy of Thessaloniki, Greece
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in
 *	all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *	THE SOFTWARE.
 */
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




	
