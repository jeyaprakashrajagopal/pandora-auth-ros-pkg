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


#include "gmock/gmock.h"  
#include "gtest/gtest.h"
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#define MOCK_ACTION_SERVER(name,type) 									/**																	
	 * An pure abstract interface to an action server.					
	 */																	\
class name##ActionServer { 												\
																		\
	private:																/**																	
	 * ROS Node Handle.													
	 */																	\
	ros::NodeHandle _nh;													/**																	
	 * The action name.													
	 */																	\
	std::string _actionName;												/**																	
	 * The declaration of the actionlib									
	 */																	\
	actionlib::SimpleActionServer<type##Action> _action;				\
																		\
	public:																	/**																	
	 * The object constructor.											
	 * @param name the name of the action that the tested server		
	 * 				is being called.									
	 */																	\
	name##ActionServer(std::string name);									/**																	
	 * Destructor														
	 */																	\
	virtual ~name##ActionServer() {};										/**																	
	* Abstract function to actionServer callback						
	*/																	\
	virtual void actionServerCallback(const type##GoalConstPtr &goal) = 0; \
																		\
};																		\
																		\
name##ActionServer::name##ActionServer(std::string name) : _action(name, false)	\
{																		\
	_actionName = name;													\
	_action.start();													\
}																		 /**																	
 * The Google Mock ActionServer.										
 */																		\
class Mock##name##ActionServer : public name##ActionServer {			\
																		\
	public:																\
																		\
	Mock##name##ActionServer(std::string actionName) : name##ActionServer(actionName) {}; \
																		\
	MOCK_METHOD1(actionServerCallback, void(const type##GoalConstPtr &goal)); \
																		\
};																

#endif




	
