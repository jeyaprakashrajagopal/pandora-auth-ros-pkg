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
#include "interface_testing/InterfaceTester.h"

	
bool InterfaceTester::checkForService(std::string serviceName) {
	return ros::service::exists(serviceName,false);	
}

bool InterfaceTester::checkForTopicPublishing(std::string topicName,std::string topicType) {
	ros::master::V_TopicInfo topicList;
	ros::master::getTopics(topicList);
	
	bool found = false;
	for (unsigned int i = 0; i < topicList.size(); i++) {
		ros::master::TopicInfo currentInfo;
		currentInfo = topicList.at(i);
		
		if (currentInfo.name ==  topicName && currentInfo.datatype == topicType){
			found = true;
			break;
		}
	}
	
	return found;	
}

bool InterfaceTester::checkForSubscribed(std::string topic) {
	XmlRpc::XmlRpcValue args, result, payload;
	ros::NodeHandle nh;
	args[0] = "/";

	if (!ros::master::execute("getSystemState", args, result, payload, true)) {
		ROS_ERROR("Error: Could not connect to master or other problem");
		return false;
	}
	
	
	for (int j =0; j< payload[1].size();j++){
		if (topic == std::string(payload[1][j][0]))
			return true;
	}
		
	return false;
}

bool InterfaceTester::checkForSubscribedNode(std::string topic, std::string node) {
	XmlRpc::XmlRpcValue args, result, payload;
	ros::NodeHandle nh;
	args[0] = "/";

	if (!ros::master::execute("getSystemState", args, result, payload, true)) {
		ROS_ERROR("Error: Could not connect to master or other problem");
		return false;
	}
	
	
	for (int j =0; j< payload[1].size();j++){
		if (std::string(payload[1][j][0]).find(topic)==0) {				
			for (int l=0; l< payload[1][j][1].size();l++)
				if(std::string(payload[1][j][1][l]).find(node)==0)
					return true;				
		}
	}
		
	return false;
}

bool InterfaceTester::checkForNodePublishing(std::string topic,std::string node) {
	XmlRpc::XmlRpcValue args, result, payload;
	ros::NodeHandle nh;
	args[0] = "/";

	if (!ros::master::execute("getSystemState", args, result, payload, true)) {
		ROS_ERROR("Error: Could not connect to master or other problem");
		return false;
	}
	
	
	for (int j =0; j< payload[0].size();j++){
		if (std::string(payload[0][j][0])==topic) {				
			for (int l=0; l< payload[0][j][1].size();l++)
				if(std::string(payload[0][j][1][l]).find(node)==0)
					return true;				
		}
	}
		
	return false;
}

bool InterfaceTester::checkForNodeService(std::string serviceName,std::string node) {
	XmlRpc::XmlRpcValue args, result, payload;
	ros::NodeHandle nh;
	args[0] = "/";

	if (!ros::master::execute("getSystemState", args, result, payload, true)) {
		ROS_ERROR("Error: Could not connect to master or other problem");
		return false;
	}
	
	
	for (int j =0; j< payload[2].size();j++){
		if (std::string(payload[2][j][0])==serviceName) {				
			for (int l=0; l< payload[2][j][1].size();l++)
				if(std::string(payload[2][j][1][l]).find(node)==0)
					return true;				
		}
	}
		
	return false;
}


bool InterfaceTester::checkForNode(std::string nodeName) {
	ros::V_string nodes;
	ros::master::getNodes (nodes);
	
	bool found = false;
	for (unsigned int i = 0; i < nodes.size(); i++){
		if (nodes.at(i) == nodeName){
			found = true;
			break;
		}
	}
	
	return found;
}

bool InterfaceTester::checkForMaster() {
	return ros::master::check();
}

bool InterfaceTester::checkForActionServer(std::string actionName, std::string type)
{

	return checkForTopicPublishing(actionName+"/feedback",type+"ActionFeedback") 
			&& checkForTopicPublishing(actionName+"/status","actionlib_msgs/GoalStatusArray") 
			&& checkForTopicPublishing(actionName+"/result",type+"ActionResult") 
			&& checkForSubscribed(actionName+"/goal") 
			&& checkForSubscribed(actionName+"/cancel");
}

bool InterfaceTester::checkForActionNodeServer(std::string actionName, std::string nodeName)
{
	return checkForNodePublishing(actionName+"/feedback",nodeName) 
			&& checkForNodePublishing(actionName+"/status",nodeName) 
			&& checkForNodePublishing(actionName+"/result",nodeName) 
			&& checkForSubscribedNode(actionName+"/goal",nodeName) 
			&& checkForSubscribedNode(actionName+"/cancel",nodeName);
}

bool InterfaceTester::checkForActionClient(std::string actionName, std::string type)
{
	return checkForTopicPublishing(actionName+"/goal",type+"ActionGoal") && 
		checkForTopicPublishing(actionName+"/cancel","actionlib_msgs/GoalID") &&
		checkForSubscribed(actionName+"/feedback") &&
		checkForSubscribed(actionName+"/status") &&
		checkForSubscribed(actionName+"/result");		
}

bool InterfaceTester::checkForActionNodeClient(std::string actionName, std::string nodeName)
{
		return checkForNodePublishing(actionName+"/goal", nodeName) && 
		checkForNodePublishing(actionName+"/cancel", nodeName) &&
		checkForSubscribedNode(actionName+"/feedback", nodeName) &&
		checkForSubscribedNode(actionName+"/status", nodeName) &&
		checkForSubscribedNode(actionName+"/result", nodeName);	
	
}
