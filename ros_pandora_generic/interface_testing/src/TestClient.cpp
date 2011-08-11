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
/**
 * A Class for remote calling of tests 
 * given a specific node name
 */

#include "interface_testing/TestClient.h"

/**
 * The constructor method
 */
TestClient::TestClient(std::string actionName) : _clientTester(actionName, true)
{
	int retries = 0;
	while (!_clientTester.waitForServer(ros::Duration(.1)) && ros::ok) {
		if (retries >10)
			ROS_ERROR("Couldn't find action %s",actionName.c_str());
		else
			retries++;
	}
	interface_testing::testerGoal goal;
	
	_clientTester.sendGoal(goal);
	_clientTester.waitForResult(ros::Duration(5.0));
	if (_clientTester.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("SUCCESS on test %s", actionName.c_str());
	else 
		ROS_ERROR("FAILED on test %s", actionName.c_str());
}	

/**
 * Method for selecting the specific node
 * to be tested
 */
void TestClient::testNode(std::string nodeName)
{
	XmlRpc::XmlRpcValue args, result, payload;
	ros::NodeHandle n;
	args[0] = "/";
	
	if(!ros::master::execute("getSystemState", args, result, payload, true)) {
		ROS_ERROR("Error: Could not connect to master or something else went wrong!");
		return;
	}

	//Select Node To Be Tested
	for(int i = 0; i < payload[1].size(); i++) {
		std::string topicName = std::string(payload[1][i][0]);
		if((topicName.find("/"+nodeName+"/test/")==0 || (nodeName=="" && topicName.find("/test/")!=std::string::npos)) && topicName.find("cancel")!=std::string::npos) {
			std::string testActionName = topicName.substr(0, topicName.length()-7);
			TestClient test(testActionName);
		}
	}
}

/**
 * The main method
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "testClient");
	
	TestClient::testNode (argv[1]==NULL?"":argv[1]);
}
