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
 * File Description State Manager - Server Implementation
 * Author: Group 0
 * Date: 15 April 2011
 * Change History: -
 */

#include "state_manager/StateServer.h"

StateServer::StateServer() : _nh() {
	setStateNames();
	
	
	//Create watchdog
	_watchdog = _nh.createTimer(ros::Duration(15),&StateServer::watchdog,this);
	_timer = _nh.createTimer(ros::Duration(1),&StateServer::publishDiagnostics,this);
	_watchdog.stop();
	_transitioning = false;
	
	//Subcriber and Publisher declaration
	_statePublisher = _nh.advertise<state_manager_communications::robotModeMsg>("/robot/state/clients",5,true);	
	_registerService = _nh.advertiseService("/robot/state/register", &StateServer::registerNode, this);
	_acknowledgeSubscriber = _nh.subscribe("/robot/state/server",100, &StateServer::clientStateInformation, this);
	
	_updater.setHardwareID("none");
	_updater.add("System State", this, &StateServer::stateDiagnostic);
	
	ros::Duration w(10); //Wait 10 seconds for all nodes to start.
	w.sleep();
	_previousState = 0;
	_currentState = 0;
	_numOfNodesAcked = 0;
}

void StateServer::watchdog(const ros::TimerEvent&) {
	ROS_ERROR("State Transition Timeout");
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).currentState != _currentState)
			ROS_ERROR("Node %s timeout", nodeNames.at(i).nodeName.c_str());
	}
	sendStart(_currentState);
}

void StateServer::setStateNames() {
	stateNames[state_manager_communications::robotModeMsg::MODE_OFF] = "MODE_OFF";
	stateNames[state_manager_communications::robotModeMsg::MODE_EXPLORATION] = "MODE_EXPLORATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_IDENTIFICATION] = "MODE_IDENTIFICATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_SCAN] = "MODE_ARM_SCAN";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_APPROACH] = "MODE_ARM_APPROACH";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_SEARCH_COMPLETE] = "MODE_ARM_SEARCH_COMPLETE";
	stateNames[state_manager_communications::robotModeMsg::MODE_SEMI_AUTONOMOUS] = "MODE_SEMI_AUTONOMOUS";
	stateNames[state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION] = "MODE_TELEOPERATED_LOCOMOTION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_TELEOPERATION] = "MODE_ARM_TELEOPERATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_TELEOPERATION_TUCK] = "MODE_ARM_TELEOPERATION_TUCK";
	stateNames[state_manager_communications::robotModeMsg::MODE_TERMINATING] = " MODE_TERMINATING";
}

void StateServer::clientStateInformation(const state_manager_communications::robotModeMsgConstPtr& msg) {
	ROS_INFO("Received new msg from state client %s", msg->nodeName.c_str());
	if (msg->type ==  state_manager_communications::robotModeMsg::TYPE_REQUEST) {
		ROS_INFO("%s requested state transition to %i", msg->nodeName.c_str(),msg->mode);
		sendTransitionRequest(msg->mode);
	} else if (msg->type == state_manager_communications::robotModeMsg::TYPE_ACK) {
		registerNodeTransition(msg);
	} else {
		ROS_ERROR("State Server received a state message, but it was not understandable.");
	}
}

bool StateServer::registerNode(state_manager_communications::registerNodeSrv::Request& rq,state_manager_communications::registerNodeSrv::Response &rs) {
	stateNode node;
	node.nodeName = rq.nodeName;
	
	ROS_INFO("Node %s has registered in state manager", node.nodeName.c_str());
	node.currentState = 0;
	node.previousState = 0;
	if (!nodeListed(node.nodeName)) {
		nodeNames.push_back(node);
	} else {
		ROS_ERROR("Node %s tried to register in state manager, but is already registerd",  node.nodeName.c_str());
	}
	return true;
}

void StateServer::registerNodeTransition(const state_manager_communications::robotModeMsgConstPtr& msg) {
	if (msg->mode != _currentState){
		ROS_ERROR("Received and ACK from %s, but it transitioned to a wrong state", msg->nodeName.c_str()); 
		return;
	}
	if (!nodeListed(msg->nodeName)) {
		ROS_ERROR("Received an ACK from an unknown node %s",msg->nodeName.c_str());
		return;
	}
	ROS_INFO("%s transitioned sucessfully",msg->nodeName.c_str());
	_numOfNodesAcked++;
	
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).nodeName == msg->nodeName){
			nodeNames.at(i).previousState = nodeNames.at(i).currentState;
			nodeNames.at(i).currentState = _currentState;
		}
	}
	if (_numOfNodesAcked >= nodeNames.size()) {
		sendStart(_currentState);
		ROS_INFO("System Successfully transitioned to %i",_currentState);
	}	
}

bool StateServer::nodeListed(std::string name) {
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).nodeName == name)
			return true;
	}
	return false;
}

void StateServer::performNodeCensus() {
	nodeNames.clear();
	
	//First retrieve all nodes
	ros::V_string nodes;
	ros::master::getNodes (nodes);
	
	
	for (unsigned int i = 0; i < nodes.size(); i++){
		stateNode node;
		node.nodeName = nodes.at(i);
		if (node.nodeName == "/rosout" || node.nodeName == ros::this_node::getName())
			continue;
		ROS_INFO("%s",node.nodeName.c_str());
		node.currentState = 0;
		nodeNames.push_back(node);	
	}
}

void StateServer::sendTransitionRequest(int newState) {
	state_manager_communications::robotModeMsg msg;
	ROS_INFO("Changing system state to %i", newState);
	_previousState = _currentState;
	_currentState = newState;
	msg.nodeName = "";
	msg.mode = newState;
	msg.type = state_manager_communications::robotModeMsg::TYPE_TRANSITION;
	_statePublisher.publish(msg);	
	_numOfNodesAcked = 0;
	ROS_INFO("Informed all state clients for state change");
	_watchdog.start();
	_transitioning = true;
}

void StateServer::sendStart(int newState) {
	state_manager_communications::robotModeMsg msg;
	_watchdog.stop(); //Disable watchdog
	msg.nodeName = "";
	msg.mode = newState;
	_numOfNodesAcked = 0;
	msg.type = state_manager_communications::robotModeMsg::TYPE_START;
	_statePublisher.publish(msg);
	_transitioning = false;
	statusNode();	
}

void StateServer::statusNode() {
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		ROS_INFO("Node %s is at state %i", nodeNames.at(i).nodeName.c_str(), nodeNames.at(i).currentState);	
	}
}

std::string StateServer::getStateName(int state){
	if (state < NUM_OF_STATES) 
		return stateNames[state];
	else
		return "INVALID";
}

void StateServer::stateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    bool allOk = true;
    if(!_transitioning) {
		for (unsigned int i = 0; i < nodeNames.size(); i++) {
			if (nodeNames.at(i).currentState != _currentState) {
				allOk = false;
			}
		}
	}
	if (allOk && getStateName(_currentState) != "INVALID") {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, getStateName(_currentState));
	}
	else {
		std::string timedOutNodes;
		for (unsigned int i = 0; i < nodeNames.size(); i++) {
			if (nodeNames.at(i).currentState != _currentState) {
				timedOutNodes = timedOutNodes + " " + nodeNames.at(i).nodeName.c_str();
			}
		}
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Timed Out Nodes:" + timedOutNodes);
	}
	stat.add("System Current State ", getStateName(_currentState));
	stat.add("System Previous State ", getStateName(_previousState));
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		stat.add(nodeNames.at(i).nodeName, getStateName(nodeNames.at(i).currentState) + 
		"\t(" + getStateName(nodeNames.at(i).previousState)+")");
	}
}

void StateServer::publishDiagnostics(const ros::TimerEvent&) {
	_updater.update();
}


int main(int argc, char** argv) {
	ros::init(argc, argv,"StateServer");
	StateServer server;
	ros::spin();
}
