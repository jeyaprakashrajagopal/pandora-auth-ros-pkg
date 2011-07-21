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
 * File Description: State Manager - Client Implementation
 * Contents: Methods for implementing nodes as clients for changing
 * 			 states of operation 			 
 * Author: Group 0
 * Date: 15 April 2011
 * Change History: -
 */


#include "StateClient.h"

StateClient::StateClient (bool doRegister)  : _nh() {
	name = ros::this_node::getName();
	_acknowledgePublisher = _nh.advertise<stateManager_communications::robotModeMsg>("/robot/state/server",5,true);
	_stateSubscriber = _nh.subscribe("/robot/state/clients",10, &StateClient::serverStateInformation, this);		
	
	if (doRegister)
		clientRegister();
}

void StateClient::clientRegister()
{
	while (!ros::service::waitForService("/robot/state/register", ros::Duration(.1)) && ros::ok()) 
	{
		ROS_ERROR("[%s] Couldn't find service /robot/state/register", name.c_str());
		ros::spinOnce();
	}
	_registerServiceClient = _nh.serviceClient<stateManager_communications::registerNodeSrv>
	("/robot/state/register");
	
	stateManager_communications::registerNodeSrv rq;
	rq.request.nodeName = name;
	
	while (!_registerServiceClient.call(rq) && ros::ok())
		ROS_ERROR("[%s] Failed to register node. Retrying...", name.c_str());
}

void StateClient::startTransition(int newState) {

	ROS_INFO("[%s] Starting Transition to state %i",name.c_str(), newState);	
	transitionComplete(newState);
}

void StateClient::transitionComplete(int newState){

	ROS_INFO("[%s] Node Transition to state %i Completed",name.c_str(), newState);	

	stateManager_communications::robotModeMsg msg;
	msg.nodeName = name;
	msg.mode = newState;
	msg.type = stateManager_communications::robotModeMsg::TYPE_ACK;
	_acknowledgePublisher.publish(msg);
}

void StateClient::completeTransition(){
	ROS_INFO("[%s] System Transitioned, starting work", name.c_str());
}

void StateClient::transitionToState(int newState){

	ROS_INFO("[%s] Requesting transition to state %i",name.c_str(), newState);	
	stateManager_communications::robotModeMsg msg;
	msg.nodeName = name;
	msg.mode = newState;
	msg.type = stateManager_communications::robotModeMsg::TYPE_REQUEST;
	_acknowledgePublisher.publish(msg);
	
}

void StateClient::serverStateInformation(const stateManager_communications::robotModeMsgConstPtr& msg) {
	ROS_INFO("[%s] Received new information from state server",name.c_str());
	if (msg->type == stateManager_communications::robotModeMsg::TYPE_TRANSITION) {
		startTransition(msg->mode);
	} else if (msg->type == stateManager_communications::robotModeMsg::TYPE_START) {
		completeTransition();
	} else {
		ROS_ERROR("[%s] StateClient received a new state command, that is not understandable",name.c_str());
	}
}


