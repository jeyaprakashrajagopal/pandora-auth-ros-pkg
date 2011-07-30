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
#ifndef STATE_CLIENT
#define STATE_CLIENT

#include "ros/ros.h"
#include "stateManager_communications/robotModeMsg.h"
#include "stateManager_communications/registerNodeSrv.h"

class StateClient {
	private:
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Publisher to the state topic
		 */
		ros::Publisher _acknowledgePublisher;
		
		/**
		 * Subscriber to the state topic
		 */
		ros::Subscriber _stateSubscriber;
		
		/**
		 * Callback for receiving state info from server.
		 */
		void serverStateInformation(const stateManager_communications::robotModeMsgConstPtr&);
		
		void clientRegister();
	
		/**
		 * The node name.
		 */
		std::string name;
		
		/**
		 * Client for registering
		 */
		ros::ServiceClient _registerServiceClient;
		
		/**
		 * True to register to server.
		 */
		bool registerToServer;
		
	
	protected: 
		/**
	   * Start the client transition.
	   * @param newState the new state
	   */
		virtual void startTransition(int newState);
		
		/**
		 * Update the system that we have finished transitioning.
		 * @param newState the state at which the client has transitioned.
		 */
		void transitionComplete(int newState);
	
	public:
	
		/**
		 * Constructor.
		 */	
		StateClient (bool doRegister = true);
		
		/**
		 * Update the client that the system has tranisitioned.
		 */
		virtual void completeTransition();
	
		/**
		 * Ask the system to change the transition/
		 * @param newState the state to transition to
		 */
		void transitionToState(int newState);
	
	
};

#endif
