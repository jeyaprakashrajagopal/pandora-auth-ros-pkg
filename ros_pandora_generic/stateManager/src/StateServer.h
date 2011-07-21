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
#ifndef STATE_SERVER
#define STATE_SERVER

#include "ros/ros.h"
#include "stateManager_communications/robotModeMsg.h"
#include "stateManager_communications/registerNodeSrv.h"
#include "diagnostic_updater/diagnostic_updater.h"

#define NUM_OF_STATES 11

/**
 * State Information Internal Structure.
 */
 
struct stateNode {
	std::string nodeName;
	int currentState;
	int previousState;	
};

class StateServer {
		private:
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Publisher to the state topic
		 */
		ros::Publisher _statePublisher;
		
		/**
		 * Subscriber to the state topic
		 */
		ros::Subscriber _acknowledgeSubscriber;
		
		/**
		 * Register service
		 */
		ros::ServiceServer _registerService;
		
		/**
		 * Timer watchdog
		 */
		ros::Timer _watchdog;
		
		/**
		 * Timer for monitoring
		 */
		ros::Timer _timer;
		
		/**
		 * The diagnostic updater
		 */		
		diagnostic_updater::Updater _updater;
		
		/**
		 * Variable indicating that the system is transitioning.
		 */
		bool _transitioning;
		
		/**
		 * Client State information getter (callback)
		 */
		void clientStateInformation(const stateManager_communications::robotModeMsgConstPtr&);
		
		/**
		 * Register node to a new state
		 */
		void registerNodeTransition(const stateManager_communications::robotModeMsgConstPtr& msg);
		
		/**
		 * Register a node.
		 */
		bool registerNode(stateManager_communications::registerNodeSrv::Request& rq,stateManager_communications::registerNodeSrv::Response &rs);
		
		/**
		 * Send a transition request to all nodes
		 */
		void sendTransitionRequest(int newState);
		
		/**
		 * Send the start msg to nodes.
		 */
		void sendStart(int newState);
		
		/**
		 * The current state
		 */
		int _currentState;
		
		/**
		 * The previous state
		 */
		int _previousState;
		
		/**
		 * The number of nodes that Ack'ed
		 */
		unsigned int _numOfNodesAcked;
		
		/**
		 * A vector of the subscribed node names.
		 */
		std::vector<stateNode> nodeNames;
		
		/**
		 * The names of the states.
		 */
		std::string stateNames[NUM_OF_STATES];
		
		
		inline std::string getStateName(int state);
		
		/**
		 * Perform a census of all running nodes.
		 */
		void performNodeCensus();
		
		/**
		 * Checks if a node is listed.
		 * @param name the name of the node being searched
		 */
		bool nodeListed(std::string name);
		
		/**
		 * Watchdog timer waiting for system transition
		 */
		void watchdog(const ros::TimerEvent&);
		
		/**
		 * Giving info about nodes and current state
		 */
		void statusNode();
		
		void setStateNames();
		
		/**
		 * Monitors the behavior of state manager
		 */
		void stateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&);
		
		/**
		 * Displays the diagnostic messages
		 */
		void publishDiagnostics(const ros::TimerEvent&);
		
		public:
		
		/**
		 * Constructor.
		 */
		StateServer();
		
	

};

#endif
