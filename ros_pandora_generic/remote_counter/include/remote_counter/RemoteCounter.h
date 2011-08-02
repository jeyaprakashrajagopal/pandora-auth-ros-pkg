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
#include "ros/ros.h"
#include "remote_counter/countingSrv.h"
		
/**
 * This class implements a remote counter that can be incremented and decremented remotely by other ROS Nodes.
 */


class RemoteCounter{
	
		/**
		 * The ros nodeHandle 
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Actual counter
		 */
		int timesCalled;
		
		/**
		 * Service server
		 */
		ros::ServiceServer _counterServer;
		
		/**
		 * The timestamp that the last activity occured.
		 */
		ros::Time lastActivity;
		
		/**
		 * The counter name.
		 */
		std::string _name;
		
	public:
		/**
		* Constructor
		* @param The name of the node that calls the RemoteCounter
		*/
		RemoteCounter(std::string);
		
		/**
		 * The ROS Service callback for the RemoteCounter.
		 */
		bool counterCallback(remote_counter::countingSrv::Request &req, remote_counter::countingSrv::Response &res);
		
		/**
		 * Set the value stored in the counter
		 * @param value the value to set the counter at
		 * @param setTimestamp true to set the timestamp
		 */
		void setValue(int value, bool setTimestamp = false);
		
		/**
		 * Get the value stored in the counter
		 */
		int getValue();
		
		/**
		 * Returns the timestamp that the counter has been last modified.
		 */
		ros::Time getLastActivity();
		
		
	
	};
