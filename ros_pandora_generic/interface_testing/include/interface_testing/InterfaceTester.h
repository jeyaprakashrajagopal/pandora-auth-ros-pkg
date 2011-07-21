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
#ifndef INTERFACETESTER_H
#define INTERFACETESTER_H
#include "ros/ros.h"

/**
 * A class for testing for various interafaces
 */
 

class InterfaceTester {
	private:
		/**
		 * Constructor of class
		 */
		InterfaceTester() {};
	
	public:		
		/**
		 * Tests if a service has been advertised.
		 * @param serviceName the name of the service to test against
		 * @return true if the service exists
		 */
		static bool checkForService(std::string serviceName);
		
		/**
		 * Test if a topic exisits.
		 * @param topicName the name of the topic
		 * @param topicType the type of the topic
		 * @return true if topic exists
		 */
		static bool checkForTopicPublishing(std::string topicName,std::string topicType);
		
		/**
		 * Checks if a specific node is publishing at a node.
		 * @param topicName the name of the topic
		 * @param nodeName a string that is the beginning (or whole) of the node name
		 * @return true if the node publishes on the given topic
		 */
		static bool checkForNodePublishing(std::string topicName,std::string nodeName);
		
		/**
		 * Checks if a specific node offers a specific service.
		 * @param serviceName the name of the topic
		 * @param nodeName a string that is the beginning (or whole) of the node name
		 * @return true if the node publishes on the given topic
		 */
		static bool checkForNodeService(std::string serviceName,std::string nodeName);
		
		/**
		 * Checks if a node is running.
		 * @param nodeName the name of the node
		 * @return true if the node seems to be running
		 */
		static bool checkForNode(std::string nodeName);
		
		/**
		 * Checks for an action server.
		 * @param actionName the action name
		 * @param type the type of the action
		 * @return true if action exists
		 */
		static bool checkForActionServer(std::string actionName, std::string type);
		
		/**
		 * Checks for an action server.
		 * @param actionName the action name
		 * @param nodeName the name of the node acting as an action server
		 * @return true if action exists
		 */
		static bool checkForActionNodeServer(std::string actionName, std::string nodeName);
		
		/**
		 * Checks for an action client.
		 * @param actionName the action name
		 * @param type the action type
		 * @return true if action exists
		 */
		static bool checkForActionClient(std::string actionName, std::string type);
		
		/**
		 * Checks for an node action client.
		 * @param actionName the action name
		 * @param nodeName the node acting as client
		 * @return true if action exists
		 */
		static bool checkForActionNodeClient(std::string actionName, std::string nodeName);
		
		/**
		 * A simple wrapper for checking if master is running
		 * @return true if master is running
		 */
		static bool checkForMaster();
		
		/**
		 * Checks if any message subscriber.
		 * @param topic to check if any node is subscribed
		 * @return true if subscribed succesfully
		 */
		static bool checkForSubscribed(std::string topic);
		
		/**
		 * Check if a specific node is subscribed at a specific topic.
		 * @param topic the topic name
		 * @param  a string that is the beginning (or whole) of the node name
		 * @return true if the node is subscribed to the topic
		 */
		static bool checkForSubscribedNode(std::string topic, std::string node);
		
		
	
};

#endif 
