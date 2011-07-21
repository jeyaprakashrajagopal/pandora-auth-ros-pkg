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
#ifndef PUBLISHER_TESTER
#define PUBLISHER_TESTER

#include "ros/ros.h"
#include "interface_testing/testerAction.h"
#include <actionlib/server/simple_action_server.h>

/**
 * Template class used for testing a publisher.
 * @author Miltos Allamanis, Marina Stamatiadou
 */
 
typedef actionlib::SimpleActionServer<interface_tester::testerAction> testActionServer;

template <class MessageType>
class PublisherTester {
	private:
		/**
		 * The ROS node handle.
		 */
		ros::NodeHandle nh;
		
		/**
		 * The action server used for testing
		 */
		testActionServer _testPublisher;
		
		/**
		 * The subscriber used for subscribing to the topic being tested.
		 */
		ros::Subscriber sub;
			
		/**
		 * A volatile boolean variable for indicating that the test
		 * message has been received.
		 */
		volatile bool received;
		
		/**
		 * Boolean variable idicating that the test has started
		 */
		volatile bool testStarted;
		
		/**
		 * Counter indicating the number of tries
		 */
		int counter;
		
		/**
		 * The topic on which to test the Subscriber.
		 */
		std::string topicName;
		
		/**
		 * Action topic used for the testing action
		 */
		std::string actionTopic;
		
		/**
		 * The timer used to check for test progress
		 */
		ros::Timer _testTimer;
		
		/**
		 * The Tester Timer
		 */
		void testTimer(const ros::TimerEvent&);
    
		/**
		 * Method to perform test. The test published a test message
		 * (frame_id = "health") and waits to be catched or a timeout
		 * occurs.
		 * @return true if the test succeeded, false otherwise
		 */
		bool performTest();
		
		/**
		 * The callback method that starts the test
		 * use as an action server
		 * @return true if test has been performed
		 */
		void startPublisherTester(const interface_tester::testerGoalConstPtr &testGoal);
		
		/**
		 * Catch a message received at a subscriber and test if the 
		 * message is a test message.
		 * @param msg the message that was catched.
		 * @return true if the message that was catched was a test
		 * 			message, false otherwise.
		 */
		void catchMsg(boost::shared_ptr<MessageType const> msg);
		
		
	public:
		/**
		 * The object constructor.
		 * @param name the name of the topic that the tested subscriber
		 * 			is listening.
		 * @param serviceName the service name to expose for subscriber
		 * 			testing.
		 */
		PublisherTester(std::string name, std::string actionName);
		
		/**
		 * Initiate the test. For the test to start we first wait for 
		 * the publisher to emit a message and then we wait for to catch
		 * the message.
		 * @return true if the test succeeds, false otherwise
		 */
		bool startTest();
		

		/**
		 * Checkout message.
		 */
		void checkoutMsg(MessageType& msg);
};

#include "src/PublisherTester.cpp"
#endif
