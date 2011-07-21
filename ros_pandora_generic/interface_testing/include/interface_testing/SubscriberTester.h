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
#ifndef SUBSCRIBER_TESTER
#define SUBSCRIBER_TESTER

#include "ros/ros.h"
#include "interface_testing/testerAction.h"
#include <actionlib/server/simple_action_server.h>

/**
 * A template class for testing a subscriber. When performTest() is 
 * called a test message is published and the we wait for the message
 * to be cached by catchMsg() that should be embedded to the subscriber
 * listener.
 * @author Miltos Allamanis, Marina Stamatiadou
 * 
 */
 
typedef actionlib::SimpleActionServer<interface_tester::testerAction> testActionServer;

template <class MessageType>
class SubscriberTester {
	private:
	
		/**
		 * The ROS node handle.
		 */
		ros::NodeHandle nh;
		
		/** 
		 * The ServiceTester used for
		 * performing the test as an action server
		 */
		testActionServer _testSubscriber;
		
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
		 * Method to perform test. The test published a test message
		 * (frame_id = "health") and waits to be catched or a timeout
		 * occurs.
		 * @return true if the test succeeded, false otherwise
		 */
		bool performTest();
    
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
		 * The callback method that starts the test
		 * use as an action server
		 * @return true if test has been performed
		 */
		void startSubscriberTester(const interface_tester::testerGoalConstPtr &testGoal);
		
		/**
		 * The Tester Timer
		 */
		void testTimer(const ros::TimerEvent&);
		
	public:
		/**
		 * The object constructor.
		 * @param name the name of the topic that the tested subscriber
		 * 			is listening.
		 * @param serviceName the service name to expose for subscriber
		 * 			testing.
		 */
		SubscriberTester(std::string name, std::string actionName);
		
		/**
		 * Catch a message received at a subscriber and test if the 
		 * message is a test message.
		 * @param msg the message that was catched.
		 * @return true if the message that was catched was a test
		 * 			message, false otherwise.
		 */
		bool catchMsg(boost::shared_ptr<MessageType const> msg);
		
		/**
		 * Overloaded Function
		 */
		bool catchMsg(MessageType const &msg);
		
		/**
		 * Initiate the test. For the test to start we first wait for 
		 * the publisher to emit a message and then we wait for to catch
		 * the message.
		 * @return true if the test succeeds, false otherwise
		 */
		bool startTest();
};

template <class MessageType>
SubscriberTester<MessageType>::SubscriberTester(std::string name, std::string actionName) : 
	_testSubscriber(nh, "/test/" + actionName, boost::bind(&SubscriberTester<MessageType>::startSubscriberTester, this, _1), false)
 {
	//Set Timer
	_testTimer = nh.createTimer(ros::Duration(0.3), &SubscriberTester::testTimer, this);
	_testTimer.stop();
	_testSubscriber.start();
	testStarted = false;
	received = false;
    actionTopic = actionName;
	topicName = name;
}

template <class MessageType>
bool SubscriberTester<MessageType>::performTest() {
	testStarted = true;
	received = false;
	counter = 0;
	ros::Publisher pub;
	pub = nh.advertise<MessageType>(topicName,100);
	ros::Rate q(1);
	q.sleep();
	
	
	std::string out = "Starting Subscriber Test on "+topicName;
	ROS_INFO ("%s",out.c_str());
	MessageType testMsg;
	testMsg.header.frame_id = "health";
	
	pub.publish(testMsg);
				
	ros::Rate w(1);
	w.sleep();
	return true;
}

template <class MessageType>
void SubscriberTester<MessageType>::testTimer(const ros::TimerEvent&)
{		
		
	//Test received	
	if (received){
		_testTimer.stop();
		ROS_INFO("Success on Subscribe Test on %s",topicName.c_str());
		_testSubscriber.setSucceeded();
		testStarted = false;
		received = false;
	} 
	else {
		counter++;
		if (counter > 10) {
			_testTimer.stop();
			ROS_ERROR("Failed to catch message on %s",topicName.c_str());
			_testSubscriber.setAborted();
			testStarted = false;
			received = false;
		}
	}	
}
  
template <class MessageType>
bool SubscriberTester<MessageType>::catchMsg(MessageType const &msg){
	
	if (msg.header.frame_id=="health") {
		if(testStarted){	
			received = true;
		}
		return true;
	};
	return false;
}  

template <class MessageType>
bool SubscriberTester<MessageType>::catchMsg(boost::shared_ptr<MessageType const> testMsg){
	
	if (testMsg->header.frame_id=="health") {
		if(testStarted){	
			received = true;
		}
		return true;
	};
	return false;
}

//callback
template <class MessageType>
void SubscriberTester<MessageType>::startSubscriberTester(const interface_tester::testerGoalConstPtr &testGoal)
{
	received = false;
	ROS_INFO("Received Test Goal");
	_testTimer.start();
	performTest();
	ros::Rate pause(10);
	while (testStarted)
		pause.sleep();
}

template <class MessageType>
bool SubscriberTester<MessageType>::startTest()
{
	performTest();
	
	//Wait
	ros::Rate pause(10);
	for (int i = 0; i <500; i++) {
		if (received || !ros::ok())
			break;
		ros::spinOnce();
		pause.sleep();
	}
	testStarted = false;
	
	//Test Received
	if (!received) {
		ROS_ERROR("Failed to catch message on %s",topicName.c_str());
		return false;
	}
	ROS_INFO("Success on Subscribe Test on %s",topicName.c_str());
	received = false;
	return true;
}
	
#endif

