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
#ifndef MOCK_SUBSCRIBER_H
#define MOCK_SUBSCRIBER_H

#include "gmock/gmock.h"  
#include "gtest/gtest.h"
#include "ros/ros.h"


/**
 * An pure abstract interface to a subscriber.
 */
template <class MessageType>
class SimpleSubscriber {
	/**
	 * ROS Node Handle
	 */
	ros::NodeHandle _nh;	
	
	/**
	 * ROS Subscriber
	 */
	ros::Subscriber _subscriber;
	
	/**
	 * The name of the subscriber topic.
	 */
	std::string _topicName;
	
	public:	
	
	/**
	* The object constructor.
	* @param name the name of the topic that the tested subscriber
	* 		is listening.
	* Checks whether a subscriber catches 
	* the message has been published from a specific node
	*/
	SimpleSubscriber(std::string name);
	
	/**
	 * Destructor
	 */
    virtual ~SimpleSubscriber() {};
  
  /**
   * Abstract function to subscriber callback
   */
    virtual void subscriberActualCallback(boost::shared_ptr<MessageType const> a) =0; 

};


template <class MessageType>
SimpleSubscriber<MessageType>::SimpleSubscriber(std::string name) {
		_topicName = name;
		_subscriber = _nh.subscribe(_topicName,100,&SimpleSubscriber<MessageType>::subscriberActualCallback, this);
		
};

using::testing::Invoke;

/**
 * The Google Mock Object.
 */
template <class MessageType>
class MockSubscriber : public SimpleSubscriber<MessageType> {
	
	
 public:
	/**
	 * Constructor.
	 * @param name the topic name
	 */
	MockSubscriber(std::string name):SimpleSubscriber<MessageType>(name){};
 
  MOCK_METHOD1_T(subscriberActualCallback, void(boost::shared_ptr<MessageType const> a));
};

#endif

