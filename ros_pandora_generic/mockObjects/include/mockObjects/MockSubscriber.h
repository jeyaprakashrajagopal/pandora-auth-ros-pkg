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

