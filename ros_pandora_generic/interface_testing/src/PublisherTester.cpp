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
template <class MessageType>
PublisherTester<MessageType>::PublisherTester(std::string name, std::string actionName) : 
	_testPublisher(nh, "/test/" +actionName, boost::bind(&PublisherTester<MessageType>::startPublisherTester, this, _1), false)
{
	//Set Timer
	_testTimer = nh.createTimer(ros::Duration(0.3), &PublisherTester::testTimer, this);
	_testTimer.stop();
	_testPublisher.start();
	//Set Values
	testStarted = false;
	received = false;
    actionTopic = actionName;
	topicName = name;
}

template <class MessageType>
bool PublisherTester<MessageType>::startTest() 
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

template <class MessageType>
bool PublisherTester<MessageType>::performTest() {
	testStarted = true;
	received = false;
	counter = 0;
	sub = nh.subscribe(topicName, 1000, &PublisherTester<MessageType>::catchMsg, this);
	ros::Rate pause(10);
	
	std::string out = "Starting Publisher Test on "+topicName;
	ROS_INFO("%s",out.c_str());	
	while (testStarted)
		pause.sleep();
		
	return true;
}

template <class MessageType>
void PublisherTester<MessageType>::testTimer(const ros::TimerEvent&)
{		
	//Test received	
	if (received){
		_testTimer.stop();
		ROS_INFO("Success on Subscribe Test on %s",topicName.c_str());
		_testPublisher.setSucceeded();
		testStarted = false;
		received = false;
	} 
	else {
		counter++;
		if (counter > 10) {
			_testTimer.stop();
			ROS_ERROR("Failed to catch message on %s",topicName.c_str());
			_testPublisher.setAborted();
			testStarted = false;
			received = false;
		}
	}	
}

template <class MessageType>
void PublisherTester<MessageType>::catchMsg(boost::shared_ptr<MessageType const> testMsg) {
	if (testMsg->header.frame_id=="health") {
		if(testStarted) {
			received = true;
		}
	}
}

template <class MessageType>
void PublisherTester<MessageType>::checkoutMsg(MessageType& testMsg) {
	if (testStarted)
		testMsg.header.frame_id = "health";
}

template <class MessageType>
void PublisherTester<MessageType>::startPublisherTester(const interface_tester::testerGoalConstPtr &testGoal)
{
	ROS_INFO("Received Test Goal");	
	_testTimer.start();
	performTest();
	while(_testPublisher.isActive()) {
		ros::Rate w(1);
		w.sleep();
	}
}


