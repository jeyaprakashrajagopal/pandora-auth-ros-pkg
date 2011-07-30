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

#include "remote_counter/RemoteCounter.h"


RemoteCounter::RemoteCounter(std::string counterName) {
	
	_name = "/counter/" + counterName;
	_counterServer = _nh.advertiseService(_name, &RemoteCounter::counterCallback, this);
	timesCalled = 0;
}

bool RemoteCounter::counterCallback(remote_counter::countingSrv::Request &req, remote_counter::countingSrv::Response &res){
	timesCalled += req.count;
	lastActivity = ros::Time::now();
	ROS_INFO("Counter changed by %s! New value is %i ",(*req.__connection_header)["callerid"].c_str(), timesCalled);
	return true;
}

int RemoteCounter::getValue() {
	return timesCalled;
	}
	
void RemoteCounter::setValue(int value, bool setTimestamp) {
	timesCalled = value;
	if (setTimestamp)
		lastActivity = ros::Time::now();
	}
	
ros::Time RemoteCounter::getLastActivity() {
	return lastActivity;
}

