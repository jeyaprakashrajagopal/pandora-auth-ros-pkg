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
/** 
 * File Description <Watchdog Monitor Implementation>
 * Author: Allamanis M.
 * Date: 2 May 2011
 * Change History: -
 */
#include "watchdog_timer/WatchdogMonitor.h"

WatchdogMonitor::WatchdogMonitor(float checkFrequency) {
	_resetSubscriber = _nh.subscribe("/robot/watchdog",1000,&WatchdogMonitor::receiveReset,this);
	_registerWDTService = _nh.advertiseService("/robot/watchdogRegistry",&WatchdogMonitor::registerWdtService,this);
	_timeoutsPublisher = _nh.advertise<watchdog_timer::wdtTimeoutNotificationMsg>("/robot/watchdogResets",1000);
	_wdtTimer = _nh.createTimer(ros::Duration(checkFrequency),&WatchdogMonitor::checkForTimeouts,this);
}

void WatchdogMonitor::checkForTimeouts(const ros::TimerEvent&) {
	std::map<std::string, wdtInformation>::iterator currentWdt;
	for (currentWdt=_activeWdt.begin() ; currentWdt != _activeWdt.end(); currentWdt++ ) {
		wdtInformation info = (*currentWdt).second;
		if (info.lastReset + info.wdtDuration < ros::Time::now()) {
			ROS_ERROR("Watchdog %s timedout",(*currentWdt).first.c_str());
			watchdog_timer::wdtTimeoutNotificationMsg msg;
			msg.lastReset = info.lastReset;
			msg.wdtName = (*currentWdt).first;
			msg.nodeName = msg.wdtName.substr(0,msg.wdtName.rfind("/"));
			_timeoutsPublisher.publish(msg);
		}
	}
	
}

bool WatchdogMonitor::registerWdtService(watchdog_timer::watchdogSrv::Request& rq, watchdog_timer::watchdogSrv::Response&) {
	if (rq.type == watchdog_timer::watchdogSrv::Request::TYPE_START) {
		if (_activeWdt.count(rq.watchdogName) > 0) return false; //WDT already exists
		wdtInformation info;
		info.wdtDuration = rq.timeoutDuration;
		info.lastReset = ros::Time::now();
		_activeWdt[rq.watchdogName] = info;
		return true;
	} else {
		std::map<std::string, wdtInformation>::iterator wdtToBeStopped;
		wdtToBeStopped = _activeWdt.find(rq.watchdogName);
		if (_activeWdt.count(rq.watchdogName) == 0) return false; //WDT does not exist
		_activeWdt.erase(wdtToBeStopped);		
		return true;
	}
	
}

void WatchdogMonitor::receiveReset(const watchdog_timer::watchdogResetMsgConstPtr& msg) {
	_activeWdt[msg->watchdogName].lastReset = msg->header.stamp;
}

int main(int argc, char **argv) {
	ros::init(argc,argv,"watchdogMonitor");
	
	WatchdogMonitor wdt(1);
	ros::spin();
	
}
