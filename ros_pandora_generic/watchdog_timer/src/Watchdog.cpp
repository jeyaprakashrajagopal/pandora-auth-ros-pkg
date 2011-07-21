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
 * File Description <Watchdog Implementation>
 * Author: Allamanis M.
 * Date: 2 May 2011
 * Change History: -
 */
#include "watchdog_timer/Watchdog.h"

Watchdog::Watchdog(std::string wdtName, ros::Duration duration) {
	_name = ros::this_node::getName() + "/" + wdtName;
	_wdtCounter = duration;
	_started = false;
	
	_watchdogResetPublisher = _nh.advertise<watchdog_timer::watchdogResetMsg>("/robot/watchdog", 100);
	
	int retries = 0;
	while (!ros::service::waitForService("/robot/watchdogRegistry", ros::Duration(.2)) &&
			ros::ok()) {
		retries++;
		if (retries > 10)
			ROS_ERROR("Couldn't find service watchdogRegistry.");
		ros::spinOnce();
	}
	_registerService = _nh.serviceClient<watchdog_timer::watchdogSrv>("/robot/watchdogRegistry");
}

Watchdog::~Watchdog() {
	if (_started) {
		ROS_ERROR("Watchdog %s is being destructed, but has not been stopped",_name.c_str());
		stop();
	}
}

void Watchdog::start() {
	if (_started) return;
	watchdog_timer::watchdogSrv srv;
	srv.request.watchdogName = _name;
	srv.request.type = watchdog_timer::watchdogSrv::Request::TYPE_START;
	srv.request.timeoutDuration = _wdtCounter;
	if (_registerService.call(srv)) {
		_started = true;
		ROS_INFO("Watchdog started");
	} else {
		ROS_ERROR("Failed to contact watchdog service");
	}
}

void Watchdog::stop() {
	if (!_started) return;
	watchdog_timer::watchdogSrv srv;
	srv.request.watchdogName = _name;
	srv.request.type = watchdog_timer::watchdogSrv::Request::TYPE_STOP;
	if (_registerService.call(srv)) {
		_started = false;
		ROS_INFO("Watchdog %s stopped",_name.c_str());
	} else {
		ROS_ERROR("Failed to contact watchdog service for %s", _name.c_str());
	}	
}

void Watchdog::reset() {
		watchdog_timer::watchdogResetMsg msg;
		msg.watchdogName = _name;
		msg.header.stamp = ros::Time::now();
		_watchdogResetPublisher.publish(msg);
}
