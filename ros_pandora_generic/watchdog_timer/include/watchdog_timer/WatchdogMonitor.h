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
 * File Description <Watchdog Monitor Header File>
 * Author: Allamanis M.
 * Date: 2 May 2011
 * Change History: -
 */

#ifndef WATCHDOG_MONITOR_H
#define WATCHDOG_MONITOR_H


#include "ros/ros.h"
#include "watchdog_timer/watchdogSrv.h"
#include "watchdog_timer/watchdogResetMsg.h"
#include "watchdog_timer/wdtTimeoutNotificationMsg.h"

struct wdtInformation {
	ros::Duration wdtDuration;
	ros::Time lastReset;	
};

/**
 * A class implementing the watchdog monitor
 */
class WatchdogMonitor {
	
	private:
		/**
		 * The ROS node handle.
		 */
		ros::NodeHandle _nh;
	
		/**
		 * A subscriber to the watchdog reset topic.
		 */
		ros::Subscriber _resetSubscriber;
		
		/**
		 * A publisher for the timeouts.
		 */
		ros::Publisher _timeoutsPublisher;
		
		/**
		 * A service for registering WDT.
		 */
		ros::ServiceServer _registerWDTService;
		
		/**
		 * A map for storing the wdt information.
		 */
		std::map<std::string, wdtInformation> _activeWdt;
		
		/**
		 * The actual timer watching the wdts
		 */
		ros::Timer _wdtTimer;		
		
		/**
		 * The timer callback.
		 */
		void checkForTimeouts(const ros::TimerEvent&);
		
		/**
		 * Register the WDT service callback
		 */
		bool registerWdtService(watchdog_timer::watchdogSrv::Request&, watchdog_timer::watchdogSrv::Response&);
		
		/**
		 * Callback for receiving resets.
		 */
		void receiveReset(const watchdog_timer::watchdogResetMsgConstPtr&);
		
	public:
		/**
		 * Costructor
		 * @param checkFrequency the frequency to check for resets
		 */
		WatchdogMonitor(float checkFrequency);
};

#endif
