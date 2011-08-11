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
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "ros/ros.h"
#include "watchdog_timer/watchdogSrv.h"
#include "watchdog_timer/watchdogResetMsg.h"



/**
 * A watchdog class for creating a watchdog.
 */
class Watchdog {
	private: 
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Watchdog Reset Publisher.
		 */
		ros::Publisher _watchdogResetPublisher;
		
		/**
		 * Service for starting and stopping watchdog.
		 */
		ros::ServiceClient _registerService;
	
		/**
		 * The name of the watchdog.
		 */
		std::string _name;
		
		/**
		 * The duration to count before reseting the watchdog.
		 */
		ros::Duration _wdtCounter;
		
		/**
		 * A boolean variable indicating if the wdt is enabled.
		 */
		bool _started;
		
		/**
		 * ROS Service Server
		 */
		ros::ServiceServer _timeoutSrv;

	public:
		/**
		 * Constructor.
		 * @param watchDogName the name of the watchdog
		 * @param duration the duration of the timer until reset
		 */
		Watchdog(std::string watchDogName, ros::Duration duration);

		/**
		 * Destructor for throwing error if watchdog is being destructed but
		 * watchdog is still running.
		 */
		~Watchdog();
		
		/**
		 * Reset the watchdog and start timing again.
		 */
		void reset();
	
		/**
		 * Starting the wdt.
		 */
		void start();
		
		/**
		 * Stop the wdt timer.
		 */
		void stop();
};

#endif
