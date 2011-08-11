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
#include "ros/ros.h"
#include "RemoteCounter.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
	/**
	* This class implements a limited counter that can indicate the loss of messages or invalid calls
	*/
class LimitedCounter {
	
	ros::NodeHandle _nh;
	RemoteCounter counter;
	std::string _name;
	
	/**
	 * The updater for posting diagnostics to monitor
	 */
	diagnostic_updater::Updater _updater;
	
	/**
	 * Max & min values accepted
	 */
	int minValue, maxValue;
	
	/**
	 * The duration that the counter will be considered active.
	 */
	 ros::Duration secondsConsideredActive;
	
	/**
	 * The ros timer
	 */
	ros::Timer _timer;
	
		
	public:
		/**
		 * Constructor
		 * @param name the name of the counter
		 * @param max the max value of counter that the counter will be considered OK.
		 * @param activeSecs the num of secs that the counter will be considered active since an update.
		 */
		LimitedCounter(std::string name, int max, int activeSecs);
		
		/**
		 * For posting the diagnostics
		 */
		void valueDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
		void eventDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
	
		/**
		 * Timer to periodically call the updater.
		 */
		void postValues(const ros::TimerEvent& event);
	};
