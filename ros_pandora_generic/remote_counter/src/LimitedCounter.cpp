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
#include "remote_counter/LimitedCounter.h"

LimitedCounter::LimitedCounter(std::string name, int max, int activeSecs): 
				_updater(), 
				counter(name), 
				secondsConsideredActive(activeSecs) {
	_updater.setHardwareID("none");
	maxValue = max;
	minValue = 0;
	_name = name;
	_timer = _nh.createTimer(ros::Duration(.5), &LimitedCounter::postValues, this);
	_updater.add("Counter Value ", this, &LimitedCounter::valueDiagnostic);
};

void LimitedCounter::valueDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
	bool active = ((ros::Time::now() - counter.getLastActivity()) < secondsConsideredActive);
	if (counter.getValue() > maxValue) {
		if ((ros::Time::now() - counter.getLastActivity()) > ros::Duration(5)) {
			counter.setValue(0);
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "There are lost messages. Reseting counter");
		} else
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "There are lost messages");
	}
	else if (counter.getValue() < minValue) {
		if ((ros::Time::now() - counter.getLastActivity()) > ros::Duration(5)) {
			counter.setValue(0);
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid calls. Reseting counter.");
		} else	
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid calls");
	}
	else {
		if (active) {
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Active. No lost messages.");
			stat.add("New Event: ", _name);
		}
		else {
			if ((ros::Time::now() - counter.getLastActivity()) > ros::Duration(5)) {
				counter.setValue(0);
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Inactive. Counter value turned to zero.");
			} else {
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Inactive. No lost messages");
			}
		}	
	}
	stat.add("Counter Value ", counter.getValue());
	stat.add("Max Value Accepted ", maxValue);
	
	stat.add("Counter Active", active);
}

void LimitedCounter::postValues(const ros::TimerEvent& event) {
	_updater.update();
}
