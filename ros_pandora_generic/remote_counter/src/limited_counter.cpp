#include "remote_counter/limited_counter.h"

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
