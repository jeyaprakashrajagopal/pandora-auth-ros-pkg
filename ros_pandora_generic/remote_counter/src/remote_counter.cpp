
#include "remote_counter/remote_counter.h"


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

