#include "remote_mutex/RemoteMutex.h"

RemoteMutex::RemoteMutex(std::string name) {
	_remoteMutexName = "/mutexes/"+name;
	
	int retries = 0;
	while (!ros::service::waitForService(_remoteMutexName, ros::Duration(.1)) && ros::ok()) {
		retries++;
		if (retries > 10) 
			ROS_ERROR("[RemoteMutex] Could not find %s",_remoteMutexName.c_str());
		ros::spinOnce();
	}
	
	_client = _nh.serviceClient<remote_mutex::mutexSrv>(_remoteMutexName);
	
}

bool RemoteMutex::getStatus() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_POLL;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_LOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::tryLock() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_LOCK;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_LOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::unlock() {
	remote_mutex::mutexSrv srv;
	
	srv.request.requestor =  ros::this_node::getName();
	srv.request.requestType = remote_mutex::mutexSrv::Request::TYPE_UNLOCK;
	if (_client.call(srv)) {
		return srv.response.status == remote_mutex::mutexSrv::Response::STATUS_UNLOCKED;
	} else {
		ROS_ERROR("Failed to call mutex %s service",_remoteMutexName.c_str());
		return false;
	}
}

bool RemoteMutex::lock(ros::Duration timeout) {
	ros::Time start = ros::Time::now();
	
	while(ros::Time::now() < start + timeout || timeout == ros::Duration(0)) {		
		if (tryLock()) {
			ROS_INFO("Locked sucessfully");
			return true;
		}
		ros::spinOnce();
		if (!ros::ok()) return false;
	}
	
	return false;
}
