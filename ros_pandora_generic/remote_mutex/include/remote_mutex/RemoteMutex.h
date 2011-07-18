#include "ros/ros.h"
#include "remote_mutex/mutexSrv.h"
#ifndef REMOTE_MUTEX_H
#define REMOTE_MUTEX_H
/**
 * Impements a remote mutex. This is the client.
 */
class RemoteMutex {
	private:
		/**
		 * The ROS Node Handle.
		 */
		ros::NodeHandle _nh;
		
		/**
		 * The Client to the mutex serivce.
		 */
		 ros::ServiceClient _client;
		 
		 /**
		  * The Remote Mutex Name.
		  */
		 std::string _remoteMutexName;
		
	public:
		/**
		 * Constructor.
		 * @param name the name of the mutex.
		 */
		RemoteMutex(std::string name);
		
		/**
		 * Try to lock once the mutex. 
		 * @return true if the mutex was successfuly locked
		 */
		bool tryLock();
		
		/**
		 * Retrive the current status of the mutex.
		 */
		bool getStatus();
		
		/**
		 * Will try to lock the mutex for a give time. 
		 * @param timeout the timeout wait for locking.
		 * @return true if we have sucessfully locked the mutex
		 */
		bool lock(ros::Duration timeout = ros::Duration(0));
		
		/**
		 * Will unlock the mutex.
		 * @return true if the mutex was unlocked sucessfully. False otherwise.
		 */
		bool unlock();		
};
#endif
