#include "ros/ros.h"
#include "remote_counter/countingSrv.h"


class RemoteCounter{
	
		/**
		 * The ros nodeHandle 
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Actual counter
		 */
		int timesCalled;
		
		/**
		 * Service server
		 */
		ros::ServiceServer _counterServer;
		
		/**
		 * The timestamp that the last activity occured.
		 */
		ros::Time lastActivity;
		
		/**
		 * The counter name.
		 */
		std::string _name;
		
	public:
		/**
		* Constructor
		* @param The name of the node that calls the RemoteCounter
		*/
		RemoteCounter(std::string);
		
		/**
		 * The ROS Service callback for the RemoteCounter.
		 */
		bool counterCallback(remote_counter::countingSrv::Request &req, remote_counter::countingSrv::Response &res);
		
		/**
		 * Set the value stored in the counter
		 * @param value the value to set the counter at
		 * @param setTimestamp true to set the timestamp
		 */
		void setValue(int value, bool setTimestamp = false);
		
		/**
		 * Get the value stored in the counter
		 */
		int getValue();
		
		/**
		 * Returns the timestamp that the counter has been last modified.
		 */
		ros::Time getLastActivity();
		
		
	
	};
