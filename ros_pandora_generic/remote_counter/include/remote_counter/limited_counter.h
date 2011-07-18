#include "ros/ros.h"
#include "remote_counter.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"

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
