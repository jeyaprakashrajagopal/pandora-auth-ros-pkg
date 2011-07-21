/**
  * butterflyGui Module header files
  * Author: Dimitrios Vitsios
  * Date: 11 April 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include "controllersAndSensors_communications/butterflyMsg.h"
#include <QtGui>

/**
* Class as a butterflyGui subscriber node
*/
class butterflyGui : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The butterflyMsg subscriber
		ros::Subscriber _butterflySubscriber;	
		controllersAndSensors_communications::butterflyMsg butterflyMessage;

		int motorsPercentage;
		int psuPercentage;
		int BATTERIES_RANGE;
		QMutex mutex;
		long lowVoltageCounter;

	public:
		//butterflyGui Constructor
		butterflyGui();

		//A callback function to receive butterflyMsg
		void butterflyGuiReceiver(const controllersAndSensors_communications::butterflyMsg &msg);

	public Q_SLOTS:
		void updateButterfly();
		void updateProgressBars();
};


