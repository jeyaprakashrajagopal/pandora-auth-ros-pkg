/**
  * irGui Module header files
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "controllersAndSensors_communications/irMsg.h"
#include <QtGui>

/**
* Class as a irGui subscriber node
*/
class irGui : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The irMsg subscriber
		ros::Subscriber _irSubscriber;	
		controllersAndSensors_communications::irMsg irMessage;
		ros::CallbackQueue my_callback_queue;
		QPicture pic;

	public:
		//irGui Constructor
		irGui();

		//A callback function to receive irMsg
		void irGuiReceiver(const controllersAndSensors_communications::irMsg &msg);

	public Q_SLOTS:
		void updateIRs();
};


