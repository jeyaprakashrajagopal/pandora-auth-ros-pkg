/**
  * co2Gui Module header files
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "controllersAndSensors_communications/co2Msg.h"
#include <QtGui>

/**
* Class as a irGui subscriber node
*/
class co2Gui : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The co2Msg subscriber
		ros::Subscriber _co2Subscriber;	
		controllersAndSensors_communications::co2Msg CO2Message;
		ros::CallbackQueue my_callback_queue;

	public:
		//co2Gui Constructor
		co2Gui();

		//A callback function to receive co2Msg
		void co2GuiReceiver(const controllersAndSensors_communications::co2Msg &msg);

	public Q_SLOTS:
		void updateCO2();
};


