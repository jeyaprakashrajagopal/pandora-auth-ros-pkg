/**
  * sonarGui Module header files
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "controllersAndSensors_communications/sonarMsg.h"
#include <QtGui>

/**
* Class as a sonarGui subscriber node
*/
class sonarGui : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The sonarMsg subscriber
		ros::Subscriber _sonarSubscriber;	
		controllersAndSensors_communications::sonarMsg sonarMessage;
		ros::CallbackQueue my_callback_queue;
		
	public:
		//sonarGui Constructor
		sonarGui();

		//A callback function to receive sonarMsg
		void sonarGuiReceiver(const controllersAndSensors_communications::sonarMsg &msg);

	public Q_SLOTS:
		void updateSonars();
};


