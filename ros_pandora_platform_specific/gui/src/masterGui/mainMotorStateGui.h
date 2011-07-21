/**
  * mainMotorStateGui node header files
  * Author: Dimitrios Vitsios
  * Date: 10 March 2011
  * Change History: 
*/

#include "ros/ros.h"
#include "mainMotorControl_communications/mainMotorStateMsg.h"
#include <QtGui>

/**
* Class as a mainMotorState subscriber node
*/
class mainMotorStateGui : QWidget
{
	Q_OBJECT

	private:
		/**
		 * The ROS Node handle
		 */
		ros::NodeHandle _handle;

		/**
		* The mainMotorStateMsg subscriber
		*/
		ros::Subscriber _mainMotorStateSubscriber;

		mainMotorControl_communications::mainMotorStateMsg mainMotStMessage;
	public:
		/**
		 * mainMotorStateGui Constructor
		 */
		mainMotorStateGui();

		/**
		 * A callback function to receive mainMotorStateMsg
		 */
		void mainMotorStateReceiver(const mainMotorControl_communications::mainMotorStateMsg &msg);

	public Q_SLOTS:	
		void updateMotorStatus();
};
