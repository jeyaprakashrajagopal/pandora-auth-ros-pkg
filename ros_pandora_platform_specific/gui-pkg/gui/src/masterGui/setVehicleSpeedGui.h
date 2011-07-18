/**
  * setVehicleSpeedGui client header files
  * Author: Dimitrios Vitsios
  * Date: 14 March 2011
  * Change History: 
*/

#define QT_NO_KEYWORDS
#include "ros/ros.h"
#include "mainMotorControl_communications/setVehicleSpeedSrv.h"
#include <QtGui>
#include <QAction>
#include <QMutexLocker>


/**
* Class as a setVehicleSpeedSrv Client
*/
class setVehicleSpeedGui : public QWidget
{

	Q_OBJECT

	private:
		/**
		 * The ROS client handle
		 */
		ros::NodeHandle _handle;

		/**
		* The setVehicleSpeedSrv client
		*/
		ros::ServiceClient _setVehicleSpeedClient;
		mainMotorControl_communications::setVehicleSpeedSrv srv;	
		
		float curLinearSpeed;
		float prevLinearSpeed;
		float curRotationalSpeed;
		int testCnt;
		
	public:
		/**
		 * setVehicleSpeedGui Constructor
		 */
		setVehicleSpeedGui();
		

		QMutex mutex;
		
		void generateActions(void);
		QAction* forwardAction;
		QAction* backwardAction;
		QAction* leftAction;
		QAction* rightAction;
		QAction* brakeAction;	
		
	public Q_SLOTS:
		int onForwardClicked(void);
		int onBackwardClicked(void);
		int onRightClicked(void);
		int onLeftClicked(void);
		int forceStop(void);
		void setSpeed(void);
};
