/**
  * debugText Module header files
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#define QT_NO_KEYWORDS
#include "signalslib.hpp"
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include <QtGui>
#include <QAction>

/**
* Class as a debugText subscriber node
*/
class debugText : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The rosout_agg msg subscriber
		ros::Subscriber _debugTextSubscriber;	
		rosgraph_msgs::Log globalMessage;

		int debugCnt;
		int infoCnt;
		int warnCnt;
		int errorCnt;
		int fatalCnt;
		int debugFlag;
		int infoFlag;
		int warnFlag;
		int errorFlag;
		int fatalFlag;

		
	public:
		//debugText Constructor
		debugText();

		//A callback function to receive rosout_agg msg
		void debugTextReceiver(const rosgraph_msgs::Log &msg);

	Q_SIGNALS:
		void enableDebugHandle();

	public Q_SLOTS:
	
		void onDebugClicked();	
		void onInfoClicked();
		void onWarnClicked();
		void onErrorClicked();
		void onFatalClicked();
		void onStartConsoleClicked();
		void onStopConsoleClicked();
		void onClearConsoleClicked();
		void handleMessage();
};


