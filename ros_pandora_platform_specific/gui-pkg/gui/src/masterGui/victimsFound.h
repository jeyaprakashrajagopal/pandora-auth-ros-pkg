/**
  * victimsFound Module header files
  * Author: Dimitrios Vitsios
  * Date: 27 April 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include "navigation_communications/victimsMsg.h"
#include "navigation_communications/victimIdentificationSrv.h"
#include <QtGui>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

/**
* Class as a victimsFound subscriber node
*/
class victimsFound : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//subscriber
		ros::Subscriber _victimsFoundSubscriber;	
		navigation_communications::victimsMsg victimsFoundMessage;
		ros::ServiceClient _acceptDeclineVictimClient;
		navigation_communications::victimIdentificationSrv srv;

		int victimsCounter;
		int *victimsx;
		int *victimsy;
		char *sensorsString;
		

	public:
		//Constructor
		victimsFound();

		//callback function
		void victimsFoundReceiver(const navigation_communications::victimsMsg &msg);

	Q_SIGNALS:
		void enableHandle();

	public Q_SLOTS:
		void handleVictimsFoundMessage();
		int onAcceptClicked();
		int onDeclineClicked();

};


