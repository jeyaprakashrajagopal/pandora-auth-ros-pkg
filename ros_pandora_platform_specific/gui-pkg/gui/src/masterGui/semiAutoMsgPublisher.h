/**
  * semiAutoMsgPublisher client header files
  * Author: Dimitrios Vitsios
  * Date: 28 June 2011
  * Change History: 
*/

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "gui_communications/targetPosition.h"
#include <cstdlib>
#include <QtGui>
#include <vector>
using namespace std;

class semiAutoMsgPublisher : QWidget
{
	Q_OBJECT

	private:

		ros::NodeHandle _handle;
		ros::Publisher targetPosition_pub;
		gui_communications::targetPosition positionMsg;
		
		ros::CallbackQueue my_callback_queue;
		vector<int> selectedSensors;
		
	public:
		semiAutoMsgPublisher();
		
	public Q_SLOTS:
		void onConfirmButtonClicked();
};
