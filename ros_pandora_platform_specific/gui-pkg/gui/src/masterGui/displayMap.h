/**
  * displayMap Module header files
  * Author: Dimitrios Vitsios
  * Date: 8 June 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "gui_communications/targetPosition.h"
#include <QtGui>

/**
* Class as a displayMap subscriber node
*/
class displayMap : QWidget
{
	Q_OBJECT

	private:
		ros::NodeHandle _handle;
		ros::Publisher targetPosition_pub;
		gui_communications::targetPosition positionMsg;
		ros::CallbackQueue my_callback_queue;
		QSize mapsizeNav;
		QSize mapsize;
		float widthAnNav,heightAnNav;
		int widthFinalNav,heightFinalNav;
		float widthAn,heightAn;
		int targetX;
		int targetY;
	public:
		//displayMap Constructor
		displayMap();
		bool eventFilter(QObject *obj, QEvent *event);

	public Q_SLOTS:
		void displayMapTimer();
		void onClearTextBrowserClicked();
};


