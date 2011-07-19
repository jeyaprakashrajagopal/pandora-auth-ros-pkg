/**
  * displayVideo Module header files
  * Author: Dimitrios Vitsios
  * Date: 19 May 2011
  * Change History: - 
  */

#include <QtGui>
#include "ros/ros.h"

/**
* Class as an object for updating video streaming in GUI
*/
class displayVideo : QWidget
{
	Q_OBJECT

	public:
		//displayVideo Constructor
		displayVideo();

	public Q_SLOTS:
		void updateVideo();
};


