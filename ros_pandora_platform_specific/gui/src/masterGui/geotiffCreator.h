/**
  * geotiffCreator header files
  * Author: Dimitrios Vitsios
  * Date: 24 June 2011
  * Change History: 
*/

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "navigation_communications/geotiffSrv.h"
#include <QtGui>
#include <QAction>

/**
 * Class as a "geotiff painter" that creates a geotiff-formatted image 
 * conforming to the rules and the standards of the Robocup - Rescue Robot League Competition.
 */
class geotiffCreator : public QWidget
{

	Q_OBJECT

	private: 
		ros::NodeHandle _handle;
		ros::ServiceClient _geotiffServiceClient;
		navigation_communications::geotiffSrv srv;
		QString missionName;
		
		uchar *map;
		uchar *coverage;
		int xsize;
		int ysize;
		int *pathx;
		int *pathy;
		int *victimsx;
		int *victimsy;
		int *hazmatx;
		int *hazmaty;
		int *hazmatType;
		
		int pathSize;
		int victimsSize;
		int hazmatSize;

	public:
		geotiffCreator();
	
	public Q_SLOTS:
		void onCreateGeotiffClicked();
		void saveMissionName(const QString & textInserted);
};
