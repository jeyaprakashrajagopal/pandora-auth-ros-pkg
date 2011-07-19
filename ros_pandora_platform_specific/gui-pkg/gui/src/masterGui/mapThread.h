/**
  * mapThread header files
  * Author: Dimitrios Vitsios
  * Date: 9 June 2011
  * Change History: - 
  */
  
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "navigation_communications/guiInfoLargeSrv.h"
#include "navigation_communications/guiInfoSmallSrv.h"
#include "gui_communications/targetPosition.h"
#include <QtGui>


/*
 * Class for acquiring map data from corresponding services
 * and for map drawing with Qt painters.
 * -- deprecated.
 */ 
class mapThread : public QThread
{

	private:

		ros::NodeHandle _handle;
		ros::ServiceClient _guiInfoLargeClient;	
		ros::ServiceClient _guiInfoSmallClient;
		ros::Publisher targetPosition_pub;
		gui_communications::targetPosition positionMsg;
		ros::CallbackQueue my_callback_queue;
		
		navigation_communications::guiInfoLargeSrv srv;	
		navigation_communications::guiInfoSmallSrv srvSmall;

		//read from services
		uchar *map;
		uchar *coverage;
		uchar *voronoi;
		int xsize;
		int ysize;
		int xRobot;
		int yRobot;
		int *coverageLimitsx;
		int *coverageLimitsy;
		int *voronodesx;
		int *voronodesy;
		int *neighboursFirst;
		int *neighboursLast;
		int *goalsx;
		int *goalsy;

		//auxiliary variables
		float widthAnNav,heightAnNav;
		float widthAn,heightAn;
		int covLimitSize;
		int voronodesSize;
		int neighboursSize;
		int goalsxSize;
		int goalsySize;
		float angleRobot;
		int targetX;
		int targetY;
		int xAnglePoint;
		int yAnglePoint;
		QSize mapsize;
		QSize mapsizeNav;

		QPainter painter;
		QMutex mutex;

		int drawMapSmallFlag;
		int drawMapFlag;
		

	public:
		mapThread();
		void run();
		void generateMap(); 
		void generateMapSmall(); 
		void drawMap();
		void drawMapSmall();

};
