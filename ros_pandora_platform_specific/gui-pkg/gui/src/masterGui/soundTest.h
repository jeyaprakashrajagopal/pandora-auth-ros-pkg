/**
  * soundTest Module header files
  * Author: Dimitrios Vitsios
  * Date: 28 June 2011
  * Change History: - 
  */

#include "ros/ros.h"
#include "controllersAndSensors_communications/soundExistenceMsg.h"
#include <QtGui>

/**
* Class as a soundTest subscriber node
*/
class soundTest : QWidget
{
	Q_OBJECT

	private:
		//The ROS Node handle 
		ros::NodeHandle _handle;

		//The soundExistenceMsg subscriber
		ros::Subscriber _soundSubscriber;	
		controllersAndSensors_communications::soundExistenceMsg msg;

		bool soundExistence;
		float soundDirection;
		int soundExistPercentage;
		int soundDirPercentage;
		QMutex mutex;
		
	public:
		soundTest();

		void soundReceiver(const controllersAndSensors_communications::soundExistenceMsg &soundMessage);

	public Q_SLOTS:
	    void updateSoundIndicator();
		void testSoundService();
	
};


