/**
  * stateManagerGui header files
  * Author: Dimitrios Vitsios
  * Date: 18 April 2011
  * Change History: 
*/

#define QT_NO_KEYWORDS
#include "ros/ros.h"
#include "StateClient.h"
#include <string>
#include <QtGui>
#include <QAction>
#include <QMutexLocker>
#include <time.h>
#include "dataFusion_communications/fusionGlobalMsg.h"



/**
* Class as a stateManagerGui Client
*/
class stateManagerGui : public QWidget, StateClient
{

	Q_OBJECT

	private:
		/**
		 * The ROS client handle
		 */
		ros::NodeHandle _handle;
		ros::Subscriber _testSensorsSubscriber;
		std::string MODES[9];
		int curState;
		int transState;
		int globalCnt;
		int timer;
		QAction *forceModeOffAction;
		float mlx, co2, sound, skin, motion, face;
		
		dataFusion_communications::fusionGlobalMsg msg;
	public:
		/**
		 * stateManagerGui Constructor
		 */
		stateManagerGui();

		void generateActions(void);
		void startTransition(int newState);
		void completeTransition();

		QMutex mutex;
		
		void testSensorsReceiver(const dataFusion_communications::fusionGlobalMsg &sensorsMsg);

	public Q_SLOTS:
		int changeState(int index);
		int changeArmState(int index);
		void displayElapsedTime();
		void onModeOffClicked();
		void onClearPandoraTextBrClicked();
		void displaySensorsValues();
};
