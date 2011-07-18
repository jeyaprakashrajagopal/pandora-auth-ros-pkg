/**
  * semiAutonomousGui client header files
  * Author: Dimitrios Vitsios
  * Date: 15 March 2011
  * Change History: 
*/


#include <cstdlib>
#include <QtGui>
#include <QAction>


/**
* Class for the manual marking of a victim's position on the map
* by clicking on a specific point on the mapLabel in "Victim Identification" tab.
* It's also used to register the sensors that contributed to identify the victim.
*/
class semiAutonomousGui : public QWidget
{

	Q_OBJECT

	private:

		//auxiliary variables
		QSize mapsize;
		
	public:
		semiAutonomousGui();
		bool eventFilter(QObject *obj, QEvent *event);
		

	public Q_SLOTS:
		void onSensor1Selected(int index);
		void onSensor2Selected(int index);
		void onSensor3Selected(int index);
		void onSensor4Selected(int index);
		void onSensor5Selected(int index);
		void onSensor6Selected(int index);
		void onClearTextBrowserClicked();
};
