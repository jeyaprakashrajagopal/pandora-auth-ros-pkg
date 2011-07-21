/**
  * displayVideo Module implementation
  * Author: Dimitrios Vitsios
  * Date: 19 May 2011
  * Change History: - 
  */

#include "displayVideo.h"
#include "masterGui.h"

extern QMutex videoMutex;
extern QImage videoDisplay;

displayVideo::displayVideo()
{
	QTimer *videoTimer= new QTimer(this);
	connect(videoTimer, SIGNAL(timeout()), this, SLOT(updateVideo()));
	videoTimer->start(200);
}

void displayVideo::updateVideo()
{
	if(gui->ui.groupBox->isChecked()){
		ROS_INFO("Setting pixmap in video label");		
		videoMutex.lock();
		gui->ui.videoLabel->setPixmap(QPixmap().fromImage(videoDisplay));
		videoMutex.unlock();
	}
}
