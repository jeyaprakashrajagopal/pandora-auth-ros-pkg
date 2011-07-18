/**
  * displayMap Module implementation files
  * Author: Dimitrios Vitsios
  * Date: 8 June 2011
  * Change History: - 
  */

#include "displayMap.h"
#include "masterGui.h"

extern QImage mapImage;
extern QMutex mapMutex;
extern int imageWidth, imageHeight;
extern int semiAutonomousStateFlag;
int widthFinal, heightFinal;


displayMap::displayMap(){

	mapsizeNav = gui->ui.mapLabelNav->size();
	mapsize = gui->ui.mapLabel->size();

	//establish semi-autonomous functionality
	connect(gui->ui.clearMousePressEventsPushButton, SIGNAL(clicked()), this, SLOT(onClearTextBrowserClicked()));
	targetPosition_pub = _handle.advertise<gui_communications::targetPosition>("/guiMsgs/semiAutonomous",1);


	_handle.setCallbackQueue(&my_callback_queue);
	my_callback_queue.callAvailable(ros::WallDuration());

	gui->ui.mapLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	gui->ui.mapLabel->installEventFilter(this);	

	QTimer *displayTimer = new QTimer(this);
     	connect(displayTimer, SIGNAL(timeout()), this, SLOT(displayMapTimer()), Qt::QueuedConnection); //Qt::DirectConnection);
     	displayTimer->start(1000);
		
}


void displayMap::displayMapTimer()
{

	widthFinalNav = imageWidth; 
	heightFinalNav = imageHeight;
	float globalRatioNav=1.0;

	widthFinal = widthFinalNav;
	heightFinal = heightFinalNav;
	float globalRatio=1.0;

	while(1){
		if(mapsizeNav.width()<widthFinal){
			widthAnNav=mapsizeNav.width()/(float)widthFinalNav;
			globalRatioNav*=widthAnNav;
			widthFinalNav=widthFinalNav*widthAnNav;
			heightFinalNav=heightFinalNav*widthAnNav;
		}
		else if(mapsizeNav.height()<heightFinalNav){
			heightAnNav=mapsizeNav.height()/(float)heightFinalNav;
			globalRatioNav*=heightAnNav;
			widthFinalNav=widthFinalNav*heightAnNav;
			heightFinalNav=heightFinalNav*heightAnNav;
		}
		if(mapsize.width()<widthFinal){
			widthAn=mapsize.width()/(float)widthFinal;
			globalRatio*=widthAn;
			widthFinal=widthFinal*widthAn;
			heightFinal=heightFinal*widthAn;
		}
		else if(mapsize.height()<heightFinal){
			heightAn=mapsize.height()/(float)heightFinal;
			globalRatio*=heightAn;
			widthFinal=widthFinal*heightAn;
			heightFinal=heightFinal*heightAn;
		}
		else break;
	}

	mapMutex.lock();
	if(gui->ui.mapNavGroupBox->isChecked())
		gui->ui.mapLabelNav->setPixmap(QPixmap().fromImage(mapImage.scaled(widthFinalNav,heightFinalNav)));
	if(gui->ui.mapGroupBox->isChecked())	
		gui->ui.mapLabel->setPixmap(QPixmap().fromImage(mapImage.scaled(widthFinal,heightFinal)));
	mapMutex.unlock();
}

bool displayMap::eventFilter(QObject *obj, QEvent *event)
{

	if(semiAutonomousStateFlag){
		if (event->type() == QEvent::MouseButtonPress)
		{
			QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
			targetX = mouseEvent->x();
			targetY = mouseEvent->y();
			//sending the x,y-coordinates of a mapLabel's point
			positionMsg.x = (float)targetX/imageWidth;
			positionMsg.y = (float)targetY/imageHeight;
			
			targetPosition_pub.publish(positionMsg); 
		

			gui->ui.mousePressEventTextBrowser->append("Received mouse press event");
			QString xPos  = QString::number(targetX);
			QString yPos  = QString::number(targetY);
			gui->ui.mousePressEventTextBrowser->append("x-position in map: ");
			gui->ui.mousePressEventTextBrowser->append(xPos);
			gui->ui.mousePressEventTextBrowser->append("y-position in map: ");
			gui->ui.mousePressEventTextBrowser->append(yPos);
			gui->ui.mousePressEventTextBrowser->append("--------------\n");
		}
	

		return QWidget::eventFilter(obj, event);
	}
	return false;
}

void displayMap::onClearTextBrowserClicked()
{
	gui->ui.mousePressEventTextBrowser->setText("");
}
