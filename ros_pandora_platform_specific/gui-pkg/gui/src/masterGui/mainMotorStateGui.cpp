/**
  * mainMotorStateGui node implementation files
  * Author: Dimitrios Vitsios
  * Date: 10 March 2011
  * Change History: 
*/

#include "mainMotorStateGui.h"
#include "masterGui.h"

extern int teleoperatedStateFlag;

mainMotorStateGui::mainMotorStateGui(){

	//Declare subscribers
	_mainMotorStateSubscriber = _handle.subscribe("/mainMotorControl/status",1,&mainMotorStateGui::mainMotorStateReceiver,this);

	//_handle.setCallbackQueue(&my_callback_queue);
	//my_callback_queue.callAvailable(ros::WallDuration());

	QTimer *motorStatusTimer= new QTimer(this);
	connect(motorStatusTimer, SIGNAL(timeout()), this, SLOT(updateMotorStatus()));
	motorStatusTimer->start(1000);
	
}


void mainMotorStateGui::mainMotorStateReceiver(const mainMotorControl_communications::mainMotorStateMsg &msg){
	
	//ROS_INFO("Hi I got the message %i",msg.velocityLeft);
	mainMotStMessage = msg;
}

void mainMotorStateGui::updateMotorStatus()
{	
	//displaying in navigation tab
	if(!teleoperatedStateFlag){
		gui->ui.motorLinearLCD->display(mainMotStMessage.velocityLinear);
		gui->ui.motorAngularLCD->display(mainMotStMessage.velocityAngular);
	}
	
	//displaying in debug tab
	gui->ui.debugMotorLinearLCD->display(mainMotStMessage.velocityLinear);
	gui->ui.debugMotorAngularLCD->display(mainMotStMessage.velocityAngular);
	gui->ui.motorLeftRpm->display(mainMotStMessage.velocityLeft);
	gui->ui.motorRightRpm->display(mainMotStMessage.velocityRight);
}

