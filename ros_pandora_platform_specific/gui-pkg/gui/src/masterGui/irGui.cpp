/**
  * irGui Module implementation
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#include "irGui.h"
#include "masterGui.h"

irGui::irGui()
{
	
     //Declare subscribers
     _irSubscriber = _handle.subscribe("/gui/sensors/ir", 1 ,&irGui::irGuiReceiver,this);
	
	_handle.setCallbackQueue(&my_callback_queue);
	my_callback_queue.callAvailable(ros::WallDuration());

	// (-1) means out of range.
	gui->ui.irLCDFront->display(-1);
	gui->ui.irLCDLeft->display(-1);
	gui->ui.irLCDRight->display(-1);
	gui->ui.irLCDRear->display(-1);
	gui->ui.irLCDArm->display(-1);
	

	QTimer *irTimer= new QTimer(this);
	connect(irTimer, SIGNAL(timeout()), this, SLOT(updateIRs()));
	irTimer->start(1000);
}

/**
-- in irMsg:
int32[6] distance -> corresponding sides/places:

					  (bounds) [cm]
- distance[0] : front   (10,80) 
- distance[1] : right   (4,30) 
- distance[2] : left    (4,30) 
- distance[3] : back    (10,80)
- distance[4] : arm head
*/

void irGui::irGuiReceiver(const controllersAndSensors_communications::irMsg &msg)
{	
	//ROS_INFO("IR msg received");	
	irMessage = msg;
}

void irGui::updateIRs()
{
	//display distance from IRs in cm
	if(irMessage.distance[0] == -1)
		gui->ui.irLCDFront->display(-1);
	else
		gui->ui.irLCDFront->display((irMessage.distance[0])/10);
		
	if(irMessage.distance[1] == -1)
		gui->ui.irLCDRight->display(-1);
	else
		gui->ui.irLCDRight->display((irMessage.distance[1])/10);
		
	if(irMessage.distance[2] == -1)
		gui->ui.irLCDLeft->display(-1);
	else	
		gui->ui.irLCDLeft->display((irMessage.distance[2])/10);
		
	if(irMessage.distance[3] == -1)
		gui->ui.irLCDRear->display(-1);
	else
		gui->ui.irLCDRear->display((irMessage.distance[3])/10);
		
	if(irMessage.distance[4] == -1)	
		gui->ui.irLCDArm->display(-1);
	else
		gui->ui.irLCDArm->display((irMessage.distance[4])/10);
}
