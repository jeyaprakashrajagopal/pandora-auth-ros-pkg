/**
  * co2Gui Module implementation
  * Author: Dimitrios Vitsios
  * Date: 16 March 2011
  * Change History: - 
  */

#include "co2Gui.h"
#include "masterGui.h"

co2Gui::co2Gui()
{
	
     //Declare subscribers
     _co2Subscriber = _handle.subscribe("/gui/sensors/co2", 1 ,&co2Gui::co2GuiReceiver,this);

	_handle.setCallbackQueue(&my_callback_queue);
	my_callback_queue.callAvailable(ros::WallDuration());

	QTimer *CO2Timer= new QTimer(this);
     	connect(CO2Timer, SIGNAL(timeout()), this, SLOT(updateCO2()));
     	CO2Timer->start(1000);
}


void co2Gui::co2GuiReceiver(const controllersAndSensors_communications::co2Msg &msg)
{	
	CO2Message = msg;
}

void co2Gui::updateCO2()
{
	gui->ui.CO2Lcd->display(int(CO2Message.ppm));
}
