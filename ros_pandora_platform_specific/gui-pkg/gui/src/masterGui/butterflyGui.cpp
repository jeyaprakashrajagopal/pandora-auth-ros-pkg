/**
  * butterflyGui Module implementation
  * Author: Dimitrios Vitsios
  * Date: 11 April 2011
  * Change History: - 
  */

#include "butterflyGui.h"
#include "masterGui.h"

butterflyGui::butterflyGui()
{
	
	//Declare subscribers
	_butterflySubscriber = _handle.subscribe("/gui/sensors/butterfly", 1 ,&butterflyGui::butterflyGuiReceiver,this);

	lowVoltageCounter = 0;
	gui->ui.lowVoltageLabel->setText("");
	gui->ui.lowVoltageLabel_2->setText("");

	QTimer *butterflyTimer= new QTimer(this);
	connect(butterflyTimer, SIGNAL(timeout()), this, SLOT(updateButterfly()));
	butterflyTimer->start(1000);

	QTimer *barsTimer= new QTimer(this);
	connect(barsTimer, SIGNAL(timeout()), this, SLOT(updateProgressBars()));
	barsTimer->start(1000);

	BATTERIES_RANGE = 5;
}

void butterflyGui::butterflyGuiReceiver(const controllersAndSensors_communications::butterflyMsg &msg)
{	
	//ROS_INFO("Butterfly msg received");
	mutex.lock();
	butterflyMessage = msg;
	mutex.unlock();
}

void butterflyGui::updateButterfly()
{
	gui->ui.motorsVoltage->display( butterflyMessage.voltage[0] );
	gui->ui.psuVoltage->display( butterflyMessage.voltage[1] );
	lowVoltageCounter++;
	if(lowVoltageCounter > 8){ 
		if( ((int)butterflyMessage.voltage[0] < 21) || ((int)butterflyMessage.voltage[1] < 21) ){
			if(lowVoltageCounter%2){
				gui->ui.lowVoltageLabel->setText("LOW VOLTAGE!!!");
				gui->ui.lowVoltageLabel_2->setText("LOW VOLTAGE!!!");
			}
			else{
				gui->ui.lowVoltageLabel->setText("");
				gui->ui.lowVoltageLabel_2->setText("");
			}
		}
	}
}

void butterflyGui::updateProgressBars()
{
	motorsPercentage = (butterflyMessage.voltage[0]-20)/BATTERIES_RANGE*100;
	gui->ui.MotorsBatteryProgressBar->setValue(motorsPercentage);

	psuPercentage = (butterflyMessage.voltage[1]-20)/BATTERIES_RANGE*100;
	gui->ui.psuBatteryProgressBar->setValue(psuPercentage);
}
