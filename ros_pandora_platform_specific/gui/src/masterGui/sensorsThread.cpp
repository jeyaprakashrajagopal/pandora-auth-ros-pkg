/**
  * sensorsThread implementation files
  * Author: Dimitrios Vitsios
  * Date: 8 May 2011
  * Change History: 
*/

#include "sensorsThread.h"
#include "mainMotorStateGui.h"
#include "irGui.h"
#include "sonarGui.h"
#include "co2Gui.h"
#include "thermalGui.h"
#include "butterflyGui.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>

void sensorsThread::run()
{
	mainMotorStateGui mmsGui;
	irGui ir;
	sonarGui sonar;
	co2Gui co2;
	exec();
}


