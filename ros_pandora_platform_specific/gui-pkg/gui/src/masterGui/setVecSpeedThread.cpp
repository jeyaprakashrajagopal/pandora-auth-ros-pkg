/**
  * setVecSpeedThread implementation files
  * Author: Dimitrios Vitsios
  * Date: 15 May 2011
  * Change History: 
*/

#include "setVecSpeedThread.h"
#include "setVehicleSpeedGui.h"

void setVecSpeedThread::run()
{
	setVehicleSpeedGui svspdg;
	exec();
}


