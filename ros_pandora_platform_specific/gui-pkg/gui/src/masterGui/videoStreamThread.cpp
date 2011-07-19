/**
  * videoStreamThread implementation files
  * Author: Dimitrios Vitsios
  * Date: 19 May 2011
  * Change History: 
*/


#include "videoStreamThread.h"
#include "videoStreaming.h"
#include "masterGui.h"


void videoStreamThread::run()
{
	videoStreaming vdStr;
	exec();
}
