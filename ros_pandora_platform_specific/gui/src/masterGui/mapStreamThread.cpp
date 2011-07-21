/**
  * mapStreamThread implementation files
  * Author: Dimitrios Vitsios
  * Date: 8 June 2011
  * Change History: 
*/


#include "mapStreamThread.h"
#include "masterGui.h"
#include "mapStreaming.h"

/*
 * Class for running map streaming in a new thread. Not used in the current version.
 */ 
void mapStreamThread::run()
{
	mapStreaming mapStr;
	exec();
}
