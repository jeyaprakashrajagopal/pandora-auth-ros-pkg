/**
  * consoleManager Module header files
  * Author: Dimitrios Vitsios
  * Date: 16 June 2011
  * Change History: - 
  */

#include "moveArmGui.h"
#include <QApplication>
#include <QtCore>
#include "mainMotorStateGui.h"
#include "setVehicleSpeedGui.h"
#include "semiAutonomousGui.h"
#include "irGui.h"
#include "sonarGui.h"
#include "co2Gui.h"
#include "thermalGui.h"
#include "butterflyGui.h"
#include "debugText.h"
#include "stateManagerGui.h"
#include "victimsFound.h"
#include "displayVideo.h"
#include "displayMap.h"
#include "videoStreaming.h"
#include "mapStreaming.h"
#include "semiAutoMsgPublisher.h"
#include "soundTest.h"


class consoleManager : public QWidget 
{
		Q_OBJECT
		
		public:
			consoleManager();
		
		public Q_SLOTS:
			//functions for calling console commands on remote machine via ssh
			void onShutdownClicked();
			void onRestartClicked();
			void onStartGlobalLauncherClicked();
			void onKillGlobalLauncherClicked();
};
