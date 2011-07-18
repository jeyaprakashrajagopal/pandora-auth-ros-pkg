/**
  * consoleManager Module implementation
  * Author: Dimitrios Vitsios
  * Date: 16 June 2011
  * Change History: - 
  */

#include "consoleManager.h"
#include "math.h"
#include "masterGui.h"


consoleManager::consoleManager()
{
	connect(gui->ui.shutdownPandoraButton, SIGNAL(clicked()), this, SLOT(onShutdownClicked()));
	connect(gui->ui.restartPandoraButton, SIGNAL(clicked()), this, SLOT(onRestartClicked()));
	connect(gui->ui.startGlobLauncherButton, SIGNAL(clicked()), this, SLOT(onStartGlobalLauncherClicked()));
	connect(gui->ui.killGlobLauncherButton, SIGNAL(clicked()), this, SLOT(onKillGlobalLauncherClicked()));
}

void consoleManager::onStartGlobalLauncherClicked()
{
      QStringList arguments;
      QProcess exec;
      arguments << "-c" << "ssh pandora@10.100.0.5 'roslaunch --pid=/tmp/globalLaunch.pid pandora_robot globalLauncherReal.launch'";
      exec.start("/bin/sh", arguments);
      exec.waitForFinished(4000);
}

void consoleManager::onKillGlobalLauncherClicked()
{
      QStringList arguments;
      QProcess exec;
      arguments << "-c" << "ssh pandora@10.100.0.5 'kill -INT `cat /tmp/globalLaunch.pid` '";
      exec.start("/bin/sh", arguments);
      exec.waitForFinished(4000);
}

void consoleManager::onShutdownClicked()
{
      QStringList arguments;
      QProcess exec;
      arguments << "-c" << "ssh pandora@10.100.0.5 'echo pandora | sudo -S shutdown -h now'";
      exec.start("/bin/sh", arguments);
      exec.waitForFinished(2000);
}

void consoleManager::onRestartClicked()
{
QStringList arguments;
      QProcess exec;
      arguments << "-c" << "ssh pandora@10.100.0.5 'echo pandora | sudo -S shutdown -r now'";
      exec.start("/bin/sh", arguments);
      exec.waitForFinished(2000);
}
