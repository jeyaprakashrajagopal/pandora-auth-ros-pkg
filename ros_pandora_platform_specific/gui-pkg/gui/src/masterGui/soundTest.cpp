/**
  * soundTest Module implementation
  * Author: Dimitrios Vitsios
  * Date: 28 June 2011
  * Change History: - 
  */

#include "soundTest.h"
#include "masterGui.h"

soundTest::soundTest()
{
	
	//Declare subscribers
	_soundSubscriber = _handle.subscribe("/sensors/soundExistence", 1 ,&soundTest::soundReceiver,this);

	connect(gui->ui.testSoundButton, SIGNAL(clicked()), this, SLOT(testSoundService()));
	
	QTimer *soundTimer = new QTimer(this);
    connect(soundTimer, SIGNAL(timeout()), this, SLOT(updateSoundIndicator()));
    soundTimer->start(1000);
}

void soundTest::updateSoundIndicator()
{
	if(msg.soundExists == true){
		gui->ui.soundExistenceLabel->setStyleSheet("color: yellow");
		gui->ui.soundExistenceLabel->setText("true");
	}
	else{
		gui->ui.soundExistenceLabel->setStyleSheet("color: red");
		gui->ui.soundExistenceLabel->setText("false");
	}
	gui->ui.progressBar->setValue(msg.certainty*100);
}

void soundTest::soundReceiver(const controllersAndSensors_communications::soundExistenceMsg &soundMessage)
{	
	//ROS_INFO("Sound msg received");
	mutex.lock();
	msg = soundMessage;
	mutex.unlock();
}


void soundTest::testSoundService()
{
	//soundExistPercentage = (butterflyMessage.voltage[0]-20)/BATTERIES_RANGE*100;
	//gui->ui.MotorsBatteryProgressBar->setValue(motorsPercentage);
}
