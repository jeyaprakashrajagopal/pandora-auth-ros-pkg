/*
 *	Copyright (C) 2011 by Pandora Robotics Team, Aristotle Univeristy of Thessaloniki, Greece
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in
 *	all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *	THE SOFTWARE.
 */
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "interface_testing/InterfaceTester.h"


/**
 * Navigation Testing
 */

//For Service Servers
TEST(NAVIGATIONtest, navigationMapDataSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/navigation/navigationMapSrv"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/navigation/navigationMapSrv","/navigation/"));
}

TEST(NAVIGATIONtest, victimIdentificationSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/navigation/victimIdentificationSrv"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/navigation/victimIdentificationSrv","/navigation/"));
}

TEST(NAVIGATIONtest, geotiffSrvSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/navigation/geotiffSrv"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/navigation/geotiffSrv","/navigation/"));
}

//For Service Clients
TEST(NAVIGATIONtest, slamMapSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/slam/slamMap"));
}

//For Subscribers
TEST(NAVIGATIONtest, navigationStateSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/robot/state/clients"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/navigation/"));
}

TEST(NAVIGATIONtest, slamRobotPoseSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/slam/robotPose"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/slam/robotPose","/navigation/"));
}

TEST(NAVIGATIONtest, visionArenaColorsSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/vision/arenaColors"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/vision/arenaColors","/navigation/"));
}

TEST(NAVIGATIONtest, visionHazmatIdentificationSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/vision/hazmats"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/vision/hazmats","/navigation/"));
}

TEST(NAVIGATIONtest, sensorsCompassSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/compass"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/compass","/navigation/"));
}

TEST(NAVIGATIONtest, dataFusionSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/dataFusion/fusionData"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/dataFusion/fusionData","/navigation/"));
}

TEST(NAVIGATIONtest, sensorsButterflySubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/butterfly"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/butterfly","/navigation/"));
}

TEST(NAVIGATIONtest, sensorsSonarSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/sonar"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/sonar","/navigation/"));
}

TEST(NAVIGATIONtest, mainMotorControlStatusSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/mainMotorControl/status"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/mainMotorControl/status","/navigation/"));
}


TEST(NAVIGATIONtest, semiAutonomousSubscriberTest)  
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/guiMsgs/semiAutonomous"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/guiMsgs/semiAutonomous","/navigation/"));
}

TEST(NAVIGATIONtest, sensorsIrSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/ir"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/ir","/navigation/"));
}



//For Action Client
TEST(NAVIGATIONtest, armNavigationToGoalActionClientTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionClient("/arm/navigateToGoal","RoboticArm_communications/navigateToGoal"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeClient("/arm/navigateToGoal","/navigation/"));
}

//For Publishers
TEST(NAVIGATIONtest, navigationStatePublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/robot/state/server","/navigation/"));
}

TEST(NAVIGATIONtest, victimsDataPublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/navigation/victimsData","navigation_communications/victimsMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/navigation/victimsData","/navigation/"));
}


/** 
 * mainMotorControl Testing
 */
 
//For Publishers
TEST(MAINMOTORCONTROLtest, mainMotorControlStatusPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/mainMotorControl/status","mainMotorControl_communications/mainMotorStateMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/mainMotorControl/status","/mainMotorControl/"));
}

//For Service Servers
TEST(MAINMOTORCONTROLtest, mainMotorControlSpeedSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/mainMotorControl/setVehicleSpeed"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/mainMotorControl/setVehicleSpeed","/mainMotorControl/"));
	
}


/**
 * controllersAndSensors Testing
 */
 
//For Service Servers
TEST(CONTROLLERSANDSENSORStest, sensorsLaserPointerControlSrvTest)
{
	EXPECT_TRUE(InterfaceTester::checkForService("/sensors/laserPointerControl"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/sensors/laserPointerControl","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsLedControlSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/sensors/ledControl"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/sensors/ledControl","/sensors/"));
}


//For Action Servers
TEST(CONTROLLERSANDSENSORStest, sensorsCameraRotationActionServerTest) {
	EXPECT_TRUE(InterfaceTester::checkForActionServer("/sensors/cameraRotation","controllersAndSensors_communications/controlServo"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeServer("/sensors/cameraRotation","/sensors/"));
	
}

TEST(CONTROLLERSANDSENSORStest, sensorsMoveHeadActionServerTest) {
	EXPECT_TRUE(InterfaceTester::checkForActionServer("/sensors/moveHead","controllersAndSensors_communications/controlServo"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeServer("/sensors/moveHead","/sensors/"));
}

//For Publishers
TEST(CONTROLLERSANDSENSORStest, sensorsSoundExistencePublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/soundExistence","controllersAndSensors_communications/soundExistenceMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/soundExistence","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsCompassPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/compass","controllersAndSensors_communications/compassMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/compass","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsIrPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/ir","controllersAndSensors_communications/irMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/ir","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsHeadIrPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/headIr","controllersAndSensors_communications/headIrMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/headIr","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsTpa0PublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/tpa/0","controllersAndSensors_communications/tpaMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/tpa/0","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsTpa1PublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/tpa/1","controllersAndSensors_communications/tpaMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/tpa/1","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsTpa2PublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/tpa/2","controllersAndSensors_communications/tpaMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/tpa/2","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsCo2PublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/co2","controllersAndSensors_communications/co2Msg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/co2","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsButterflyPublisherTest) {
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/butterfly","controllersAndSensors_communications/butterflyMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/butterfly","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsSonarPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/sonar","controllersAndSensors_communications/sonarMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/sonar","/sensors/"));
}

TEST(CONTROLLERSANDSENSORStest, sensorsMlxPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/sensors/mlx","controllersAndSensors_communications/mlxTempMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/sensors/mlx","/sensors/"));
}

/**
 * GUI Testing
 */
 
//For Subscribers
TEST(GUItest, guiStateSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/robot/state/clients"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/gui/"));
}

TEST(GUItest, mainMotorControlStatusSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/mainMotorControl/status"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/mainMotorControl/status","/gui/"));
}

TEST(GUItest, butterflySubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/butterfly"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/butterfly","/gui/"));
}

TEST(GUItest, co2SubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/co2"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/co2","/gui/"));
}

TEST(GUItest, irSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/ir"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/ir","/gui/"));
}

TEST(GUItest, sonarSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/sonar"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/sonar","/gui/"));
}

TEST(GUItest, mlxSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/mlx"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/mlx","/gui/"));
}

TEST(GUItest, tpa0SubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/tpa/0"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/0","/gui/"));
}

TEST(GUItest, tpa1SubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/tpa/1"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/1","/gui/"));
}

TEST(GUItest, tpa2SubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/gui/sensors/tpa/2"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/2","/gui/"));
}

TEST(GUItest, mapStreamingSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/navigation/navigationStreaming"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/navigation/navigationStreaming","/gui/"));
}

TEST(GUItest,imageSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/vision/image"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/vision/image/theora","/gui/"));
}




//TEST(GUItest, robotStateSubscriberTest) 
//{
	//EXPECT_TRUE(InterfaceTester::checkForSubscribed("/robot/state"));
	//EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/robot/state","/gui/gui"));
//}

//For Publishers
TEST(GUItest, guiStatePublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/robot/state/server","/gui/"));
}

TEST(GUItest, targetPositionPublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/guiMsgs/semiAutonomous","gui_communications/targetPosition"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/guiMsgs/semiAutonomous","/gui/"));
}

//For Action Clients
TEST(GUItest, armControlActionClientTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionClient("/arm/armControl","RoboticArm_communications/moveArm"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeClient("/arm/armControl","/gui/gui"));
}

//For Service Clients
TEST(GUItest, mainMotorControlSpeedSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/mainMotorControl/setVehicleSpeed"));
}

TEST(GUItest, geotiffSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/navigation/geotiffSrv"));
}


/**
 * slam Testing
 */

//For Subscribers

TEST(SLAMtest, slamStateSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/robot/state/clients"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/slam/"));
}

TEST(SLAMtest, mainMotorControlStatusSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/mainMotorControl/status"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/mainMotorControl/status","/slam/slam"));
}

//For Publishers
TEST(SLAMtest, slamStatePublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/robot/state/server","/slam/"));
}

TEST(SLAMtest, slamRobotPosePublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/slam/robotPose", "slam_communications/robotPoseMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/slam/robotPose", "/slam/slam"));
}

TEST(SLAMtest, tfSubscribedTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/tf","tf/tfMessage"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/tf", "/slam/slam"));
}

//For Service Servers
TEST(SLAMtest, slamMapSrvTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForService("/slam/slamMap"));
	EXPECT_TRUE(InterfaceTester::checkForNodeService("/slam/slamMap", "/slam/slam"));
}

/**
 * dataFusion Testing
 */
 
//For Subscribers
TEST(DATAFUSIONtest, dataFusionStateSubscriberTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/robot/state/clients"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsCo2SubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/co2"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/co2","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsMlxSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/mlx"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/mlx","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsSoundExistenceSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/soundExistence"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/soundExistence","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsTpa0SubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/tpa/0"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/tpa/0","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsTpa1SubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/tpa/1"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/tpa/1","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, sensorsTpa2SubscriberTest) {
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/tpa/2"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/tpa/2","/sensors/dataFusion"));
	
}

TEST(DATAFUSIONtest, visionVictimDirectionSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/vision/victimDirection"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/vision/victimDirection","/sensors/dataFusion"));
}

/*
TEST(DATAFUSIONtest, visionVictimPositionSubscriberTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/stereo/victimPosition"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/stereo/victimPosition","/sensors/dataFusion"));
}
*/
//For Publishers
TEST(DATAFUSIONtest, dataFusionStatePublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/robot/state/server","/sensors/dataFusion"));
}

TEST(DATAFUSIONtest, dataFusionPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/dataFusion/fusionData","dataFusion_communications/fusionDataMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/dataFusion/fusionData","/sensors/dataFusion"));
}

/**
 * vision Testing
 */
//For publishers
TEST(VISIONtest, visionStatePublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/robot/state/server","/vision/"));
}

 TEST(VISIONtest, hazmatIdentificationPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/vision/hazmats","vision_communications/hazmatIdentificationMsg"));
        EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/vision/hazmats","/vision/"));
}

 TEST(VISIONtest, visionArenaColorsPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/vision/arenaColors","vision_communications/lineColorMsg"));
        EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/vision/arenaColors","/vision/"));
}
	
TEST(VISIONtest, visionVictimDirectionPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/vision/victimDirection","vision_communications/victimIdentificationDirectionMsg"));
        EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/vision/victimDirection","/vision/"));
}
/*
TEST(VISIONtest, visionVictimPositionPublisherTest)
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/stereo/victimPosition","vision_communications/victimIdentificationPositionMsg"));
        EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/stereo/victimPosition","/stereo/"));
}
*/

/**
 *  armNavigation Testing 
 */

//For Subscribers
TEST(ARMNAVIGATIONtest, dataFusionSubscribedTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/dataFusion/fusionData"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/dataFusion/fusionData","/arm/armNavigation"));
}

TEST(ARMNAVIGATIONtest, tfSubscribedTest){
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/tf"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/tf","/arm/armNavigation"));
}


TEST(ARMNAVIGATIONtest, sensorsCompassSubscribedTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/compass"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/compass","/arm/armNavigation"));
}


//For Action Clients
TEST(ARMNAVIGATIONtest, armControlActionClientTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionClient("/arm/armControl","RoboticArm_communications/moveArm"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeClient("/arm/armControl","/arm/armNavigation"));
}

//TEST(ARMNAVIGATIONtest, soundScanActionClientTest) 
//{
	//EXPECT_TRUE(InterfaceTester::checkForActionClient("/sensors/soundScan","controllersAndSensors_communications/soundScan"));
	//EXPECT_TRUE(InterfaceTester::checkForActionNodeClient("/sensors/soundScan","/arm/armNavigation"));
//}


//For Action Server
TEST(ARMNAVIGATIONtest, armNavigateToGoalActionServerTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionServer("/arm/navigateToGoal","RoboticArm_communications/navigateToGoal"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeServer("/arm/navigateToGoal","/arm/armNavigation"));
}



/**
 *  armControl Testing 
 */
//For Subscribers

TEST(ARMCONTROLtest, sensorsCompassSubscribedTest)
{
	EXPECT_TRUE(InterfaceTester::checkForSubscribed("/sensors/compass"));
	EXPECT_TRUE(InterfaceTester::checkForSubscribedNode("/sensors/compass","/arm/armControl"));
}


// For publishers
TEST(ARMCONTROLtest, tfPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/tf","tf/tfMessage"));
	EXPECT_TRUE(InterfaceTester::checkForNodePublishing("/tf","/arm/armControl"));
}

//For Action Clients
TEST(ARMCONTROLtest, sensorsMoveHeadActionClientTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionClient("/sensors/moveHead","controllersAndSensors_communications/controlServo"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeClient("/sensors/moveHead","/arm/armControl"));
}
//For Action Server
TEST(ARMCONTROLtest, armControlActionServerTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForActionServer("/arm/armControl","RoboticArm_communications/moveArm"));
	EXPECT_TRUE(InterfaceTester::checkForActionNodeServer("/arm/armControl","/arm/armControl"));
}
/**
 *  stateManager Testing 
 */
// For publishers
 TEST(STATEMANAGERtest, robotGetRobotPublisherTest) 
{
	EXPECT_TRUE(InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg"));
}




