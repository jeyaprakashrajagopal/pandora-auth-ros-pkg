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
#include "interface_testing/InterfaceTester.h"
#include "interface_testing/GenericDiagnostic.cpp"

#include "state_manager/StateClient.h"

//#include "InterfaceDiagnostic.h"


using namespace state_manager_communications;

class InterfaceDiagnostics: GenericDiagnostic, StateClient {
	
	int currentState;
	
	public:
	InterfaceDiagnostics():GenericDiagnostic("System Interfaces"), StateClient(false){
		currentState = 0;
	};
	
	~InterfaceDiagnostics() {};
	virtual void startTransition (int newState);
	int getState();
	
	void nodeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
		bool allOk = true;
		
		//NAVIGATION TESTS
		if(!InterfaceTester::checkForNodeService("/navigation/navigationMapSrv","/navigation/")){
			stat.add("Navigation is not offering the service", "/navigation/navigationMapSrv");
			allOk = false;
		}	
		
		if(!InterfaceTester::checkForNodeService("/navigation/navigationMapSrv","/navigation/")){
			stat.add("Navigation is not offering the service", "/navigation/navigationMapSrv");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodeService("/navigation/geotiffSrv","/navigation/")){
			stat.add("Navigation is not offering the service","/navigation/geotiffSrv");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/robot/state/clients");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/slam/robotPose", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/slam/robotPose");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/vision/arenaColors", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/vision/arenaColors");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/vision/hazmats", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/vision/hazmats");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/sensors/compass", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/sensors/compass");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/sensors/butterfly", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/sensors/butterfly");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/sensors/sonar", "/navigation/")
		&& ((currentState!=robotModeMsg::MODE_OFF)&&(currentState!=robotModeMsg::MODE_TELEOPERATED_LOCOMOTION)
		&&(currentState!=robotModeMsg::MODE_ARM_TELEOPERATION))){
			stat.add("Navigation is not subscribing to topic", "/sensors/sonar");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/dataFusion/fusionData", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/dataFusion/fusionData");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/mainMotorControl/status", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/mainMotorControl/status");
			allOk = false;
		}

		if(!InterfaceTester::checkForSubscribedNode("/guiMsgs/semiAutonomous", "/navigation/")){
			stat.add("Navigation is not subscribing to topic", "/guiMsgs/semiAutonomous");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/ir", "/navigation/") 
		 && ((currentState!=robotModeMsg::MODE_OFF) && (currentState!=robotModeMsg::MODE_TELEOPERATED_LOCOMOTION)
		&& (currentState!=robotModeMsg::MODE_ARM_TELEOPERATION))){
			stat.add("Navigation is not subscribing to topic", "/sensors/ir");
			allOk = false;
		}

		if(!InterfaceTester::checkForActionNodeClient("/arm/navigateToGoal","/navigation/")){
			stat.add("Navigation is not calling action", "/arm/navigateToGoal");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/robot/state/server","/navigation/")){
			stat.add("Navigation is not publishing to topic", "/robot/state/server");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/navigation/victimsData","/navigation/")){
			stat.add("Navigation is not publishing to topic", "/navigation/victimsData");
			allOk = false;
		}
		
		//MAIN MOTOR CONTROL
		if(!InterfaceTester::checkForNodePublishing("/mainMotorControl/status","/mainMotorControl/")){
			stat.add("Main Motor Control is not publishing to topic","/mainMotorControl/status");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodeService("/mainMotorControl/setVehicleSpeed", "/mainMotorControl/")){
			stat.add("Main Motor Control is not offering the service", "/mainMotorControl/setVehicleSpeed");
			allOk = false;
		}
		
		//CONTROLLERS AND SENSORS
		if(!InterfaceTester::checkForNodeService("/sensors/laserPointerControl","/sensors/")){
			stat.add("Controllers & Sensors are not offering the service", "/sensors/laserPointerControl");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodeService("/sensors/ledControl","/sensors/")){
			stat.add("Controllers & Sensors are not offering the service", "/sensors/ledControl");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForActionNodeServer("/sensors/cameraRotation","/sensors/")){
			stat.add("Controllers & Sensors are not offering the action", "/sensors/cameraRotation");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForActionNodeServer("/sensors/moveHead","/sensors/")){
			stat.add("Controllers & Sensors are not offering the action", "/sensors/moveHead");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/soundExistence","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/soundExistence");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/compass","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/compass");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/butterfly","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/butterfly");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/sonar","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/sonar");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/ir","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/ir");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/headIr","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/headIr");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/tpa/0","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/tpa/0");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/tpa/1","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/tpa/1");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/tpa/2","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/tpa/2");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/co2","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/co2");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/sensors/mlx","/sensors/")){
			stat.add("Controllers & Sensors are not publishing to topic", "/sensors/mlx");
			allOk = false;
		}
		
		//GUI
		if(!InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/gui/")){
			stat.add("GUI is not subscribing to topic", "/robot/state/clients");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/mainMotorControl/status", "/gui/")){
			stat.add("GUI is not subscribing to topic", "/mainMotorControl/status");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/butterfly","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/butterfly");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/co2","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/co2");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/sonar","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/sonar");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/ir","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/ir");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/mlxthrottled","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/mlxthrottled");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/0","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/tpa/0");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/1","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/tpa/1");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/gui/sensors/tpa/2","/gui/")){
			stat.add("GUI is not subscribing to topic", "/gui/sensors/tpa/2");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/navigation/navigationStreaming","/gui/")){
			stat.add("GUI is not subscribing to topic", "/navigation/navigationStreaming");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/vision/image","/gui/")){
			stat.add("GUI is not subscribing to topic", "/vision/image");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/robot/state/server","/gui/")){
			stat.add("GUI is not publishing to topic", "/robot/state/server");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodePublishing("/guiMsgs/semiAutonomous","/gui/")){
			stat.add("GUI is not publishing to topic", "/guiMsgs/semiAutonomous");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForActionNodeClient("/arm/armControl","/gui/gui")){
			stat.add("Navigation is not calling action", "/arm/armControl");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForService("/mainMotorControl/setVehicleSpeed")){
			stat.add("GUI is not calling service", "/mainMotorControl/setVehicleSpeed");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForNodeService("/navigation/geotiffSrv","/navigation/")){
			stat.add("GeoTiff Service in not provided", "/navigation/geotiffSrv");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForService("/slam/slamMap")){
			stat.add("Slam calling service", "/slam/slamMap");
			allOk = false;
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/slam/")){
			stat.add("SLAM has is not subscibed","/robot/state/clients");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/mainMotorControl/status","/slam/slam")) {
			stat.add("SLAM is not subscribed","/mainMotorControl/status");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/robot/state/server","/slam/")) {
			stat.add("SLAM is not publishing ","/robot/state/server");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/slam/robotPose", "/slam/slam")) {
			stat.add("SLAM does not have publishing interface","/slam/robotPose");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/tf", "/slam/slam")) {
			stat.add("SLAM is does not have publishing interface","/tf");
		}
		
		if(!InterfaceTester::checkForNodeService("/slam/slamMap", "/slam/slam")) {
			stat.add("SLAM does not provide service","/slam/slamMap");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/robot/state/clients", "/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to", "/robot/state/clients");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/co2","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed","/sensors/co2");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/mlx","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/sensors/mlx");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/soundExistence","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/sensors/soundExistence");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/tpa/0","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/sensors/tpa/0");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/tpa/1","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/sensors/tpa/1");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/tpa/2","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/sensors/tpa/2");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/vision/victimDirection","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/vision/victimDirection");
		}
		
		/*if(!InterfaceTester::checkForSubscribedNode("/stereo/victimPosition","/sensors/dataFusion")) {
			stat.add("Data Fusion is not subscribed to","/stereo/victimPosition");
		}*/
		
		if(!InterfaceTester::checkForNodePublishing("/robot/state/server","/sensors/dataFusion")) {
			stat.add("Data Fusion is not publishing to","/robot/state/server");
		}
		
		if (!InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg")) {
			stat.add("Topic of StateServer is wrong","/robot/state/server");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/dataFusion/fusionData","/sensors/dataFusion")){
			stat.add("Topic of Data Fusion is not publishing","/dataFusion/fusionData");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/robot/state/server","/vision/")) {
			stat.add("Vision is not publishing at","/robot/state/server");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/vision/hazmats","/vision/")) {
			stat.add("Vision is not publishing at","/vision/hazmats");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/vision/arenaColors","/vision/")) {
			stat.add("Vision is not publishing","vision/arenaColors");
		}
		
		if(!InterfaceTester::checkForNodePublishing("/vision/victimDirection","/vision/")) {
			stat.add("Vision is not publishing at", "/vision/victimDirection");
		}
		
		if(!InterfaceTester::checkForTopicPublishing("/vision/victimDirection","vision_communications/victimIdentificationDirectionMsg")) {
			stat.add("Vision topic is incorrect","/vision/victimDirection");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/dataFusion/fusionData","/arm/armNavigation")) {
			stat.add("Arm Navigation is not subscribed to: ","/dataFusion/fusionData");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/dataFusion/fusionData","/arm/armNavigation")) {
			stat.add("Arm Navigation is not subscribed to: ","/dataFusion/fusionData");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/tf","/arm/armNavigation")){
			stat.add("Arm Navigation is not subscribed to: ","/tf");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/compass","/arm/armNavigation")){
			stat.add("Arm Navigation is not subscribed to: ","/sensors/compass");
		}
		
		if(!InterfaceTester::checkForActionNodeClient("/arm/armControl","/arm/armNavigation")){
			stat.add("Arm Navigation is not subscribed to: ","/arm/armControl");
		}
		
		if(!InterfaceTester::checkForSubscribedNode("/sensors/compass","/arm/armControl")){
			stat.add("Arm Control is not subscribed to: ","/sensors/compass");
		}
		
		
		if(!InterfaceTester::checkForNodePublishing("/tf","/arm/armControl")){
			stat.add("Arm Control is not subscribed to: ","/tf");
		}
		
		if(!InterfaceTester::checkForActionNodeClient("/sensors/moveHead","/arm/armControl")){
			stat.add("Arm Control is not an action client","/sensors/moveHead");
		}
		
		if(!InterfaceTester::checkForActionNodeServer("/arm/armControl","/arm/armControl")) {
			stat.add("Arm Control is not action node server","/arm/armControl");
		}
		
		if(!InterfaceTester::checkForTopicPublishing("/robot/state/server","stateManager_communications/robotModeMsg")){
			stat.add("State Server topic is incorrect","/robot/state/server");
		}
		
		if (allOk) {
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"All interfaces are OK");
		} else {
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"Some interfaces are OFF");
		}
	};
	
	
};

void InterfaceDiagnostics::startTransition (int newState) {
		currentState = newState;
		transitionComplete(currentState);	
}

int InterfaceDiagnostics::getState() {
	return currentState;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "InterfaceDiagnostic");
	InterfaceDiagnostics nd;
	ros::spin();
	
	
}
