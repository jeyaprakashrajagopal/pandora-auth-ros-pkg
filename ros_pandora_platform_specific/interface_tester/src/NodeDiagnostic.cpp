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
#include "interface_testing/GenericDiagnostic.cpp"

class NodeDiagnostics: GenericDiagnostic {
	
	public:
	NodeDiagnostics():GenericDiagnostic("System Nodes"){};
	
	 void nodeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
		bool allOk = true;
		//-----------------------------navigation-------------------------------------
		if (!InterfaceTester::checkForNode("/navigation/plannerNode")){
			stat.add("/navigation/plannerNode","Down");
			allOk = false;
		}			
		
		if (!InterfaceTester::checkForNode("/navigation/navigatorNode")){
			stat.add("/navigation/navigatorNode","Down");
			allOk = false;
		}	
		
		if (!InterfaceTester::checkForNode("/navigation/navigationStreamer")){
			stat.add("/navigation/navigationStreamer","Down");
			allOk = false;
		}	
		//------------------------------state manager-----------------------------------
		if (!InterfaceTester::checkForNode("/stateServer")){
			stat.add("/stateServer","Down");
			allOk = false;
		}
		//------------------------------diagnostics-------------------------------------
		if (!InterfaceTester::checkForNode("/network_detector")){
			stat.add("/network_detector","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/hd_monitor")){
			stat.add("/hd_monitor","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/counter/holeCounter")){
			stat.add("/counter/holeCounter","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/counter/faceCounter")){
			stat.add("/counter/faceCounter","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/counter/motionCounter")){
			stat.add("/counter/motionCounter","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/counter/hazmatCounter")){
			stat.add("/counter/hazmatCounter","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/cpu_monitor")){
			stat.add("/cpu_monitor","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/diagnostic_aggregator")){
			stat.add("/diagnostic_aggregator","Down");
			allOk = false;
		}
		//------------------------------slam-----------------------------------
		if (!InterfaceTester::checkForNode("/slam/slamModule")){
			stat.add("/slam/slamModule","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/slam/laserURG")){
			stat.add("/slam/laserURG","Down");
			allOk = false;
		}
		//----------------------------data fusion------------------------------
		if (!InterfaceTester::checkForNode("/sensors/dataFusion")){
			stat.add("/sensors/dataFusion","Down");
			allOk = false;
		}
		//--------------------------------sensors-------------------------------
		if (!InterfaceTester::checkForNode("/sensors/xmega")){
			stat.add("/sensors/xmega","Down");
			allOk = false;
		}
		//------------------------------main motor control-----------------------
		if (!InterfaceTester::checkForNode("/mainMotorControl/eposController")){
			stat.add("/mainMotorControl/eposController","Down");
			allOk = false;
		}
		//--------------------------------robotic Arm--------------------------
		if (!InterfaceTester::checkForNode("/arm/armNavigation")){
			stat.add("/arm/armNavigation","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/arm/armControl")){
			stat.add("/arm/armControl","Down");
			allOk = false;
		}
		//----------------------------------vision---------------------------------
		if (!InterfaceTester::checkForNode("/vision/webNode")){
			stat.add("/vision/webNode","Down");
			allOk = false;
		}

		//------------------------------------gui-----------------------------------
		if (!InterfaceTester::checkForNode("/gui_butterfly")){
			stat.add("/gui_butterfly","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui_co2")){
			stat.add("/gui_co2","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui_ir")){
			stat.add("/gui_ir","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui_mainMotorControl_status")){
			stat.add("/gui_mainMotorControl_status","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui_sonar")){
			stat.add("/gui_sonar","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui_thermal0")){
			stat.add("/gui_thermal0","Down");
			allOk = false;
		}	
		if (!InterfaceTester::checkForNode("/gui_thermal1")){
			stat.add("/gui_thermal1","Down");
			allOk = false;
		}	
		if (!InterfaceTester::checkForNode("/gui_thermal2")){
			stat.add("/gui_thermal2","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/gui/gui")){
			stat.add("/gui/gui","Down");
			allOk = false;
		}
		//------------------------------mutexs---------------------------------
		if (!InterfaceTester::checkForNode("/headMutex")){
			stat.add("/headMutex","Down");
			allOk = false;
		}
		if (!InterfaceTester::checkForNode("/mlxMutex")){
			stat.add("/mlxMutex","Down");
			allOk = false;
		}
		if (allOk) {
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"All nodes are up and running");
		} else {
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"Some nodes are down");
		}
	};
	
	
};


int main(int argc, char **argv){
	ros::init(argc, argv, "NodeDiagnostic");
	NodeDiagnostics nd;
	ros::spin();
	
}
