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
/**
 * Test for checking that all nodes exist
 */

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "interface_testing/InterfaceTester.h"

//NAVIGATION
TEST(NAVIGATIONtest, plannerNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/navigation/plannerNode"));
}

TEST(NAVIGATIONtest, navigatorNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/navigation/navigatorNode"));
}

TEST(NAVIGATIONtest, streamerNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/navigation/navigationStreamer"));
}

//STATE MANAGER
TEST(STATEMANAGERtest, stateServerNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/stateServer"));
}

//DIAGNOSTICS
TEST(DIAGNOSTICStest, diagnosticsNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/network_detector"));	
	EXPECT_TRUE(InterfaceTester::checkForNode("/hd_monitor"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/counter/holeCounter"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/counter/faceCounter"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/counter/motionCounter"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/counter/hazmatCounter"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/cpu_monitor"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/diagnostic_aggregator"));
}

//SLAM	
TEST(SLAMtest, slamNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/slam/slamModule"));
}

TEST(SLAMtest, laserNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/slam/laserURG"));
}

//DATA FUSION
TEST(DATAFUSIONtest, dataFusionNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/sensors/dataFusion"));
}

//SENSORS
TEST(SENSORStest, sensorsNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/sensors/xmega"));
}

//MAIN MOTOR CONTROL
TEST(MOTORStest, motorsNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/mainMotorControl/eposController"));
}

//ROBOTIC ARM
TEST(ARMtest, armNavigationNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/arm/armNavigation"));	
}

TEST(ARMtest, armControlNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/arm/armControl"));
}

//VISION
TEST(VISIONtest, visionNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/vision/webNode"));	
}

TEST(VISIONtest, stereoNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/videre_stereo_cam"));	
}

//GUI
TEST(GUItest, guiNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_butterfly"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_co2"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_ir"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_mainMotorControl_status"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_sonar"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_thermal0"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_thermal1"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui_thermal2"));
	EXPECT_TRUE(InterfaceTester::checkForNode("/gui/gui"));	
}

//MUTEXES
TEST(MUTEXtest, headMutexNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/headMutex"));
}

TEST(MUTEXtest, mlxMutexNode) {
	EXPECT_TRUE(InterfaceTester::checkForNode("/mlxMutex"));
}



