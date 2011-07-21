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
 * Range Checks.
 */

#include "rangeTests.h"
#include "VisionRangeChecks.cpp"
#include "DataFusionRangeChecks.cpp"
#include "ArmRangeChecks.cpp"
#include "ControllersAndSensorsRangeChecks.cpp"
#include "SlamRangeChecks.cpp"
#include "MainMotorControlRangeChecks.cpp"



TEST(rangeTest, allValues) {
	
	VisionRangeChecks vsChk;
	DataFusionRangeChecks dfChk;
	RoboticArmRangeChecks armChk;
	ControllersAndSensorsRangeChecks csChk;
	MainMotorControlRangeChecks mmcChk;
	SlamRangeChecks slChk;
	
	ros::Duration d(.1);
	for (int i = 0; i < 500; i++) {
		d.sleep(); 
		ros::spinOnce();
	}
}


int main(int argc, char** argv) {
	 ::testing::InitGoogleMock(&argc, argv);
  ros::init(argc,argv, "test");
  return RUN_ALL_TESTS();
}
