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
