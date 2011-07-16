#include "rangeTests.h"
#include "slam_communications/laserScanMsg.h"
#include "slam_communications/robotPoseMsg.h"

typedef slam_communications::laserScanMsg laserScan;
typedef slam_communications::robotPoseMsg robotPose;

class SlamRangeChecks {
	
	MockSubscriber<laserScan> laserScanSubscriber;
	MockSubscriber<robotPose> robotPoseSubscriber;

	MockSubscriber<tfMsg> tfMsgSubscriber;
	
	public:
		
	SlamRangeChecks() : laserScanSubscriber("/slam/laserScanMsg"), tfMsgSubscriber("/tf"),
						robotPoseSubscriber("/slam/robotPoseMsg") {
							
							
			EXPECT_CALL(laserScanSubscriber, subscriberActualCallback(
						Pointee(Field(&laserScan::data, Each(
							IsBetween(-1,6000))))
					)).Times(AtLeast(0));

			EXPECT_CALL(tfMsgSubscriber, subscriberActualCallback( Pointee(tfLimit("world","robotCenter", -40,40,-40,40,0,0)) )).Times(AtLeast(0));	
		
					
			EXPECT_CALL(robotPoseSubscriber, subscriberActualCallback(AllOf(
			
						Pointee(Field(&robotPose::x, IsBetween(0,4000))),
						Pointee(Field(&robotPose::y, IsBetween(0,4000))),
						Pointee(Field(&robotPose::theta, IsBetween(-PI,PI)))
			))).Times(AtLeast(0));
							
		};
	};
