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
