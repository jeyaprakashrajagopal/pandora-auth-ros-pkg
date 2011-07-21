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
#include "RoboticArm_communications/moveArmAction.h"
#include "RoboticArm_communications/navigateToGoalAction.h"
#include "tf/transform_listener.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"


typedef RoboticArm_communications::moveArmGoal  armGoal;
typedef RoboticArm_communications::navigateToGoalGoal  navigateGoal;
typedef geometry_msgs::TransformStamped tfMsg;
typedef geometry_msgs::Transform transf;
typedef geometry_msgs::Vector3 vect;


class RoboticArmRangeChecks {
	
	MockSubscriber<armGoal> moveArmActionGoal;
	MockSubscriber<navigateGoal> navigateActionGoal;
	MockSubscriber<tfMsg> tfMsgSubscriber;
	
	public: 
	
	RoboticArmRangeChecks(): moveArmActionGoal("/arm/armControl/goal"),
							 tfMsgSubscriber("/tf"),
							 navigateActionGoal("/arm/navigateToGoal/goal")	{
		
		EXPECT_CALL(moveArmActionGoal, subscriberActualCallback(AllOf(
					Pointee(Field(&armGoal::motionType, IsBetween(1,5))),
					Pointee(Field(&armGoal::gripperPosition, IsBetween(1,3))),				
					Pointee(Field(&armGoal::a, IsBetween(-PI,PI))),				
					Pointee(Field(&armGoal::b, IsBetween(-PI,PI))),				
					Pointee(Field(&armGoal::c, Eq(0))),		
					Pointee(Field(&armGoal::th5, IsBetween(-PI/2, PI/2))),				
					Pointee(Field(&armGoal::th4, IsBetween(-PI, PI/2))),				
					Pointee(Field(&armGoal::th1, IsBetween(-PI, PI))),
					Pointee(DistanceIsLess(.98))			
				))).Times(AtLeast(0));	
				
		EXPECT_CALL(tfMsgSubscriber, subscriberActualCallback( Pointee(tfLimit("armBase","armHead",.065,.98,.065,.98,.065,.98)) )).Times(AtLeast(0));	
		EXPECT_CALL(tfMsgSubscriber, subscriberActualCallback( Pointee(
				Field(&tfMsg::transform,
					Field(&transf::translation,DistanceIsLess(.98))
			)))).Times(AtLeast(0));
		
		EXPECT_CALL(navigateActionGoal, subscriberActualCallback(AllOf(
					Pointee(Field(&navigateGoal::state, IsBetween(1,8))),
					Pointee(Field(&navigateGoal::th, IsBetween(-PI,PI)))		
		))).Times(AtLeast(0));	
		};
		
};
