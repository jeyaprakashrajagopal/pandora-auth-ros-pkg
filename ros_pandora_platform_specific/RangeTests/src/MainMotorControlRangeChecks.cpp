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
#include "mainMotorControl_communications/mainMotorStateMsg.h"
#include "mainMotorControl_communications/motorErrorMsg.h"

typedef mainMotorControl_communications::mainMotorStateMsg motorState;
typedef mainMotorControl_communications::motorErrorMsg mError;


class MainMotorControlRangeChecks {
	
	MockSubscriber<motorState> statusSubscriber;
	
	public:
		
	MainMotorControlRangeChecks() : statusSubscriber("/mainMotorControl/status") {
					
			EXPECT_CALL(statusSubscriber, subscriberActualCallback(AllOf(
						//Pointee(Field(&motorState::currentLeft, IsBetween(-2240,2240))),
						//Pointee(Field(&motorState::currentRight, IsBetween(-2240,2240))),
						Pointee(Field(&motorState::velocityLeft, IsBetween(-6710,6710))),
						Pointee(Field(&motorState::velocityRight, IsBetween(-6710,6710))),
						Pointee(Field(&motorState::velocityLinear, IsBetween(-0.1,0.1))),
						//Pointee(Field(&motorState::velocityAngular, IsBetween(-PI,PI))),
						Pointee(Field(&motorState::powerLeft, IsBetween(-50000,50000))),
						Pointee(Field(&motorState::powerRight, IsBetween(-50000,50000))),
						Pointee(Field(&motorState::efficiencyLeft, IsBetween(0,1))),
						Pointee(Field(&motorState::efficiencyRight, IsBetween(0,1))),
						Pointee(Field(&motorState::error, Field(&mError::motorError, IsBetween(0,7))))
					))).Times(AtLeast(0));

							
		};
	};
