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
#include "vision_communications/victimIdentificationDirectionMsg.h"
#include "vision_communications/victimIdentificationPositionMsg.h"
#include "vision_communications/hazmatIdentificationMsg.h"
#include "vision_communications/lineColorMsg.h"

#define XCAM 0.593
#define YCAM 0.436

/**
 * Testing value ranges for vision
 */	
typedef vision_communications::victimIdentificationDirectionMsg  victimDirectionMsg;
typedef vision_communications::victimIdentificationPositionMsg  victimPositionMsg;
typedef vision_communications::hazmatIdentificationMsg  hazmatMsg;
typedef vision_communications::lineColorMsg  lineMsg;


class VisionRangeChecks {
	
	public: 
	
	MockSubscriber<victimDirectionMsg> victimDirectionSubscriber;
	MockSubscriber<victimPositionMsg> victimPositionSubscriber;
	MockSubscriber<lineMsg> lineColorSubscriber;
	MockSubscriber<hazmatMsg> hazmatSubscriber;
	
		VisionRangeChecks() : victimDirectionSubscriber("/vision/victimDirection"),
													victimPositionSubscriber("/vision/victimPosition"),
													lineColorSubscriber("/vision/lineColor"),
													hazmatSubscriber("/vision/hazmats")	{
				
				
	
	EXPECT_CALL(victimDirectionSubscriber, subscriberActualCallback(
			AllOf(	
					Pointee(Field(&victimDirectionMsg::x, IsBetween(-XCAM,XCAM))),
					Pointee(Field(&victimDirectionMsg::y, IsBetween(-YCAM, YCAM))),
					Pointee(Field(&victimDirectionMsg::area, IsBetween(150,307200))),
					Pointee(Field(&victimDirectionMsg::type, IsBetween(1,4))),
					Pointee(Field(&victimDirectionMsg::probability, IsBetween(0,1))),
					Pointee(Field(&victimDirectionMsg::area, Ge(0))),
					Pointee(Field(&victimDirectionMsg::header, Field(&msgHeader::frame_id, Eq("headCamera"))))
				))).Times(AtLeast(0));	

	
	EXPECT_CALL(victimPositionSubscriber, subscriberActualCallback(
				AllOf(
					Pointee(Field(&victimPositionMsg::position, ElementsAre(IsBetween(0, 4000),IsBetween(0, 4000),IsBetween(400,4000) ))),
					Pointee(Field(&victimPositionMsg::header,Field(&msgHeader::frame_id, Eq("headCamera"))))
				))).Times(AtLeast(0));	
		
		EXPECT_CALL(hazmatSubscriber, subscriberActualCallback(
				AllOf(	
						Pointee(Field(&hazmatMsg::x, IsBetween(-XCAM,XCAM))),
						Pointee(Field(&hazmatMsg::y, IsBetween(-YCAM, YCAM)))
					))).Times(AtLeast(0));	
				
	
	EXPECT_CALL(lineColorSubscriber, subscriberActualCallback(
			AnyOf(	
					AllOf(
						Pointee(Field(&lineMsg::orientation, Eq(1))),
						Pointee(Field(&lineMsg::fromAngle, IsBetween(-XCAM,XCAM))),
						Pointee(Field(&lineMsg::toAngle, IsBetween(-XCAM, XCAM))),
						Pointee(Field(&lineMsg::color, IsBetween(1,4)))),
						
					AllOf(
						Pointee(Field(&lineMsg::orientation, Eq(2))),
						Pointee(Field(&lineMsg::fromAngle, IsBetween(-YCAM, YCAM))),
						Pointee(Field(&lineMsg::toAngle, IsBetween(-YCAM, YCAM))),
						Pointee(Field(&lineMsg::color, IsBetween(1,4))))
						
				))).Times(AtLeast(0));
			
		};
	
	
};





