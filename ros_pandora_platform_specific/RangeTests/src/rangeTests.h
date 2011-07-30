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
#ifndef RANGE_TESTS_H
#define RANGE_TESTS_H

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "mock_objects/MockSubscriber.h"
#include "mock_objects/MockActionServer.h"

#define PI 3.141592653589
using ::testing::AtLeast;
using ::testing::_;
using ::testing::Return;
using ::testing::Eq;
using ::testing::Ge;
using ::testing::Property;
using ::testing::Truly;
using ::testing::AllOf;
using ::testing::Field;
using ::testing::Pointee;
using ::testing::PrintToString;
using ::testing::ElementsAre;
using ::testing::Matcher;
using ::testing::AnyOf;
using ::testing::Each;

#include "std_msgs/Header.h"
typedef std_msgs::Header msgHeader;


MATCHER_P2(IsBetween, a, b, std::string(negation ? "isn't" : "is") 
	+ " between " + PrintToString(a) + " and " + PrintToString(b)) { 
		return a <=arg; 
	}

MATCHER_P6(AreBetween,xMin, xMax, yMin, yMax, zMin, zMax, std::string(negation ? "isn't" : "is") 
	+ " between x=" + PrintToString(xMin) + ", " + PrintToString(xMax) + "y="+ PrintToString(yMin) 
	+ ", " + PrintToString(yMax) + "z="+ PrintToString(zMin) 	+ ", " + PrintToString(zMax)) { 
		return xMin <= arg[0] && arg[0] <= xMax && yMin <= arg[1] && arg[1] <= yMax && zMin <= arg[2] && arg[2] <= zMax; 
	}
	
	
MATCHER_P(DistanceIsLess, distance, "") {
	return arg.x*arg.x+arg.y*arg.y+arg.z*arg.z < distance * distance;
}
	
MATCHER_P2(stepCheck, start, step, std::string(negation ? "isn't" : "is") 
	+ " between " + PrintToString(start) + " and " + PrintToString(step)) { 
		return arg-((int)((arg-start)/step))*step == 0;
	}

MATCHER_P8(tfLimit, frame, childFrame, minX, maxX, minY, maxY, minZ, maxZ, std::string(negation ? "isn't" : "is") 
	+ " between " + PrintToString(minX) + " and " + PrintToString(maxX)) { 
		bool xInLimit = arg.transform.translation.x >= minX && 	arg.transform.translation.x <= maxX;	
		bool yInLimit = arg.transform.translation.y >= minY && 	arg.transform.translation.y <= maxY;
		bool zInLimit = arg.transform.translation.z >= minZ && 	arg.transform.translation.y <= maxZ;
		bool correctFrames = (frame == arg.header.frame_id) && (childFrame == arg.child_frame_id);
		if (correctFrames)
			return xInLimit && yInLimit && zInLimit;
		
		return 1;
	}
	
#endif
