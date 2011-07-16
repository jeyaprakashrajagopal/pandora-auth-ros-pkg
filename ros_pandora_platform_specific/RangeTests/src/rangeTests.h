#ifndef RANGE_TESTS_H
#define RANGE_TESTS_H

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "mockObjects/MockSubscriber.h"
#include "mockObjects/MockActionServer.h"

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
