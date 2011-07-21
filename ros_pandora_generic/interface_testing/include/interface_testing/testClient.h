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
  * File Description testClient implementation
  * for remote calling of node tests
  * Testing of Subscribing and Publishing Msgs
  * Author: Group 0
  * Date: 28 February 2011
  * Change History: - 
  */

#ifndef CLIENT_TEST
#define CLIENT_TEST


#include "actionlib/client/simple_action_client.h"
#include "interface_testing/testerAction.h"

typedef actionlib::SimpleActionClient<interface_testing::testerAction> ClientTester;

class TestClient {
	private:
		ClientTester _clientTester;		
		ros::NodeHandle _nh;
		
	public:
		/**
		* Constructor
		* @param the test's actionName
		*/
		TestClient(std::string actionName);
		
		/**
		* Method for selecting the specific node
		* to be tested
		* @param the name of the node to be tested
		*/
		static void testNode(std::string nodeName);
		
};

#endif
