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
#ifndef MOCK_SERVICE_SERVER_H
#define MOCK_SERVICE_SERVER_H

#include "gmock/gmock.h" 
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"


/**
 * An pure abstract interface to a service server.
 */
 
template <typename MReq, typename MRes>
class SimpleServiceServer {
	private:
	/**
	 * ROS Node Handle.
	 */
	ros::NodeHandle _nh;	
	
	/**
	 * ROS Service server.
	 */
	 ros::ServiceServer _server;
	 
	 /**
	  * The service name.
	  */
	 std::string _serviceName;
	 
	 public:
	/**
	 * The object constructor.
	 * @param name the name of the service that the tested server
	 * 				is being called.
	 */
	SimpleServiceServer(std::string name);
	
	/**
	 * Destructor
	 */
   virtual ~SimpleServiceServer() {};
   
  /**
	* Abstract function to serviceServer callback
	*/	
  virtual bool serviceServerCallback(MReq& rq, MRes& rs) = 0;
  
};

template <typename MReq, typename MRes>
SimpleServiceServer<MReq, MRes>::SimpleServiceServer(std::string name) 
{
	_serviceName = name;
	_server = _nh.advertiseService (_serviceName, &SimpleServiceServer::ServiceServerCallback,this);
	
}


/**
 * The Google Mock Server.
 */
template <typename MReq, typename MRes>
class MockServiceServer : public SimpleServiceServer<MReq, MRes> {
	
	public:
	
	MockServiceServer(std::string serviceName):SimpleServiceServer<MReq, MRes>(serviceName) {};
	
	MOCK_METHOD2_T(serviceServerCallback, bool(MReq& rq, MRes& rs));
};

#endif
	


    
	 
	 
	

