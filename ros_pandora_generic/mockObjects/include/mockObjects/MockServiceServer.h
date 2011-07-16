#ifndef MOCK_SERVICE_SERVER_H
#define MOCK_SERVICE_SERVER_H

#include "gmock/gmock.h"  // Brings in Google Mock.
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
   * Declaration of service callback
   */
  virtual bool ServiceServerCallback(MReq& rq, MRes& rs) = 0;
  
};

template <typename MReq, typename MRes>
SimpleServiceServer<MReq, MRes>::SimpleServiceServer(std::string name) 
{
	_serviceName = name;
	_server = _nh.advertiseService (_serviceName, &SimpleServiceServer::ServiceServerCallback,this);
	
}


/**
 * The Mock Server.
 */
template <typename MReq, typename MRes>
class MockServiceServer : public SimpleServiceServer<MReq, MRes> {
	
	public:
	
	MockServiceServer(std::string serviceName):SimpleServiceServer<MReq, MRes>(serviceName) {};
	
	MOCK_METHOD2_T(ServiceServerCallback, bool(MReq& rq, MRes& rs));
};

#endif
	


    
	 
	 
	

