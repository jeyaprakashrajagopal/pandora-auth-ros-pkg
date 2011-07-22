#include <iostream>

#include <cstdio>

#include "ros/ros.h"
#include "mainMotorControl_communications/setVehicleSpeedSrv.h"
#include "mainMotorControl_communications/mainMotorStateMsg.h"
#include "mainMotorControl_communications/mainMotorStateMsg.h"

#include <EposGateway.h>
#include <Kinematic.h>
#include <fstream>


#include <SubscriberTester.h>
#include <PublisherTester.h>

#include "diagnostic_updater/diagnostic_updater.h"


class mainMotorController 
{
	
	//declaration of testers
	PublisherTester<mainMotorControl_communications::mainMotorStateMsg> testMainMotorState;
	std::ofstream outfile;
	epos::EposGateway m_gateway;
	ros::Publisher *m_publisher;
	ros::Publisher m_motorState;
	ros::NodeHandle m_handle;
	Kinematic *m_robotKinematic;
	bool m_isInit;
	epos::status motorStatus;
	
	//For Diagnostics about motor status and setVehicleSpeed Calls
	ros::Time lastSetSpeedCall;
	bool falseSpeedCall;
	ros::Duration timeBetweenCalls;
	
	/**
	* The diagnostic updater
	*/		
	diagnostic_updater::Updater _updater;
	
public:

	mainMotorController(
		std::string dev,
		int speed,
		int timeout,
		ros::Publisher *p,
		ros::NodeHandle &handle);
		
	bool setSpeed(
		mainMotorControl_communications::setVehicleSpeedSrv::Request &req,
		mainMotorControl_communications::setVehicleSpeedSrv::Response &res);
		
	void postStatus(const ros::TimerEvent& event);
	
	inline bool isInit(void){ return m_isInit; }
	void speedDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
	void statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
	
	std::string convertStatusToString(epos::status);
};

/** set vehicle speed
 * 
 * sets the vehicle speed.
 */
bool mainMotorController::setSpeed(
	mainMotorControl_communications::setVehicleSpeedSrv::Request &req,
	mainMotorControl_communications::setVehicleSpeedSrv::Response &res){
	
	if(!m_isInit){
		
		m_robotKinematic=new KinematicSimple;
		m_isInit=true;
	}
	timeBetweenCalls = ros::Time::now()-lastSetSpeedCall;
	if(timeBetweenCalls<ros::Duration(.1)) {
		ROS_ERROR("[mainMotorControl]: setVehicleSpeed called too frequently. Cannot respond!");
		falseSpeedCall = true;
		return false;
	}
	falseSpeedCall = false;
	lastSetSpeedCall = ros::Time::now();
	
	ROS_INFO("mainMotorControl: got speed, linear: '%f', rotational: '%f'",
		req.linearSpeed,req.rotationalSpeed);
	
	//Robot kinematic velocity structure
	Velocity velocity;
	velocity.vx=req.linearSpeed;
	velocity.vy=0; //we dont want our robot to slide
	velocity.w=-req.rotationalSpeed;
	
	//Get required robot rpms

	ControlInput controlInput=m_robotKinematic->getControlInput(velocity);
	
	//speeds in rpm
	
	int leftSpeed=controlInput.rpml;
	int rightSpeed=-controlInput.rpmr;
	ROS_INFO("setting speed %d, %d",leftSpeed,rightSpeed);

	//Limit robot speed
	
	if(leftSpeed>5500){
		
		ROS_ERROR("mainMotorControl: robot speed to high: '%d rpm', \
limiting to 5500 rpm",leftSpeed);

		leftSpeed=5500;
	}
	if(leftSpeed<-5500){

		ROS_ERROR("mainMotorControl: robot speed to high: '%d rpm', \
limiting to -5500 rpm",leftSpeed);

		leftSpeed=-5500;
	}
	if(rightSpeed>5500){

		ROS_ERROR("mainMotorControl: robot speed to high: '%d rpm', \
limiting to 5500 rpm",rightSpeed);

		rightSpeed=5500;
	}
	if(rightSpeed<-5500){

		ROS_ERROR("mainMotorControl: robot speed to high: '%d rpm', \
limiting to -5500 rpm",rightSpeed);

		rightSpeed=-5500;
	}
	
	int signLeft=leftSpeed<0 ? 1 : 0;
	int signRight=rightSpeed<0 ? 1 : 0;
	
	int leftSpeedAbsolute=abs(leftSpeed);
	int rightSpeedAbsolute=abs(rightSpeed);
	
	
	uint32_t controlWord=0;
	
	controlWord=(1<<31) | (signRight<<30) | (rightSpeedAbsolute<<16) | (signLeft<<14) | (leftSpeedAbsolute);
	
	res.error.motorError=0;
	
	epos::status status;
	
	#define MIKE
	
	#ifndef MIKE
	
		//Kourus code
		
		epos::dWord leftMotorVelocityWord=(epos::dWord)(rightSpeed+5000); 
		epos::dWord rightMotorVelocityWord=(epos::dWord)(leftSpeed+5000);
		
		ROS_INFO("mainMotorControl: setting motor speed, left: '%d rpm', \
	right: '%d rpm'",leftSpeed,rightSpeed);

		epos::dWord startMotionFlag=1;
		
		status=m_gateway.writeObject(2,0x200C,1,leftMotorVelocityWord);
		motorStatus = status;
	#else
	
		//Mike code
		status=m_gateway.writeObject(2,0x200C,1,controlWord);
		motorStatus = status;
	#endif
	
	if(status!=epos::SUCCESS){
		ROS_ERROR("error setting speed left");
		res.error.motorError=status;
		return true;
	}
	
	#ifndef MIKE

		status=m_gateway.writeObject(2,0x200C,2,rightMotorVelocityWord);
		motorStatus = status;
		if(status!=epos::SUCCESS){
			ROS_ERROR("error setting speed right");
			res.error.motorError=status;
			return true;
		}
		
		status=m_gateway.writeObject(2,0x200C,3,startMotionFlag);
		motorStatus = status;
		if(status!=epos::SUCCESS){
			ROS_ERROR("error setting speed strobe");
			res.error.motorError=status;
			return true;
		}
	
	#endif
	
	
	return true;
}


std::string mainMotorController::convertStatusToString(epos::status st){
	if(st==epos::SUCCESS)
		return "SUCCESS";
	else if (st==epos::BUSY)
		return "BUSY";
	else if (st==epos::NACK)
		return "NACK";
	else if (st==epos::TIMEOUT)
		return "TIMEOUT";
	else if (st==epos::RS232)
		return "RS232";
	else if (st==epos::API)
		return "API";
	else if (st==epos::RESYNC)
		return "RESYNC";
	else if (st==epos::PROTOCOL)
		return "PROTOCOL";
	else return "INVALID";
}


mainMotorController::mainMotorController (
	std::string dev,
	int speed,
	int timeout,
	ros::Publisher *p,
	ros::NodeHandle &handle) : 
	testMainMotorState ("status", "testMainMotorStatePublish")
	{

	outfile.open("/home/pandora/mainmotors.csv");


	m_isInit=false;
	m_handle=handle;
	m_publisher=p;
	m_gateway=epos::EposGateway(dev,speed,timeout);
	epos::word ret[2];
	

	//reset errors
	m_gateway.readObject(2,0x6040,0,&ret[0]);
	ret[1]|=(1<<7);
	m_gateway.writeObject(2,0x6040,0,ret[1]);
	m_motorState=
		handle.advertise<mainMotorControl_communications::mainMotorStateMsg>(
			"/mainMotorControl/status",1);
	

	epos::status status=m_gateway.readObject(2,0x6040,0,&ret[0]);
	if(status!=epos::SUCCESS){
		ROS_ERROR("error: No EPOS device detected on selected serial interface");
		return;
	}

	
	try{
		m_robotKinematic=new KinematicSimple;
	}catch(std::bad_alloc &e){
		ROS_ERROR("mainMotorControl: cannot allocate memory for robot kinematic");
		return;
	}
	m_isInit=true;
	
	lastSetSpeedCall = ros::Time::now();
	falseSpeedCall = false;
	_updater.setHardwareID("EPOS");
	_updater.add("Motor Speed", this, &mainMotorController::speedDiagnostic);
	_updater.add("Motor Status", this, &mainMotorController::statusDiagnostic);
}

void mainMotorController::speedDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if (!falseSpeedCall) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Vehicle Speed OK");		
		}
	else {
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, 
		"Set Vehicle Speed Cannot Respond, called too frequently ");
		}
	}
	
void mainMotorController::statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
	if(motorStatus==0) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Status == SUCCESS");
		}
	else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor Status == " + convertStatusToString(motorStatus));
		}
	}

void mainMotorController::postStatus(const ros::TimerEvent& event){

	if(!m_isInit){
		return;
	}

	epos::word ret[2];
	
	int32_t velocityLeft,velocityRight;
	int16_t currentLeft,currentRight;
	
	m_gateway.readObject(2,0x2027,0,&ret[0]);
	currentLeft=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x2030,0,&ret[0]);
	currentRight=-(int16_t)ret[1];

	m_gateway.readObject(2,0x2028,0,&ret[0]);
	velocityLeft=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x206B,0,&ret[0]);
	velocityRight=(int32_t)ret[1];
	if(velocityRight>10000) velocityRight-=20000;
	
	velocityRight=-velocityRight;
	
	
	Kinematic *kinematic=new KinematicSimple;
	ControlInput motorRpm;
	motorRpm.rpml=velocityLeft;
	motorRpm.rpmr=velocityRight;
	Velocity velocity=kinematic->getVelocity(motorRpm);
	
	
	mainMotorControl_communications::mainMotorStateMsg stateMsg;
	stateMsg.stateLeft=0;
	stateMsg.stateRight=0;
	stateMsg.currentLeft=currentLeft;
	stateMsg.currentRight=currentRight;
	stateMsg.velocityLeft=velocityLeft;
	stateMsg.velocityRight=velocityRight;
	stateMsg.velocityLinear=velocity.vx;
	stateMsg.velocityAngular=velocity.w;
	stateMsg.error.motorError=mainMotorControl_communications::motorErrorMsg::SUCCESS;
	
	//Added by software architecture team for testing
	//DO NOT DELETE
	testMainMotorState.checkoutMsg(stateMsg);
	
	
	//Publish message
	m_motorState.publish(stateMsg);
	outfile<<currentLeft<<','<<currentRight<<','<<velocityLeft<<','<<velocityRight<<std::endl;
	
	_updater.update();
	
	/*
	int16_t statusWord,controllWord;
	int32_t internalState;
	uint8_t errorRegister;
	uint32_t errorWord;



	m_gateway.readObject(2,0x206B,0,&ret[0]);
	velocityLeft=ret[1];
	
	m_gateway.readObject(2,0x6069,0,&ret[0]);
	velocityRight=ret[1];
	
	Kinematic *kinematic=new KinematicSimple;
	ControlInput motorRpm;
	motorRpm.rpml=velocityLeft;
	motorRpm.rpmr=velocityRight;
	Velocity velocity=kinematic->getVelocity(motorRpm);
	

	m_gateway.readObject(2,0x2071,3,&ret[0]);
	statusWord=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x2071,2,&ret[0]);
	controllWord=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x6041,0,&ret[0]);
	int16_t statusWordEpos=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x6040,0,&ret[0]);
	int16_t controllWordEpos=(int16_t)ret[1];
	
	m_gateway.readObject(2,0x200c,4,&ret[0]);
	internalState=(int32_t)ret[1]+ret[0]*(1<<16);
	
	m_gateway.readObject(2,0x2071,4,&ret[0]);
	errorRegister=(uint8_t)ret[1];
	
	m_gateway.readObject(2,0x607C,0,&ret[0]);
	errorWord=(uint32_t)ret[1];

	switch(internalState){
		case 0:
			ROS_INFO("Initialisation (EPOS disabled)");
			break;
		case 1:
			ROS_INFO("Reset1(reset 1st motor)");
			break;
		case 2:
			ROS_INFO("Reset2");
			break;
		case 3:
			ROS_INFO("Power1");
			break;
		case 4:
			ROS_INFO("Power2");
			break;
		case 5:
			ROS_INFO("Get Indicator(Read flag 0->1)");
			break;
		case 6:
			ROS_INFO("GetVelocity1");
			break;
		case 7:
			ROS_INFO("GetVelocity2");
			break;
		case 8:
			ROS_INFO("Move");
			break;
		case 9:
			ROS_INFO("SetIndicator(if movement complete, flag=0)");
			break;
		case 10:{
			ROS_INFO("SetIndicator(if movement complete, flag=0)");
			switch(errorWord){
				case 0x1:
					ROS_INFO("Internal function block sequence error");
					break;
				case 0xF000FFC0:
					ROS_INFO("The device is in wrong NMT state");
					break;
				case 0x0FFFFFF0:
					ROS_INFO("CAN Communication sequence error");
					break;
				case 0x0FFFFFF1:
					ROS_INFO("Communication aborted by CAN Driver");
					break;
				case 0x0FFFFFF2:
					ROS_INFO("Communication buffer overflow");
					break;
				case 0x0FFFFFF9:
					ROS_INFO("Segmented transfer communication error");
					break;
				case 0x0FFFFFFA:
					ROS_INFO("Wrong Axis number");
					break;
				case 0x0FFFFFFB:
					ROS_INFO("Wrong Device number");
					break;
				case 0x0FFFFFFC:
					ROS_INFO("Wrong CAN port");
					break;
				case 0x0FFFFFFD:
					ROS_INFO("Bad function calling parameters");
					break;
				case 0x0FFFFFFE:
					ROS_INFO("General CAN Communication Error");
					break;
				case 0x0FFFFFFF:
					ROS_INFO("CAN communication time out");
					break;
				default:
					ROS_WARN("See leaflet for more");
			}
			
			switch(errorRegister){
				case 0b00000001:
					ROS_INFO("Generic error");
					break;
				case 0b00000010:
					ROS_INFO("OverCurrent Error");
					break;
				case 0b00000100:
					ROS_INFO("Voltage Limit Error");
					break;
				case 0b00100000:
					ROS_INFO("Softw.Error OR Sensor Pos.Error OR Follow.Error or System Overload");
					break;
				case 0b00010000:
					ROS_INFO("CAN broblem");
					break;
				case 0b10000000:
					ROS_INFO("Hall Sensor OR encoder Error");
					break;
				default:
					ROS_WARN("Unknown Error");
			}
						
			break;
		}
		default:
			ROS_WARN("Unknown motor state");
	}


	
	
	

	ROS_INFO("mainMotorControl: EPOS P status word,controll word, internal state: \
%d %d %d",statusWord, controllWord, internalState);

	ROS_INFO("mainMotorControl: EPOS status word,controll word: \
%d %d",statusWordEpos, controllWordEpos);
*/

	/*
	ROS_INFO("current left,right: %d, %d",currentL,currentR);
	ROS_INFO("velocity left,right: %d, %d",velocityL,velocityR);
	*/
}

int main(int argc,char **argv){
	ros::init(argc,argv,"epos");
	
	ros::NodeHandle n;
	ros::Publisher publisher;/*=
		n.advertise<mainMotorControl_communications::mainMotorStateMsg>(
			"mainMotorState",1);*/

	mainMotorController controller("/dev/ttyS0",115200,500,&publisher,n);

	ros::ServiceServer service=n.advertiseService(
		"setVehicleSpeed",
		&mainMotorController::setSpeed,
		&controller);

	ROS_INFO("mainMotorControl: started service epos::setVehicleSpeedSrv");

	double period;
	n.getParam("/mainMotorControl/statusPostPeriod", period);
	ros::Timer timer;
	if(period<0.1){

		ROS_ERROR("mainMotorControl: invalid value for parameter \
'/mainMotorControl/statusPostPeriod': '%f'",period);

	}else{
		
		timer= n.createTimer(
			ros::Duration(period),
			&mainMotorController::postStatus,
			&controller);
			
		ROS_INFO("mainMotorControl: started message mainMotorStateMsg, \
with statusPostPeriod: '%f'",period);

	}

	ros::spin();

	if(controller.isInit()){

		ROS_INFO("mainMotorControl: stopping service epos::setVehicleSpeed");
		ROS_INFO("mainMotorControl: stopping robot!");

		mainMotorControl_communications::setVehicleSpeedSrv::Request stopRequest;
		mainMotorControl_communications::setVehicleSpeedSrv::Response stopResponse;

		stopRequest.linearSpeed=0;
		stopRequest.rotationalSpeed=0;

		int count=0;
		do{
			controller.setSpeed(stopRequest,stopResponse);
			usleep(20000);
			++count;
		}while(stopResponse.error.motorError!=0 && count<100);

		if(count!=100)
			ROS_INFO("robot stoped!");
		else
			ROS_ERROR("mainMotorControl: cannot stop robot!");

	}


	return 0;
}

