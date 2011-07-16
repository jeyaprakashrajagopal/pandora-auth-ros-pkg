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
