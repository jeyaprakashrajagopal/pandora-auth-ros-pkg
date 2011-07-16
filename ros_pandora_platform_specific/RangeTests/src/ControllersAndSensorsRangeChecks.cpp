#include "rangeTests.h"

#include "controllersAndSensors_communications/soundConfirmationMsg.h"
#include "controllersAndSensors_communications/soundDirectionMsg.h"
#include "controllersAndSensors_communications/soundErrorMsg.h"
#include "controllersAndSensors_communications/soundExistenceMsg.h"
#include "controllersAndSensors_communications/soundVictimConditionMsg.h"
#include "controllersAndSensors_communications/butterflyMsg.h"
#include "controllersAndSensors_communications/co2Msg.h"
#include "controllersAndSensors_communications/compassMsg.h"
#include "controllersAndSensors_communications/sonarMsg.h"
#include "controllersAndSensors_communications/irMsg.h"
#include "controllersAndSensors_communications/tpaMsg.h"
#include "controllersAndSensors_communications/mlxTempMsg.h"

typedef controllersAndSensors_communications::soundConfirmationMsg  confirmationMsg;
typedef controllersAndSensors_communications::soundDirectionMsg  directionMsg;
typedef controllersAndSensors_communications::soundErrorMsg  errorMsg;
typedef controllersAndSensors_communications::soundExistenceMsg  existenceMsg;
typedef controllersAndSensors_communications::soundVictimConditionMsg  victimConditionMsg;
typedef controllersAndSensors_communications::butterflyMsg  butterfly;
typedef controllersAndSensors_communications::co2Msg  co2;
typedef controllersAndSensors_communications::compassMsg  compass;
typedef controllersAndSensors_communications::irMsg  ir;
typedef controllersAndSensors_communications::sonarMsg  sonar;
typedef controllersAndSensors_communications::tpaMsg  tpa;
typedef controllersAndSensors_communications::mlxTempMsg  mlx;

class ControllersAndSensorsRangeChecks {
	
	MockSubscriber<confirmationMsg> soundConfirmationSubscriber;
	MockSubscriber<directionMsg> soundDirectionSubscriber;
	MockSubscriber<errorMsg> soundErrorSubscriber;
	MockSubscriber<existenceMsg> soundExistenceSubscriber;
	MockSubscriber<victimConditionMsg> soundVictimConditionSubscriber;
	MockSubscriber<butterfly> butterflySubscriber;
	MockSubscriber<co2> co2Subscriber;
	MockSubscriber<compass> compassSubscriber;
	MockSubscriber<sonar> sonarSubscriber;
	MockSubscriber<tpa> tpaSubscriber;
	MockSubscriber<mlx> mlxSubscriber;
	MockSubscriber<ir> irSubscriber;
	
	public:
	
	ControllersAndSensorsRangeChecks() : soundConfirmationSubscriber("/sensors/soundConfirmation"),
													soundDirectionSubscriber("/sensors/soundDirection"),
													soundErrorSubscriber("/sensors/soundError"),
													soundExistenceSubscriber("/sensors/soundExistence"),
													soundVictimConditionSubscriber("/sensors/soundVictimCondition"),
													butterflySubscriber("/sensors/butterfly"),
													co2Subscriber("/sensors/co2"),
													compassSubscriber("/sensors/compass"),
													sonarSubscriber("/sensors/sonar"),
													tpaSubscriber("/sensors/tpa"),
													mlxSubscriber("/sensors/mlxTemp"),
													irSubscriber ("/sensors/ir") {
		
		EXPECT_CALL(soundConfirmationSubscriber, subscriberActualCallback(
				AllOf(	
						Pointee(Field(&confirmationMsg::victimProbability, IsBetween(0,1))),
						Pointee(Field(&confirmationMsg::victimConditionProbalility, IsBetween(0,1)))
					))).Times(AtLeast(0));
		
		EXPECT_CALL(soundDirectionSubscriber, subscriberActualCallback(
				AllOf(	
						Pointee(Field(&directionMsg::angleProbability, IsBetween(0,1))),
						Pointee(Field(&directionMsg::radAngle, IsBetween(0,2*PI))),
						Pointee(Field(&directionMsg::radAngle, stepCheck(0,PI*15/180))),					
						Pointee(Field(&directionMsg::victimProbability, IsBetween(0,1)))
					))).Times(AtLeast(0));
				
				
	
		EXPECT_CALL(soundErrorSubscriber, subscriberActualCallback(	
					Pointee(Field(&errorMsg::soundError, IsBetween(0,1)))
				)).Times(AtLeast(0));
						
				
		EXPECT_CALL(soundExistenceSubscriber, subscriberActualCallback(
				AllOf(
						AnyOf(
							Pointee(Field(&existenceMsg::soundExists, Eq(0))),
							Pointee(Field(&existenceMsg::soundExists, Eq(1)))),
							
						Pointee(Field(&existenceMsg::certainty, IsBetween(0,1)))						
					))).Times(AtLeast(0));			
	
		EXPECT_CALL(soundVictimConditionSubscriber, subscriberActualCallback(
					Pointee(Field(&victimConditionMsg::victimCondition, IsBetween(0,4)))
					)).Times(AtLeast(0));	
		
		EXPECT_CALL(butterflySubscriber, subscriberActualCallback(
				Pointee(Field(&butterfly::voltage, ElementsAre(IsBetween(14,25), IsBetween(14,25))))
					)).Times(AtLeast(0));	
	
		
		
		EXPECT_CALL(co2Subscriber, subscriberActualCallback(
						Pointee(Field(&co2::ppm, AnyOf(
						IsBetween(200,50000), Eq(-1))) 
					))).Times(AtLeast(0));			
		
		
		
		EXPECT_CALL(compassSubscriber, subscriberActualCallback(
				AllOf(
						Pointee(Field(&compass::pitch, IsBetween(-PI/2,PI/2))),
						Pointee(Field(&compass::roll, IsBetween(-PI/2,PI/2))) 		
					))).Times(AtLeast(0));
			
		EXPECT_CALL(irSubscriber, subscriberActualCallback(
					Pointee(Field(&ir::distance, ElementsAre(
							AnyOf(IsBetween(0, 800), Eq(-1)),
							AnyOf(IsBetween(0, 300), Eq(-1)),
							AnyOf(IsBetween(0, 300), Eq(-1)),
							AnyOf(IsBetween(0, 800), Eq(-1)),
							AnyOf(IsBetween(40, 300), Eq(-1))))							
					))).Times(AtLeast(0));	
					
		
		
		EXPECT_CALL(sonarSubscriber, subscriberActualCallback(
					Pointee(Field(&sonar::distance, ElementsAre(
							AnyOf(IsBetween(20, 7000), Eq(-1)),
							AnyOf(IsBetween(20, 7000), Eq(-1)),
							AnyOf(IsBetween(20, 7000), Eq(-1)),
							AnyOf(IsBetween(20, 7000), Eq(-1)),
							AnyOf(IsBetween(20, 7000), Eq(-1)),
							AnyOf(IsBetween(20, 7000), Eq(-1)))) 		
					))).Times(AtLeast(0));		
					
		
		
		EXPECT_CALL(tpaSubscriber, subscriberActualCallback(
				AllOf(
					Pointee(Field(&tpa::ambientTemp, IsBetween(0,120))),
					Pointee(Field(&tpa::pixelTemp, ElementsAre(
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120),			
							IsBetween(10,120))))			
					))).Times(AtLeast(0));	
		
		

		EXPECT_CALL(mlxSubscriber, subscriberActualCallback(
					Pointee(Field(&mlx::mlxTemp, ElementsAre(
							IsBetween(10,100),
							IsBetween(10,100))))			 
				)).Times(AtLeast(0));
			
	};
	
	
};
