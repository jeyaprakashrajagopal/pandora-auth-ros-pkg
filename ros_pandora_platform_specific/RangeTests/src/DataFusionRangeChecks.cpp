#include "rangeTests.h"

#include "dataFusion_communications/fusionDataMsg.h"


/**
 * Testing value ranges for dataFusion
 */	
typedef dataFusion_communications::fusionDataMsg  dataFusionMsg;


class DataFusionRangeChecks {

	MockSubscriber<dataFusionMsg> fusedDataSubscriber;
		
	public:
		DataFusionRangeChecks(): fusedDataSubscriber("/dataFusion/fusionData") {
			
			EXPECT_CALL(fusedDataSubscriber, subscriberActualCallback(
					AllOf(	
							Pointee(Field(&dataFusionMsg::sensorId, IsBetween(0,11))),
							Pointee(Field(&dataFusionMsg::probabilities, IsBetween(0,1)))
						))).Times(AtLeast(0));	
			
		}
		
	};
