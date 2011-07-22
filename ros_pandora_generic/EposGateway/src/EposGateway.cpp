#include <EposGateway.h>

namespace epos{

EposGateway::EposGateway(void){
	initialized=false;
	pthread_mutex_init(&gatewayMutex,NULL);
}

EposGateway::EposGateway(const std::string& device,unsigned int baudRate,unsigned int timeout){
	initialized=false;
	pthread_mutex_init(&gatewayMutex,NULL);
	int status=this->port.open(device,baudRate,timeout);
	if(!status){
		std::cerr<<"error: EposGateway: cannot open serial port"<<std::endl;
		return;
	}
	//status=getSoftwareVersion();
	if(!status){
		std::cerr<<"error: EposGateway: cannot communicate with epos on port "<<device<<std::endl;
		return;
	}
	initialized=true;
}

status EposGateway::getACK(void){
	char c;
	int n=port.read(c);
	if(n==0)		//Timeout
		return TIMEOUT;
	else if(n==1 && c=='F') //NACK
		return NACK;
	else if(n==1 && c!='O') //Should never happen, violates EPOS Communication Guide
		return RESYNC;
	else if(n!=1)		//RS232 error
		return RS232;
	return SUCCESS;
}

status EposGateway::ACK(void){
	char c='O';
	int n=port.write(c);
	if(n!=1) return RS232;
	return SUCCESS;
}

status EposGateway::sendFrame(unsigned char opCode,word *data,unsigned short length,word *response){
	//#define DEBUG_EposGateway
	if(!initialized){
		std::cerr<<"error: EposGateway: sendFrame: cannot send frame, EPOS is not connected. Please use EposGateway::connect"<<std::endl;
		return API;
	}
	if(length>255 || !length){
		std::cout<<"error: EposGateway: sendFrame: invalid frame data size: "<<length<<std::endl;
		return API;
	}
	
	pthread_mutex_lock(&gatewayMutex);
	//Calculate CRC
	word crcData[length+1];
	crcData[0]=(opCode<<8)|(length-1);
	//EPOS CRC calculation requires data to be Big-Endian (MSB first)
	for(unsigned int i=1;i<length+1;++i){
		crcData[i]=((data[i-1]/256)<<8)|(data[i-1]%256);
	}
	word crc=crc16CCITT(&crcData[0],length+1);

	//Frame data are sent LSB first
	char outData[2*length];
	for(unsigned int i=0;i<length;++i){
		outData[2*i]=data[i]%256;
		outData[2*i+1]=data[i]/256;
	}

	//Communication variables
	char c;
	int n;
	char buffer[128];
	status state;

	#ifdef DEBUG_EposGateway
	printf("0x%02X, ",opCode&0xFF);
	#endif

	//  === State 1: Send opCode ===
	if(port.write(opCode)!=1){
		std::cerr<<"error: EposGateway: sendFrame: unexpected RS232 error while attempting to send op code"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	//  === State 2: Wait for ACK ===
	state=getACK();
	if(state==NACK){
		std::cerr<<"warning: EposGateway: sendFrame: gateway busy"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return BUSY;
	}
	if(state!=SUCCESS){
		std::cerr<<"error: EposGateway: sendFrame: unexpected error while attempting to send op code"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return state;
	}
	
	#ifdef DEBUG_EposGateway
	printf("0x%02X, ",(length-1)&0xFF);
	#endif
	// === State 3: Send len-1 ===
	if(port.write(length-1)!=1){
		std::cerr<<"error: EposGateway: sendFrame: unexpected RS232 error while attempting to send data length"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	#ifdef DEBUG_EposGateway
	for(int temp=0;temp<(length<<1);++temp){
		printf("0x%02X, ",outData[temp]&0xFF);		
	}
	#endif
	// === State 4: Send data ===
	n=port.write(outData,2*length);
	if(n!=(2*length)){
		std::cerr<<"error: EposGateway: sendFrame: RS232 didn't finish writing data frame"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}

	#ifdef DEBUG_EposGateway
	printf("0x%02X, 0x%02X\n",buffer[0]&0xFF,buffer[1]&0xFF);		
	#endif	
	// === State 5: Send CRC ===
	buffer[0]=crc%256;
	buffer[1]=crc/256;
	n=port.write(buffer,2);
	if(n!=2){
		std::cerr<<"error: EposGateway: sendFrame: unexpected RS232 error while attempting to send CRC16"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	// === State 6: Wait for ACK ===
	state=getACK();
	if(state!=SUCCESS){
		std::cerr<<"error: EposGateway: sendFrame: unexpected error while waiting for end ACK"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return state;
	}
	// Command Sent Successfully

	//  === State 7: Wait for responce opCode ===
	n=port.read(c);
	if(n==0){
		std::cerr<<"error: EposGateway: sendFrame: responce from gateway timed-out"<<std::endl;		
		pthread_mutex_unlock(&gatewayMutex);
		return TIMEOUT;
	}else if(n==1 && c!=0x00){
		std::cerr<<"error: EposGateway: sendFrame: unexpected responce from gateway. Resyncronization needed."<<std::endl;		
		pthread_mutex_unlock(&gatewayMutex);
		return RESYNC;
	}else if(n!=1){
		std::cerr<<"error: EposGateway: sendFrame: unexpected RS232 error while waiting for responce opCode from gateway"<<std::endl;
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}

	// === State 8: Send ACK ===
	state=ACK();
	if(state!=SUCCESS){
		pthread_mutex_unlock(&gatewayMutex);
		return state;
	}
	// === State 9: Receive len-1 ===
	n=port.read(c);
	if(n==0){
		pthread_mutex_unlock(&gatewayMutex);
		return TIMEOUT;
	}
	if(n!=1){
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	#ifdef EPOS_IN
	printf("%0.2X",c&0xFF);
	#endif
	int size=((unsigned int)(c))+1;
	if(size>63){
		pthread_mutex_unlock(&gatewayMutex);
		return PROTOCOL;
	}
	// === State 10: Receive data ===
	size<<=1;
	char inData[256];
	n=port.read(&inData[0],size);
	if(n==0){
		pthread_mutex_unlock(&gatewayMutex);
		return TIMEOUT;
	}
	if(n!=size){
		pthread_mutex_unlock(&gatewayMutex);
		return RESYNC;
	}
	#ifdef EPOS_IN
	for(unsigned int i=0;i<size;++i){
		printf("%0.2X",inData[i]&0xFF);
	}
	#endif
	// === State 11: Receive CRC ===
	n=port.read(c);
	if(n==0){
		pthread_mutex_unlock(&gatewayMutex);
		return TIMEOUT;
	}
	if(n!=1){
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	n=port.read(c);
	if(n==0){
		pthread_mutex_unlock(&gatewayMutex);
		return TIMEOUT;
	}
	if(n!=1){
		pthread_mutex_unlock(&gatewayMutex);
		return RS232;
	}
	#ifdef EPOS_IN
	printf("%0.2X",buffer[0]&0xFF);	
	printf("%0.2X\n",buffer[1]&0xFF);
	#endif
	// === State 12: Send ACK or NACK ===
	state=ACK();
	for(unsigned int i=0;i<(size/4);++i){
		uint16_t MSB,LSB;
		LSB=((inData[4*i+1]&0xFF)<<8)|(inData[4*i]&0xFF);
		MSB=((inData[4*i+3]&0xFF)<<8)|(inData[4*i+2]&0xFF);
		response[i]=(MSB<<16)|LSB;
	}
	if(state!=SUCCESS){
			pthread_mutex_unlock(&gatewayMutex);
			return state;
	}
	pthread_mutex_unlock(&gatewayMutex);
	return SUCCESS;

}

status EposGateway::readObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,word *responce){
	word data[2];
	data[0]=index;
	data[1]=(nodeID<<8)|subIndex;
	return sendFrame(0x10,&data[0],2,responce);
}

status EposGateway::writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,dWord data){
	word internalData[4];
	word dataHigh,dataLow;
	uint32_t rawData=(uint32_t)data;
	dataHigh=rawData/65536;
	dataLow=rawData%65536;
	
	internalData[0]=index;
	internalData[1]=(nodeID<<8)|subIndex;
	internalData[2]=dataLow;
	internalData[3]=dataHigh;
	word ret[2];
	return sendFrame(0x11,internalData,4,&ret[0]);
}

status EposGateway::writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,word data){
	word internalData[4];
	internalData[0]=index;
	internalData[1]=(nodeID<<8)|subIndex;
	internalData[2]=data;
	internalData[3]=0;
	word ret[2];
	return sendFrame(0x11,internalData,4,&ret[0]);
}

status EposGateway::writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,char data){
	word internalData[4];
	word wordData=0;
	wordData|=data;
	internalData[0]=index;
	internalData[1]=(nodeID<<8)|subIndex;
	internalData[2]=wordData;
	internalData[3]=0;
	word ret[2];
	return sendFrame(0x11,internalData,4,&ret[0]);
}


word EposGateway::crc16CCITT(word* data,unsigned int length){
	//Append a zero-word to packet
	uint16_t *packet,*temp;
	temp=packet=new uint16_t[length+1];
	memcpy(packet,data,length*2);
	packet[length]=0;
	++length;
	//Calculate CRC using polyonym: x^16+x^12+x^5+x^0
	uint16_t shifter,c;
	uint16_t carry;
	uint16_t CRC=0;
	while(length--){
		shifter = 0x8000;                 //Initialize BitX to Bit15
		c = *packet++;                //Copy next Data uint16_t to c
		do{
			carry=CRC&0x8000;    //Check if Bit15 of CRC is set
			CRC<<=1;               //CRC = CRC * 2
			if(c&shifter) ++CRC;   //CRC = CRC + 1, if BitX is set in c
			if(carry) CRC^=0x1021; //CRC = CRC XOR G(x), if carry is true
			shifter>>=1;           //Set BitX to next lower Bit, shifter = shifter/2
		}while(shifter);
	}
	delete[] temp;
	word ret;
	memcpy(&ret,&CRC,2);
	return ret;
}

}
