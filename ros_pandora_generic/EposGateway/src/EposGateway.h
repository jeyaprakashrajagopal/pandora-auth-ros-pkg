#ifndef EposGateway_H
#define EposGateway_H


#include <iostream>
#include <string>

#include <stdint.h>

#include <pthread.h>

#include <Rs232.h>

namespace epos{

enum status{
	SUCCESS=0, //Command was successful
	BUSY, //device is busy
	NACK, //received NACK
	TIMEOUT, //command timemed-out
	RS232, //Rs232 port error
	API, //invalid api usage
	RESYNC, //resyncronization required. disconnect/connect
	PROTOCOL //protocol error
};

typedef uint16_t word;
typedef uint32_t dWord;

class EposGateway{
	Rs232 port;
	pthread_mutex_t gatewayMutex;
	word crc16CCITT(word* data,unsigned int length);
	bool initialized;
	status getACK(void);
	status ACK(void);

public:
	EposGateway(void);
	EposGateway(const std::string& device,unsigned int baudRate=38400,unsigned int timeout=500);
	int getSoftwareVersion(void);
	status sendFrame(unsigned char opCode,word *data,unsigned short length,word *response);
	status readObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,dWord *responce);
	status readObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,word *responce);
	status readObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,char *responce);
	status writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,dWord data);
	status writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,word data);
	status writeObject(unsigned char nodeID,uint16_t index,unsigned char subIndex,char data);
};

}

#endif
