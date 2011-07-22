#ifndef ABSTRACTEPOSGATEWAY_H
#define ABSTRACTEPOSGATEWAY_H

#include <stdint.h>

namespace epos{

/// EPOS word (16bit)
typedef uint16_t Word;
/// Double (long) EPOS word (32bit)
typedef uint32_t DWord;
/// Short EPOS word (8bit)
typedef unsigned char Byte;

/**
 * \brief enumeration of error codes during device communication
 * 
 * For communication with the device, a protocol is implemented. In the
 * event that the communication was successful, each protocol command
 * returns CommandStatus::SUCCESS. 
 * 
 */
enum CommandStatus{
	/**
	 * \brief Command sent successfully
	 * 
	 * This status code is returned by the implementation in the event
	 * of a successful transaction with the device. Please, note that
	 * this doesn't mean the command execution was successful, only the
	 * data transfer was. In some cases the device my return error codes
	 * using the response data, signifying the result of the command
	 * execution. In the event that a transaction does not return SUCCESS
	 * the returned data are undefined.
	 * 
	 */
	SUCCESS=0,
	/**
	 * \brief Device busy
	 * 
	 * This error code is returned if the device signals that it is not
	 * ready to receive new commands.
	 * 
	 */
	BUSY,
	///Device replied with NACK
	NACK,
	///Time out
	TIMEOUT,
	///RS232 port error. Usually port disconnected
	RS232,
	///Invalid API usage. Read documentation
	API,
	///Unexpected responce during handshake. Please disconnect and
	///reconnect to resyncronize gateway
	RESYNC,
	///Responce from device violates protocol. Check for changes in
	///updated firmware
	PROTOCOL
};

	
}

/**
 * \brief Abstract gateway for EPOS devices
 * 
 * All gateways for EPOS (P) devices must implement this class to
 * provide compatibility with the EPOSController class
 * 
 * \author Charalampos Serenis
 * \author Electical and Computer Engineer
 * \author Department of Electrical and Computer Engineering
 * \author Aristotle University of Thessaloniki, Greece
 * 
 */


class AbstractEposGateway{
public:

	virtual epos::CommandStatus sendFrame( unsigned char opCode,
		epos::Word *data,
		unsigned short length,
		epos::Word *response,
		unsigned short& responseSize)=0;

	/** Read object dictionary entry (4 Data Bytes and less)
	*
	* Read an object value at the given Index and SubIndex from the
	* Object Dictionary.
	*
	* \param[in] nodeID the node ID of the device from which to read
	* the data
	* \param[in] index the index of the Object Dictionary we wish to
	* read
	* \param[in] subIndex the sub-index of the Dictionary we wish to
	* read
	* \param[out] error the returned error code from the device
	* \param[out] data the data read
	* \return the status of the operation
	*/
	virtual epos::CommandStatus readObject( unsigned char nodeID,
		uint16_t index,unsigned char subIndex,epos::DWord& error,
		epos::DWord& data)=0;
	
	/** Initiate read of object dictionary entry (5 Data Bytes and more)
	*
	* Start reading an object value at the given Index and SubIndex from
	* the Object Dictionary. Use the command ‘SegmentRead’ to read the
	* data.
	*
	* \param[in] nodeID the node ID of the device from which to read the
	* data
	* \param[in] index the index of the Object Dictionary we wish to
	* read
	* \param[in] subIndex the sub-index of the Dictionary we wish to
	* read
	* \param[out] error the returned error code from the device
	* \return the status of the operation
	*/	
	virtual epos::CommandStatus initiateSegmentedRead(
		unsigned char nodeID,
		uint16_t index,
		unsigned char subIndex,
		epos::DWord& error)=0;

	/** Get data of initialized segment read of object dictionary entry
	* (5 Data Bytes and more)
	*
	* Read a data segment of the object initiated with the command
	* initiateSegmentedRead.
	*
	* \param[in,out] toggle the value of the toggle bit
	* \param[out] readDataLength the length of the data that were read
	* \param[out] data the read data.  The pointer must point to an
	* array
	* of epos::Word elements, large enough to store the read data.
	* \param[out] error the returned error code from the device
	* \return the status of the operation
	*/	
	virtual epos::CommandStatus segmentRead( bool& toggle,
		unsigned char& readDataLength,
		epos::DWord& error,
		epos::Word *data)=0;

	/** Write Object Dictionary Entry (4 Data Bytes and less)
	*
	* Write an object value to the given Index and SubIndex from the
	* Object Dictionary.
	*
	* \param[in] nodeID the node ID of the device from which to read the
	* data
	* \param[in] index the index of the Object Dictionary we wish to
	* read
	* \param[in] subIndex the sub-index of the Dictionary we wish to
	* read
	* \param[in] data the data that are going to be written
	* \param[out] error the returned error code from the device
	* \return the status of the operation
	*/	
	virtual epos::CommandStatus writeObject( unsigned char nodeID,
		uint16_t index,
		unsigned char subIndex,
		epos::DWord data,
		epos::DWord& error)=0;
	
	/** Initiate write of Object Dictionary Entry (5 Data Bytes and
	* more)
	*
	* Start writing an object value to the given Index and SubIndex in
	* the Object Dictionary. Use the command ‘SegmentWrite’ to write the
	* data.
	*
	* \param[in] nodeID the node ID of the device from which to read the
	* data
	* \param[in] index the index of the Object Dictionary we wish to
	* read
	* \param[in] subIndex the sub-index of the Dictionary we wish to
	* read
	* \param[in] dataLength the number of bytes that are going to be
	* written
	* \param[out] error the returned error code from the device
	* \return the status of the operation
	*/		
	virtual epos::CommandStatus initiateSegmentedWrite(
		unsigned char nodeID,
		uint16_t index,
		unsigned char subIndex,
		epos::DWord dataLength,
		epos::DWord& error)=0;	

	/** Initiate write of Object Dictionary Entry (5 Data Bytes and
	* more)
	*
	* Write a data segment to the object initiated with the command
	* initiateSegmentedWrite.
	*
	* \param[in,out] toggle the value of the toggle bit
	* \param[in] data the data that are going to be written
	* \param[out] error the returned error code from the device
	* \return the status of the operation
	*/			
	virtual epos::CommandStatus segmentWrite( bool& toggle,
		const epos::Word *data,
		epos::DWord& error)=0;	
	
	virtual epos::CommandStatus sendNMTService(unsigned char nodeID,unsigned char commandSpecifier)=0;
	virtual epos::CommandStatus sendCANFrame(epos::Word identifier,epos::Word dataLengthCode,unsigned char data)=0;
	virtual epos::CommandStatus requestCANFrame(epos::Word identifier,epos::Word dataLengthCode,unsigned char data)=0;
	
};

#endif
