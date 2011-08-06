uint8 motorError

uint8 SUCCESS=0
## Device signals that it is not ready to receive new commands.
uint8 BUSY=1

##Device replied with NACK
uint8 NACK=2

##Time out
uint8 TIMEOUT=3

##RS232 port error. Usually port disconnected
uint8 RS232=4
	
##Invalid API usage. Read documentation
uint8 API=5

#Unexpected responce during handshake. Please disconnect and
#reconnect to resyncronize gateway
uint8 RESYNC=6

#Responce from device violates protocol. Check for changes in
#updated firmware
uint8 PROTOCOL=7
