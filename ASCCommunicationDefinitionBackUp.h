#define DEBUG_PRINT 1

#ifdef DEBUG_PRINT
	#define _SERIAL_BEGIN(x) Serial.begin(x);
	#define _SERIAL_PRINT(x) Serial.print(x);
	#define _SERIAL_PRINT_HEX(x) Serial.print(x, HEX);
	#define _SERIAL_PRINTLN(x) Serial.println(x);
	#define _SERIAL_PRINTLN_HEX(x) Serial.println(x, HEX);
#else
	#define _SERIAL_BEGIN(x)
	#define _SERIAL_PRINT(x)
	#define _SERIAL_PRINT_HEX(x)
	#define _SERIAL_PRINTLN(x)
	#define _SERIAL_PRINTLN_HEX(x)
#endif

#ifndef ESP8266
	#include <TimerOne.h>
#endif

#define drivingForwards 1
#define drivingBackwards 2

const byte maxSerialBufferLength = 180; 
const byte messageHeaderLength = 26;

volatile int timerMode;
volatile bool dataOnSPI = false;
volatile long lastInterruptAtMilliseconds = 0;
volatile long interruptCount = 0;

enum topicList
{
  spreaderLightsTimeOut,
  messageTimeOut,
  functionReplyTimeOut
};

enum messageDirection
{
  received,
  sent
};

enum SPIApplicationType
{
	IamSPIMaster,
	IamSPISlave
};

typedef struct  positionTemplate gridPositionType;
struct positionTemplate {
  int x;
  int y;
  int orientationRelativeToGrid;
};

char equipmentNameList[13][5] = {"GS  ", "S011","S012","A011","A012","T011","T012","L011","L012","C011","C012", "Z001", "Z002"};
// GS   = Groundstation
// Sxxx = AutoStraddle Carrier
// Axxx = Automated Guided Vehicle
// Txxx = Turtle
// Lxxx = Terminal Truck (or Turtle)
// Cxxx = ARTG (or Crane)
// Zxxx = Origin point (Zero)



const byte maxMessageParameters  = maxSerialBufferLength - messageHeaderLength;

enum operationPossibilities {
	standByMode=0,
	startDrivingMode=1,
	turnLeftMode=2,
	turnRightMode=3,
	continueDrivingMode=4,
	manualMode=5,
	mecanumDriveSetupMode=6,
	mecanumDriveExecuteMode=7,
	testMode=10,
	forwardDrivingMode=11,
	backwardDrivingMode=12,
	obstacleAvoidanceMode=13,
	checkPositionMode=14,
	rotatePoleMode=15,
	lockSpreaderMode=18,
	unlockSpreaderMode=19,
	liftSpreaderToTopMode=20,
	dropSpreaderToBottomMode=21,
	straightDrivingMode=22,
	abortedRunMode=23,
	stopDrivingMode=24,
	guidedDriveMode=25,
	switchedOffMode=26,
	checkRadioStatusMode=27,
	noChangeInStatus=28,
	originFoundMode=29,
	executeCommandsMode=30
};

// Define messages from between ground station and vehicle or vehicle and radar at origin
enum messageTypeIds {
	noMessageActive=0,
	radioStatusRequest=1,
	radioStatusReply=2,
	locationRequest=3,
	locationReply=4,
	radioACK=5,
	radioNACK=6,
	vehicleCommandList=7,
	errorSerialCommunication=8,
	messageTimeOutEncountered=9,
	vehicleStateList=10,
	searchOriginRequest=11,
	searchOriginReply=12,
	messageStatusReply=13,
	slaveDataRequest=14
};


enum messageCommands {
	noContent=0,
	switchToStandByMode=1,
	goStraight=2,
	goLeft=3,
	goRight=4,
	moveSpreaderToTop=5,                      // stop hoisting at highest position
	moveSpreaderToBottom=6,                   // stop lowering at lowest position
	sendACK=7,
	sendNACK=8,
	requestPosition=9,
	stopImmediately=10,  
	executeCommands=11,
	lockSpreader=17,
	unlockSpreader=18,
	switchToForwardDriving=19,
	switchToBackwardDriving=20,
	switchToTestMode=21,
	switchToManualMode=22,
	rotateRadarPole=23,
	switchToLocalGuiding=24,
	switchOff=25,
	checkRadioStatus=26,
	radioOn=27,
	radioOff=28,
	messageUnderstood=29,
	messageUnknown=30,	
	originFound=31,
	originNotFound=32,
	searchForOrigin=33,
	radarBusy=34,
	mecanumDrive=35,
	manualRouteList=97,
	simpleAutoRouteList=98,
	complexAutoRouteList=99
};

int mEngineCodes[11] = {0b10101010, 0b01010101, 0b10010110, 0b01101001, 0b10000010, 0b00101000, 0b00010100, 0b01000001, 0b01100110, 0b10011001, 0b00000000};

// motors order LF RF LR RR; 00 = stopped, 01 = forward, 10 = backward

typedef struct messageExchangeType aMessage;
struct messageExchangeType {
	byte totalMessageLength;      //  Length of the message
	byte totalContentItems;		  //  Number of items to be added in the content of the message (3 bytes per item)
	byte senderCode;			  //  One byte sender code to identify the messageNumber
	int  messageNumber;			  //  two byte integer to identify the message, in combination with the senderCode
	byte CRCByte;				  //  Control byte. Least significant byte of sum of all bytes
	byte medium[4];               //  WIFI or TXRX
	byte sender[5];               //  Main or radio moduleof center or of vehicle (4 characters + R for radiomodule or M for main module)
	byte addressee[5];            //  Main or radio moduleof center or of vehicle
	byte finalAddress[5];         //  Endpoint of message 
	byte messageTypeId;           //  Type of message = of type "messageTypeIds" 
	byte content[maxMessageParameters];            //  1 message command of type "messageCommands" plus parameter (3bytes, chars), free text or (location x, y, orientation) depending on message Type
};

volatile int byteToSend = 0;
byte byteMessage[maxSerialBufferLength];
volatile aMessage foo;
volatile bool hasData = false;
volatile bool pushToMaster = false;
volatile int bytesReceived;
volatile int slaveMessageLength;
volatile bool messageTimedOut = false;
volatile int timeOutCount = 0;
/*
void handleTimer1Trigger()
{
  noInterrupts();
  if (timerMode == messageTimeOut)
  {
    timeOutCount = timeOutCount + 1;
    if (timeOutCount > 10)
    {
      messageTimedOut = true;
      timeOutCount = 0;
    }
  }
  else if (timerMode == functionReplyTimeOut)
  {
    timeOutCount = timeOutCount + 1;
    if (timeOutCount > 90)
    {
      messageTimedOut = true;
      timeOutCount = 0;
    }

  }
  interrupts();
}
*/
#ifdef ESP8266
ICACHE_RAM_ATTR void setDataOnSPIFlag()
#else
void setDataOnSPIFlag()
#endif	
{
    dataOnSPI = true;
}

aMessage createEmptyMessage ()
{
	aMessage aux;
	memset (aux.content, 0, sizeof(aux.content));
	
	aux.totalMessageLength = 29;
	aux.totalContentItems = 1;
	aux.senderCode = 0;
	aux.messageNumber = 0;
	aux.CRCByte = 1;
	memset (aux.medium, 0, sizeof(aux.medium));
	memset (aux.sender, 0, sizeof(aux.sender));
	memset (aux.addressee, 0, sizeof(aux.addressee));
	memset (aux.finalAddress, 0, sizeof(aux.finalAddress));
	aux.messageTypeId = noMessageActive;
	memset (aux.content, 0, sizeof(aux.content));
	aux.content[0] = noContent;

	return aux;
}

template <typename T> unsigned int SPI_writeAnything (const T& value)
  {
	  // module called by master when it wants to initiate datatransfer to slave
    const byte * p = (const byte*) &value;
    unsigned int i;
	unsigned int lengthMessage;
	lengthMessage = *p;
	byte startMessage[3] = {0xEF, 0xF0, 0xF1};
	byte endMessage[3] = {0xF1, 0xF0, 0xEF};
	for (i = 0;i<3;i++)
	{
		SPI.transfer(startMessage[i]);
	}
    for (i = 0; i < lengthMessage; i++)
	{
        SPI.transfer(*p++);
	}
	for (i = 0;i<3;i++)
	{
		SPI.transfer(endMessage[i]);
	}
    return lengthMessage+6;
	
  }  // end of SPI_writeAnything

#ifndef ESP8266

template <typename T> int SPI_readAnything(bool IsISR, T& value)
  {
	  // module called by slave after master pushed a message
	  
    byte * p = (byte*) &value;
    unsigned int i;
	unsigned int lengthValue;
	int returnValue;
	
	byte one;
	byte two;
	byte three;

//
// search for start of message EF-F0-F1
//
    two = 0xFF;
	(IsISR)?three = SPDR:three = 0xFF;
	do
	{
		one = two;
		two = three;
		three =SPI.transfer(0);
	}
	while (!((one == 0xEF)&&(two == 0xF0)&&(three == 0xF1)));
 
	if (three != 0x00)
	{
		{
			lengthValue = 3;
			byte three = 0xFF;
			byte two = 0xFF;
			do
			{
				one = two;
				two = three;
				three =SPI.transfer(0);
				*p++= three;
				lengthValue++;
			}
			while (!((one == 0xF1)&&(two == 0xF0)&&(three == 0xEF)) || ((one == 0x00)&&(two == 0x00)&&(three == 0x00))) ;
			if (three == 0x00)
			{
				returnValue = -1; // unexpected end in error
			}
			else
			{
				returnValue = lengthValue;
			}
		}
	}	
	else
	{
		returnValue = -2;				// returnValue = -2 = no message found
	}
    return returnValue;
  }  // end of SPI_readAnything
#endif  
#ifndef ESP8266
ISR (SPI_STC_vect)
{
	// module on slave (radio module of all equipmentNameList
	

  interruptCount++;
  if (pushToMaster)
  {
	if (byteToSend == 0)
	{
		slaveMessageLength = byteMessage[byteToSend];
	}
    SPDR = byteMessage[byteToSend];
    byteToSend++;
	if (byteToSend > slaveMessageLength)
	{
		pushToMaster = false;
		byteToSend = 0;
	}
  }
  else
  {
	//Serial.println("Before SPI_readAnything");
    bytesReceived = SPI_readAnything (true, foo);
    hasData = true;
	//Serial.println("after SPI_readAnything, hasData = true");
  }
  
}

#endif

void triggerInterrupt (int intPin)
{
  digitalWrite (intPin, HIGH); //
  delay (2); // delay 2ms
  digitalWrite (intPin, LOW); //
}

void signalMaster(int triggerMasterPin)
{
  triggerInterrupt(triggerMasterPin);
  _SERIAL_PRINTLN("Master signalled");
}

byte transferAndWait (const byte what)
{
	// module called by SPI_Get_SlaveMessage when master initiates data download after interrupt
  byte a = SPI.transfer (what);
  delayMicroseconds (20);
  return a;
} // end of transferAndWait

aMessage SPI_Get_SlaveMessage (int csPin)
{
	// module called by master initiates data download after interrupt

	aMessage aux;
	byte lengthMessage;
	Serial.println("in SPI_GET_SLAVEMessage");
	digitalWrite(csPin, LOW);
	transferAndWait(0);
	lengthMessage = transferAndWait(0);
	byteMessage[0] = lengthMessage;
	for (int i = 1; i < lengthMessage; i++) //get all subsequent bytes
	  {
		byteMessage[i] = transferAndWait (0);
		Serial.print(" 0x");
		Serial.print(byteMessage[i],HEX);
	  }
	Serial.println();
	memcpy(&aux, &byteMessage, sizeof(aux));
	digitalWrite(csPin, HIGH);

	return aux;
}

bool sendSPIMessage (SPIApplicationType SPIType, int CSorWakeUp, aMessage SPIMessage)
{
	int bytesPassed;
	bool aux;

	if (SPIType == IamSPIMaster)
	{
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		digitalWrite(CSorWakeUp, LOW);    // SS is pin 10
		bytesPassed = SPI_writeAnything (SPIMessage);
		digitalWrite(CSorWakeUp, HIGH);
		SPI.endTransaction();
		aux = (bytesPassed > 0);
	}
	else  // now SPI Type = IamSPISlave
	{
		pushToMaster = true;
		memcpy(byteMessage, &SPIMessage, sizeof(byteMessage));
		signalMaster(CSorWakeUp);
		while (pushToMaster) 
		{ 
			delay(5);
		};
		aux = !messageTimedOut;
	}
	
	return aux;
	
}

aMessage sendAndReceiveSPIMessage (SPIApplicationType SPIType, int CSorWakeUp, aMessage SPIMessage, messageTypeIds requestedAnswer)
{
	aMessage aux;
	int bytesPassed;
	messageTypeIds receivedAnswer = noMessageActive;

	if (SPIType == IamSPIMaster)
	{
		while(receivedAnswer != requestedAnswer) // loop until answer matches question
		{	
			// send message
			
			SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
			digitalWrite(CSorWakeUp, LOW);    // SS is pin 10
			bytesPassed = SPI_writeAnything (SPIMessage);
			digitalWrite(CSorWakeUp, HIGH);
			SPI.endTransaction();
			
			// wait for any answer
			
			while (!dataOnSPI) {};
			
			// read the answer
			
			aux = SPI_Get_SlaveMessage(CSorWakeUp);
			receivedAnswer = (messageTypeIds)aux.messageTypeId;
			dataOnSPI = false;
		}
	}
	else  // now SPI Type = IamSPISlave
	{
		while(receivedAnswer != requestedAnswer)
		{
			pushToMaster = true;
			memcpy(byteMessage, &SPIMessage, sizeof(byteMessage));
			signalMaster(CSorWakeUp);
			while (pushToMaster) 
			{
				delay(5);
			};

			while (!hasData) 
			{
				delay(5);
			};
			memcpy(byteMessage, &SPIMessage, sizeof(byteMessage));
			//memcpy(&aux, &foo, sizeof(aux));
			receivedAnswer = (messageTypeIds)foo.messageTypeId;
		}		
	}	
	
	return aux;
}

volatile bool serialMessageTimedOut = false;
volatile byte TimeOutCount = 0;

void copyBytes (byte *_toBuffer, byte _startPositionTo, byte *_fromBuffer, byte _startPositionFrom, byte _length)
{
	for (int i=0;i<_length;i++)
	{
		_toBuffer[_startPositionTo+i] = _fromBuffer[_startPositionFrom+i];
	}
}

void moveStringToBytes (byte *_toBuffer, byte _startPositionTo, String _fromString, byte _StartPosition, byte _length)
{
	for (int i=0;i<_length;i++)
	{
		_toBuffer[_startPositionTo+i] = (byte)_fromString.substring(0, _length).charAt(i);
	}
}

operationPossibilities messageToState (messageCommands message)
{
	switch (message)
	{
		case noContent: return noChangeInStatus;
		case switchToStandByMode: return standByMode;
		case goStraight: return straightDrivingMode;
		case goLeft: return turnLeftMode;
		case goRight: return turnRightMode;
		case moveSpreaderToTop: return liftSpreaderToTopMode;
		case moveSpreaderToBottom: return dropSpreaderToBottomMode;                  // stop lowering at lowest position
		case sendACK: return noChangeInStatus;
		case sendNACK: return noChangeInStatus;
		case requestPosition: return checkPositionMode;
		case stopImmediately: return abortedRunMode;
		//case hoistSpreader: return  hoistSpreaderMode;                       // hoist spreader until "parameter hight" was reached
		//case lowerSpreader: return lowerSpreaderMode;                      // lower spreader until "parameter height" was reached
		//case speedupLeft: return speedupLeftMode;
		//case speedupRight: return speedupRightMode;
		//case slowdownLeft: return slowdownLeftMode;
		//case slowdownRight: return slowdownRightMode;
		case lockSpreader: return lockSpreaderMode;
		case unlockSpreader: return unlockSpreaderMode;
		case switchToForwardDriving: return forwardDrivingMode;
		case switchToBackwardDriving: return backwardDrivingMode;
		case switchToTestMode: return testMode;
		case switchToManualMode: return manualMode;
		case rotateRadarPole: return rotatePoleMode;
		case switchToLocalGuiding: return guidedDriveMode;
		case switchOff: return standByMode;
		case checkRadioStatus: return checkRadioStatusMode;
		case radioOn: return noChangeInStatus;
		case radioOff: return noChangeInStatus;
		case manualRouteList: return noChangeInStatus;
		case simpleAutoRouteList: return noChangeInStatus;
		case complexAutoRouteList: return noChangeInStatus;
	}
	return abortedRunMode;
}
	
void flushSerialBuffer (Stream &__port)
{
	byte garbageByte;
	int byteCount = 0;
	while(__port.available() > 0)
	{
		garbageByte = __port.read();
		byteCount++;
	}	
	_SERIAL_PRINTLN(byteCount);	; // flush the receive buffer
}

aMessage prepareSerialMessage (String __sender, String __addressee, String __finalAddress, messageTypeIds __aMessageType, byte __contentItems, byte __senderCode, int __messageNumber, byte *__content)
{
	aMessage __SerialMessage;
    long aux = 0;
	byte auxByte;
	
	moveStringToBytes(__SerialMessage.medium, 0, String("TXRX"), 0, 4) ;
	moveStringToBytes(__SerialMessage.sender, 0, __sender, 0, 5);    
	moveStringToBytes(__SerialMessage.addressee, 0, __addressee, 0, 5);  
	moveStringToBytes(__SerialMessage.finalAddress, 0, __finalAddress, 0, 5);   

	__SerialMessage.messageTypeId      = (byte)__aMessageType;

	__SerialMessage.totalMessageLength = __contentItems * 3 + messageHeaderLength;
	__SerialMessage.totalContentItems = __contentItems;
	__SerialMessage.senderCode = __senderCode;
	__SerialMessage.messageNumber = __messageNumber;
	copyBytes(__SerialMessage.content, 0, __content, 0, __contentItems * 3);
	aux+=__SerialMessage.totalMessageLength;
	aux+=__SerialMessage.totalContentItems;
	aux+=__SerialMessage.senderCode;
	aux+=__SerialMessage.messageNumber;
	for (int i=0;i<4;i++) aux+=__SerialMessage.medium[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.sender[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.addressee[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.finalAddress[i];
	aux+=__SerialMessage.messageTypeId;
	for (int i=0;i<__contentItems * 3;i++) aux+=__SerialMessage.content[i];

	__SerialMessage.CRCByte = aux & 0x000000FF;
	
/*
			_SERIAL_PRINT("lib: CRC calc, sum = ");
			_SERIAL_PRINT(aux);
			_SERIAL_PRINT("(");
			_SERIAL_PRINT_HEX(aux);
			_SERIAL_PRINT(")");
			_SERIAL_PRINT("\t LSB: ");
			_SERIAL_PRINT(aux & 0x000000FF);
			_SERIAL_PRINT("\t CRC Byte: ");
*/
			auxByte = __SerialMessage.CRCByte;
/*
			_SERIAL_PRINT(auxByte);
			_SERIAL_PRINT("(");
			_SERIAL_PRINT_HEX(auxByte);
			_SERIAL_PRINTLN(")");		
*/
	return __SerialMessage;
}

aMessage prepareWIFIMessage (String __sender, String __addressee, String __finalAddress, messageTypeIds __aMessageType , byte __contentItems, byte __senderCode, int __messageNumber, byte *__content)
{
	aMessage __WIFIMessage;
    long aux = 0;
	
	moveStringToBytes(__WIFIMessage.medium, 0, String("WIFI"), 0, 4) ;
	moveStringToBytes(__WIFIMessage.sender, 0, __sender, 0, 5);    
	moveStringToBytes(__WIFIMessage.addressee, 0, __addressee, 0, 5);  
	moveStringToBytes(__WIFIMessage.finalAddress, 0, __finalAddress, 0, 5);  

	__WIFIMessage.messageTypeId      = (byte)__aMessageType;
	__WIFIMessage.totalMessageLength = messageHeaderLength + __contentItems * 3;
	__WIFIMessage.totalContentItems = __contentItems;
	__WIFIMessage.senderCode = __senderCode;
	__WIFIMessage.messageNumber = __messageNumber;

	copyBytes(__WIFIMessage.content, 0, __content, 0, __contentItems * 3);
	
	aux+=__WIFIMessage.totalMessageLength;
	aux+=__WIFIMessage.totalContentItems;
	aux+=__WIFIMessage.senderCode;
	aux+=__WIFIMessage.messageNumber;
	for (int i=0;i<4;i++) aux+=__WIFIMessage.medium[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.sender[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.addressee[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.finalAddress[i];
	aux+=__WIFIMessage.messageTypeId;
	for (int i=0;i<__contentItems;i++) 
	{
		aux+=__WIFIMessage.content[i*3];
		aux+=__WIFIMessage.content[i*3+1];
		aux+=__WIFIMessage.content[i*3+2];
	}
	__WIFIMessage.CRCByte = aux & 0x000000FF;
	
	return __WIFIMessage;
}

String createString (byte *__byteBuffer, int __length)
{
	String auxString;

	for (int i=0;i<__length;i++)
	{
		auxString += char(__byteBuffer[i]);
	}	
	
	return auxString;
}

aMessage receiveWIFIUnsolicitedData(String *__msgP, String __Iam, String __acceptFrom)
{
	aMessage _WIFIMessage;
	int contentValue, contentPositionInString, contentIndex;
	String __msg = *__msgP;
	int WIFIMessageLength = __msg.length();
/*	_SERIAL_PRINT("lib:");
	_SERIAL_PRINTLN(__msg);
*/	 
	_WIFIMessage.totalMessageLength = __msg.substring(0, 3).toInt();
	_WIFIMessage.totalContentItems = __msg.substring(3, 6).toInt();  
	_WIFIMessage.senderCode = __msg.substring(6, 9).toInt();  
	_WIFIMessage.messageNumber = __msg.substring(9, 17).toInt();  
	_WIFIMessage.CRCByte = __msg.substring(17, 20).toInt();  
	
	moveStringToBytes(_WIFIMessage.medium, 0, __msg.substring(20, 24), 0, 4);
	moveStringToBytes(_WIFIMessage.sender, 0, __msg.substring(24, 29), 0, 5);
	moveStringToBytes(_WIFIMessage.addressee, 0, __msg.substring(29, 34), 0, 5);
	moveStringToBytes(_WIFIMessage.finalAddress, 0, __msg.substring(34, 39), 0, 5);
	_WIFIMessage.messageTypeId  = __msg.substring(39, 42).toInt();
//	_SERIAL_PRINT("lib:addressee =>");
//	_SERIAL_PRINT(__msg.substring(29, 33));
//	_SERIAL_PRINT("< en Iam = >");
//	_SERIAL_PRINTLN(__Iam);
	if ((__msg.substring(29, 33) == __Iam))
	{
//		_SERIAL_PRINTLN("Voldoet aan Iam");
//		_SERIAL_PRINT("lib:sender =>");
//		_SERIAL_PRINT(__msg.substring(24, 28));
//		_SERIAL_PRINT("< en acceptFrom = >");
//		_SERIAL_PRINTLN(__acceptFrom);
		if (__acceptFrom == "*" || (__msg.substring(24, 28) == __acceptFrom))
		{
//			_SERIAL_PRINTLN("Voldoet aan acceptfrom");		  
			if (_WIFIMessage.messageTypeId != radioACK)
			{
//				_SERIAL_PRINT("lib: messageTypeId is not RadioAck (=5); it is: >");
//				_SERIAL_PRINTLN(_WIFIMessage.messageTypeId);
				contentPositionInString = 42;
				contentIndex = 0;
//				_SERIAL_PRINT("lib: bericht lengte:");
//				_SERIAL_PRINT(WIFIMessageLength);
				for (int i=0;i<_WIFIMessage.totalContentItems;i++)
				{   
					_WIFIMessage.content[i*3]   = __msg.substring(contentPositionInString,   contentPositionInString+3).toInt();					
					_WIFIMessage.content[i*3+1] = __msg.substring(contentPositionInString+3, contentPositionInString+6).toInt();					
					_WIFIMessage.content[i*3+2] = __msg.substring(contentPositionInString+6, contentPositionInString+9).toInt();					
					contentPositionInString = contentPositionInString + 9;
//					_SERIAL_PRINT("\t status:");
//					_SERIAL_PRINT(_WIFIMessage.content[i*3]);
//					_SERIAL_PRINT("\t param1:>");
//					_SERIAL_PRINTLN(_WIFIMessage.content[i*3+1]);
//					_SERIAL_PRINT("\t param2:>");
//					_SERIAL_PRINTLN(_WIFIMessage.content[i*3+2]);
				}
			}
		}
		else
		{
//			_SERIAL_PRINTLN("s");
			_WIFIMessage.messageTypeId  = noMessageActive;
		}

	}
	else
	{
//		_SERIAL_PRINTLN("Message not for me");
		_WIFIMessage.messageTypeId  = noMessageActive;			
	}

	return _WIFIMessage;  
}
 
/*
mecanum wheel drive

1. forward: 01 01 01 01 
	LF: forward 
	RF: forward
	LR: forward
	RR: forward
	
2. backward: 10 10 10 10 
	LF: backward
	RF: backward
	LR: backward
	RR: backward

3. left:  10 01 01 10 
	LF: backward
	RF: forward
	LR: forward
	RR: backward

4. right: 01 10 10 01 
	LF: forward
	RF: backward
	LR: backward
	RR: forward

5. left-forward: 00 01 01 00
	LF: neutral
	RF: forward
	LR: forward
	RR: neutral

6. right-forward: 01 00 00 01
	LF: forward
	RF: neutral
	LR: neutral
	RR: forward

7. left-backward : 10 00 00 10
	LF: backward
	RF: neutral
	LR: neutral
	RR: backward

8. right-backward: 00 10 10 00
	LF: neutral
	RF: backward
	LR: backward
	RR: neutral

9. in-place-left-turn: 10 01 10 01
	LF: backward
	RF: forward
	LR: backward
	RR: forward

10. in-place-right-turn: 01 10 01 10
	LF: forward
	RF: backward
	LR: forward
	RR: backward

11. turn-of-rear-axis-left: 01 10 00 00
	LF: forward
	RF: backward
	LR: neutral
	RR: neutral

12. turn-of-rear-axis-right: 10 01 00 00
	LF: backward
	RF: forward
	LR: neutral
	RR: neutral

13. turn-of-front-axis-left: 00 00 01 10
	LF: neutral
	RF: neutral
	LR: forward
	RR: backward

14. turn-of-front-axis-right: 00 00 10 01
	LF: neutral
	RF: neutral
	LR: backward
	RR: forward

15. concerning-left-leftturn: 01 00 01 00
	LF: forward
	RF: neutral
	LR: forward
	RR: neutral

16. concerning-left-rightturn: 10 00 10 00
	LF: backward
	RF: neutral
	LR: backward
	RR: neutral

17. concerning-right-leftturn: 00 01 00 01
	LF: neutral
	RF: forward
	LR: neutral
	RR: forward

18. concerning-right-rightturn: 00 10 00 10 
	LF: neutral
	RF: backward
	LR: neutral
	RR: backward


*/
