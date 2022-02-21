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

const byte maxSerialBufferLength = 125; 
const byte messageHeaderLength = 26;
bool communicationErrorOccurred = false;
volatile int  timerMode;
volatile bool dataOnSPI = false;
volatile long lastInterruptAtMilliseconds = 0;
volatile long interruptCount = 0;
volatile int  SPIBufferPosition = 0;
volatile long triggerCount = 0;
volatile long dataOnSPICount = 0;
volatile byte TimeOutCount = 0;
volatile bool CRCStatus;

long messageNumber = 0;
long keepAliveTime;

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

enum ASCSlaveCommands {
	requestToLiftSpreader         =30,  // command sent to slave to trigger lifting the spreader
	requestToDropSpreader         =31,  // command sent to slave to trigger dropping the spreader
	requestDisplayLines           =32,  // command sent to slave to get display lines
	requestInterruptType          =33,  // command sent to slave to inform after type of interrupt
	requestSpreaderHeight         =34,  // command sent to slave to inform after the spreader height;
	requestSlaveReady             =35,  // command sent to slave to inform after the readyness of the slave (is it out of setup?);
	informMatrixBoardData         =37,  // command sent to nano to inform that a matrixboard message is coming
	requestResetSlave             =38,  // command sent to slave to execute a reset after the start
	informSlaveToStart            =39,  // command sent to slave to inform slave of master being ready for slave to start
	heartBeat                     =40,  // master still active
	informToReceiveDashboardInfo  =41,  // command to inform slave that dashboard info is coming
	requestWheelPulsesCount       =42,  // request to motor mcu to update number of wheel pulses
	informMotorOrder              =43,  // command with motor order info
	requestDisplayMotorData       =44,  // request for motor data to display
	informExecuteMotorOrder       =45,  // command to execute the motor order
 	informLoglinePart1            =46,  // command sent to nano to inform that a logline is coming (chars 0-29)
	informLoglinePart2            =47,  // command sent to nano to inform that a logline is coming (chars 30-59)
	informLoglinePart3            =48,  // command sent to nano to inform that a logline is coming (chars 60-80)
	logLineInterrupt              =94,
	noMessage 					  =95,	
 	displayLineInterrupt 		  =96,
	obstacleInterrupt 			  =97,	
	slaveBusy 					  =98,
	slaveReady 					  =99
};

enum operationPossibilities {
	standByMode					=0,
	startDrivingMode			=1,
	turnLeftMode				=2,
	turnRightMode				=3,
	continueDrivingMode			=4,
	manualMode					=5,
	mecanumDriveSetupMode		=6,
	mecanumDriveExecuteMode		=7,
	testMode					=10,
	forwardDrivingMode			=11,
	backwardDrivingMode			=12,
	obstacleAvoidanceMode		=13,
	checkPositionMode			=14,
	rotatePoleMode				=15,
	lockSpreaderMode			=18,
	unlockSpreaderMode			=19,
	liftSpreaderToTopMode		=20,
	dropSpreaderToBottomMode	=21,
	straightDrivingMode			=22,
	abortedRunMode				=23,
	stopDrivingMode				=24,
	guidedDriveMode				=25,
	switchedOffMode				=26,
	checkRadioStatusMode		=27,
	noChangeInStatus			=28,
	originFoundMode				=29,
	executeCommandsMode			=30
};

// Define messages from between ground station and vehicle or vehicle and radar at origin
enum messageTypeIds {
	noMessageActive			=0,
	radioStatusRequest		=1,
	radioStatusReply		=2,
	locationRequest			=3,
	locationReply			=4,
	radioACK				=5,
	radioNACK				=6,
	vehicleCommandList		=7,
	vehicleStateList		=8,
	searchOriginRequest		=9,
	searchOriginReply		=10,
	messageStatusReply		=11,
	slaveDataRequest		=12,
	matrixBoardAction		=13,
	loggerAction			=14,
	testMessageSingle		=15,
	testMessageRequest		=16,
	testMessageReply		=17,
	restartCommunication	=18
};


enum messageCommands {
	noContent					=0,
	switchToStandByMode			=1,
	goStraight					=2,
	goLeft						=3,
	goRight						=4,
	moveSpreaderToTop			=5,                      // stop hoisting at highest position
	moveSpreaderToBottom		=6,                   // stop lowering at lowest position
	requestPosition				=9,
	stopImmediately				=10,  
	executeCommands				=11,
	lockSpreader				=17,
	unlockSpreader				=18,
	switchToForwardDriving		=19,
	switchToBackwardDriving		=20,
	switchToTestMode			=21,
	switchToManualMode			=22,
	rotateRadarPole				=23,
	switchToLocalGuiding		=24,
	switchOff					=25,
	checkRadioStatus			=26,
	radioOn						=27,
	radioOff					=28,
	messageUnderstood			=29,
	messageUnknown				=30,	
	messageOutOfSequence		=31,
	errorSerialCommunication	=32,
	messageTimeOutEncountered	=33,
	originFound					=34,
	originNotFound				=35,
	searchForOrigin				=36,
	radarBusy					=37,
	mecanumDrive				=38,
	showMatrixBoardData			=39,
	dumpLoglineData				=40,
	testMessageData				=41,
	manualRouteList				=97,
	simpleAutoRouteList			=98,
	complexAutoRouteList		=99
};

int mEngineCodes[11] = {0b10101010, 0b01010101, 0b10010110, 0b01101001, 0b10000010, 0b00101000, 0b00010100, 0b01000001, 0b01100110, 0b10011001, 0b00000000};

// motors order LF RF LR RR; 00 = stopped, 01 = forward, 10 = backward

typedef struct messageExchangeType aMessage;
struct messageExchangeType {
	
	byte totalMessageLength;      		 //  Length of the message
	byte totalContentItems;		  		 //  Number of items to be added in the content of the message (3 bytes per item)
	byte senderCode;			  		 //  One byte sender code to identify the messageNumber
	byte messageNumberHighByte;	  		 //  two byte integer to identify the message, in combination with the senderCode
	byte messageNumberLowByte;	  		 //  two byte integer to identify the message, in combination with the senderCode
	byte CRCByte;				  		 //  Control byte. Least significant byte of sum of all bytes
	byte medium[4];               		 //  WIFI or TXRX
	byte sender[5];               		 //  Main or radio moduleof center or of vehicle (4 characters + R for radiomodule or M for main module)
	byte addressee[5];            		 //  Main or radio moduleof center or of vehicle
	byte finalAddress[5];         		 //  Endpoint of message 
	byte messageTypeId;           		 //  Type of message = of type "messageTypeIds" 
	byte content[maxMessageParameters];  //  1 message command of type "messageCommands" plus parameters (3bytes, chars), free text or (location x, y, orientation) depending on message Type
};

aMessage mS_messageStatusReply;

int freeRAM () {
	extern int __heap_start, *__brkval;
	int v;
	int fr = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

	if (fr < 1024)
	{
	  _SERIAL_PRINTLN ("ALARM -- MEMORY BELOW 1K BYTES -- ALARM");
	}

	return fr;
}

void triggerInterrupt (int intPin)
{
  digitalWrite (intPin, HIGH); //
  delay (2); // delay 2ms
  digitalWrite (intPin, LOW); //
  triggerCount++;
  Serial.println("Trigger interrupt");
}

void signalMaster(int triggerMasterPin)
{
  triggerInterrupt(triggerMasterPin);
}

void displayMessage(aMessage * msg, bool outgoing)
{

   if(outgoing)
   {
	   Serial.print("OUTGOING:\t ");
   }
   else
   {
	   Serial.print("INCOMING: \t ");
   }  

  Serial.print(msg->totalMessageLength);    //  Length of the message
  Serial.print("\t ");  
  Serial.print(msg->totalContentItems);     //  Number of items to be added in the content of the message (3 bytes per item)
  Serial.print("\t ");
  Serial.print(msg->senderCode);            //  One byte sender code to identify the messageNumber
  Serial.print("\t ");
  Serial.print(((msg->messageNumberHighByte << 8) | msg->messageNumberLowByte));         //  two byte integer to identify the message, in combination with the senderCode
  Serial.print("\t ");
  Serial.print(msg->CRCByte);               //  Control byte. Least significant byte of sum of all bytes
  Serial.print("\t ");
  for (int i = 0; i < 4; i++) Serial.print(char(msg->medium[i]));
 Serial.print("\t ");
  for (int i = 0; i < 5; i++) Serial.print(char(msg->sender[i]));
  Serial.print("\t ");  
  for (int i = 0; i < 5; i++) Serial.print(char(msg->addressee[i]));
  Serial.print("\t ");  
  for (int i = 0; i < 5; i++) Serial.print(char(msg->finalAddress[i]));
  Serial.print("\t ");
  Serial.print(msg->messageTypeId);
  for (int i = 0; i < ((msg->totalContentItems + 1) * 3); i++)
  {
    Serial.print("\t ");
    Serial.print(msg->content[i]);
  }
  Serial.println("");

}

volatile int byteToSend = 0;
volatile byte byteMessage[maxSerialBufferLength];
volatile bool hasData = false;
volatile int bytesReceived;
volatile int slaveMessageLength;
volatile bool messageTimedOut = false;
volatile int timeOutCount = 0;
volatile bool processingMessage = false;

volatile int pos;
volatile bool pushToMaster = false;
volatile bool active = false;
volatile byte endByteOne = 0;
volatile byte endByteTwo = 0;
volatile byte endByteThree = 0;


void dumpByteMessage(int bufferLength)
{
	int dataLength;
	
	dataLength = (int)byteMessage[0];
	dataLength = min(dataLength, bufferLength);
	_SERIAL_PRINT("Message has ");
	_SERIAL_PRINT(byteMessage [0]);
	_SERIAL_PRINTLN(" bytes");	
	if (dataLength > 0)
	{
		for (int i=0;i<bufferLength;i++)
		{
			if (byteMessage[i] < 16)
			{
				_SERIAL_PRINT(" 0x0");
				_SERIAL_PRINT_HEX(byteMessage[i]);
			}
			else
			{
				_SERIAL_PRINT(" 0x");
				_SERIAL_PRINT_HEX(byteMessage[i]);      
			}
			if (((i+1) % 30)==0)
			{
			_SERIAL_PRINTLN();
			}
		}
	}
	else
	{
	_SERIAL_PRINT("No message available");
	}
  _SERIAL_PRINTLN();
}

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

#ifdef ESP8266
ICACHE_RAM_ATTR void setDataOnSPIFlag()
#else
void setDataOnSPIFlag()
#endif	
{
	if (interruptCount == 0)
	{
		lastInterruptAtMilliseconds = millis();
		interruptCount++;
	}
	dataOnSPICount++;
	dataOnSPI = true;//((millis() - lastInterruptAtMilliseconds) > 500);
}


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

	__SerialMessage.totalMessageLength = __contentItems * 3 + messageHeaderLength + 3;
	__SerialMessage.totalContentItems = __contentItems;
	__SerialMessage.senderCode = __senderCode;
	__SerialMessage.messageNumberLowByte = (__messageNumber & 0x00FF) ;
	__SerialMessage.messageNumberHighByte = (__messageNumber & 0xFF00) >> 8;
	memset(__SerialMessage.content, 0, sizeof(__SerialMessage.content));
	copyBytes(__SerialMessage.content, 0, __content, 0, __contentItems * 3);
	__SerialMessage.content[__contentItems*3]   = 0xEF;
	__SerialMessage.content[__contentItems*3+1] = 0xF0;
	__SerialMessage.content[__contentItems*3+2] = 0xF1;
	
	aux+=__SerialMessage.totalMessageLength;
	aux+=__SerialMessage.totalContentItems;
	aux+=__SerialMessage.senderCode;
	aux+=__SerialMessage.messageNumberLowByte;
	aux+=__SerialMessage.messageNumberHighByte;
	for (int i=0;i<4;i++) aux+=__SerialMessage.medium[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.sender[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.addressee[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage.finalAddress[i];
	aux+=__SerialMessage.messageTypeId;
	for (int i=0;i<(__contentItems + 1) * 3;i++) aux+=__SerialMessage.content[i];
	
	__SerialMessage.CRCByte = aux & 0x000000FF;
	
	auxByte = __SerialMessage.CRCByte;

	return __SerialMessage;
}

aMessage createGeneralMessage (messageTypeIds msgType, messageCommands msgCmd, byte contentByte)
{
	aMessage aux;
	int auxCRC = 0;
	memset (aux.content, 0, sizeof(aux.content));
	
	aux.totalMessageLength = messageHeaderLength + 6;
	aux.totalContentItems = 1;
	aux.senderCode = 0;
	aux.messageNumberHighByte = 0;
	aux.messageNumberLowByte = 0;
	memset (aux.medium, 88, sizeof(aux.medium));
	memset (aux.sender, 88, sizeof(aux.sender));
	memset (aux.addressee, 88, sizeof(aux.addressee));
	memset (aux.finalAddress, 88, sizeof(aux.finalAddress));
	aux.messageTypeId = msgType;
	memset (aux.content, 0, sizeof(aux.content));
	aux.content[0] = msgCmd;
	aux.content[1] = contentByte;	
	aux.content[3] = 0xEF;
	aux.content[4] = 0xF0;
	aux.content[5] = 0xF1;
	
	auxCRC +=aux.totalMessageLength;
	auxCRC +=aux.totalContentItems;
	auxCRC +=aux.senderCode;
	auxCRC +=aux.messageNumberHighByte;
	auxCRC +=aux.messageNumberLowByte;
	for (int i=0;i<4;i++) auxCRC +=aux.medium[i];
	for (int i=0;i<5;i++) auxCRC +=aux.sender[i];
	for (int i=0;i<5;i++) auxCRC +=aux.addressee[i];
	for (int i=0;i<5;i++) auxCRC +=aux.finalAddress[i];
	auxCRC +=aux.messageTypeId;
	for (int i=0;i<(aux.totalContentItems+1)*3;i++) auxCRC +=aux.content[i];
	
	aux.CRCByte = auxCRC & 0x000000FF;
	
	return aux;
}

template <typename T> unsigned int SPI_writeAnything (const T& value)
  {
	  // module called by master when it wants to initiate datatransfer to slave
    const byte * p = (const byte*) &value;
    unsigned int i;
	unsigned int lengthMessage;
	lengthMessage = *p;
    //for (i = 0; i < sizeof(value); i++)
    for (i = 0; i < lengthMessage; i++)		
	{
        SPI.transfer(*p++);
		delay(15);
	}
    return lengthMessage;
	
  }  // end of SPI_writeAnything

aMessage SPIReadMessageFromSlave(byte CSPin)
{
	aMessage aux;
	byte byteStopOne;
	byte byteStopTwo;
	byte byteStopThree;
	byte firstByte;

	// enable Slave Select
	digitalWrite (CSPin, LOW);
	delay(15);
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	SPI.transfer (1);   // initiate transmission
	delay(15);
	byteStopOne = 0;
	byteStopTwo = 0;
	byteStopThree = 0;
	byteStopOne = byteStopTwo;
	byteStopTwo = byteStopThree;
	byteMessage [0] = SPI.transfer (0);
	delay(15);
//
// first messagebyte is length, if first byte < 32, this is no message
//

	if (byteMessage[0] < (messageHeaderLength+3))
	{
		aux = createGeneralMessage (noMessageActive, errorSerialCommunication, 6);
		_SERIAL_PRINTLN(" - No message");
	}
	else 
	{
//
// read rest of message
//
//		for (int posLocal = 1; posLocal < sizeof (byteMessage) - 1; posLocal++)
		for (int posLocal = 1; posLocal < byteMessage[0]; posLocal++)
		{
			byteStopOne = byteStopTwo;
			byteStopTwo = byteStopThree;
			byteMessage [posLocal] = SPI.transfer (0);
			delay(15);
			byteStopThree = byteMessage [posLocal];
			if ((byteStopOne == 0xEF) && (byteStopTwo == 0xF0) && (byteStopThree == 0xF1))
			{
			break;
			}
		}
		memcpy (&aux, (byte *)byteMessage, sizeof(aux));
	} 
	SPI.endTransaction();

  // disable Slave Select
	digitalWrite (CSPin, HIGH);
	CRCStatus = true;
	delay(15);  
	
	return aux;
}

void claimSPIchannel(int semafoor)
{
	Serial.println("Claim Channel");
	while (digitalRead(semafoor) == LOW)
	{
		delay(10);
	}
	pinMode(semafoor, OUTPUT);
	digitalWrite(semafoor, LOW);
	delay(10);
}

void releaseSPIChannel(int semafoor)
{
	Serial.println("Release Channel");
	pinMode(semafoor, INPUT);
	
// if error occurred in the communication, just let all communication time-outs run out and then start again

	(communicationErrorOccurred)?delay(1000):delay(10);
}


bool sendSPIMessageOneWay (SPIApplicationType SPIType, int CSorWakeUp, aMessage SPIMessage)
{
	int 	bytesPassed;
	bool 	aux;
	long 	startTimeTransaction;

	keepAliveTime = millis();

	if (SPIType == IamSPIMaster)
	{   
		delay(15);
		digitalWrite(CSorWakeUp, LOW);
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		bytesPassed = SPI_writeAnything (SPIMessage);
		SPI.endTransaction();
		digitalWrite(CSorWakeUp, HIGH);
		delay(15);
		aux = (bytesPassed > 0);
		communicationErrorOccurred = !aux;
	}
	else  // now SPI Type = IamSPISlave
	{
		memcpy((byte *)byteMessage, &SPIMessage, sizeof(byteMessage));
		pushToMaster = true;
		startTimeTransaction = millis();
		signalMaster(CSorWakeUp);
		while (((millis()-startTimeTransaction)<3000) && (pushToMaster)) {}
		aux = !pushToMaster;
		communicationErrorOccurred = !aux;
		pushToMaster = false;
	}

	return aux;
}

bool waitForMessageStatusMaster(int CSorWakeUp, byte senderCode)
{
	bool 		aux;
	bool 		waiting = true;
	aMessage 	mR_ReturnMessage;
	byte		rawContent[6];
	byte		messageTypeId;
	bool		sentSuccess;
	long		startWaitTime;
	
	//Serial.println("Master: Waiting for messageStatusReply after sending message from Master");
	
	startWaitTime = millis();
	while (waiting)
	{
		if (dataOnSPI)
		{
			dataOnSPI = false;
			waiting = false;
			mR_ReturnMessage = SPIReadMessageFromSlave(CSorWakeUp);
			displayMessage(&mR_ReturnMessage, false);
			messageTypeId = mR_ReturnMessage.messageTypeId;
			
			switch (messageTypeId)
			{
				case restartCommunication:
					_SERIAL_PRINTLN("Master: Received Keep alive while waiting... *** ERROR ***");
					aux = false;
					break;
				case messageStatusReply:
					_SERIAL_PRINTLN("Master: Received message status reply ");
					aux = (mR_ReturnMessage.content[0] == messageUnderstood);
					break;
				default:
					_SERIAL_PRINTLN("Master:Received message out of sequence");
					rawContent[0] = messageOutOfSequence;
					rawContent[1] = mR_ReturnMessage.messageNumberHighByte;
					rawContent[2] = mR_ReturnMessage.messageNumberLowByte;
					messageNumber++;
					mS_messageStatusReply  = prepareSerialMessage ("RADIO", "MAINM", "MAINM", messageStatusReply,  1, senderCode, messageNumber, &rawContent[0]);
					sentSuccess = sendSPIMessageOneWay(IamSPIMaster, CSorWakeUp, mS_messageStatusReply);
					//displayMessage (&mS_messageStatusReply, true);
					aux = false;
					break;
			}
		}
		else
		{
			if ((millis() - startWaitTime) > 3000)
			{
				aux = false;
				waiting = false;
			}
		}
	}

	//Serial.println("Exiting waitForMessageStatusMaster");
	
	return aux;
}
bool waitForMessageStatusSlave(int CSorWakeUp, byte senderCode)
{
	bool aux;
	bool 		waiting = true;
	aMessage 	mR_ReturnMessage;
	byte		rawContent[6];
	byte		messageTypeId;
	bool		sentSuccess;
	long		startWaitTime;	
	
	//Serial.println("Slave:Waiting for messageStatusReply after sending message from Slave");
	startWaitTime = millis();

	while (waiting)
	{
		if (hasData)
		{
			hasData = false;
			waiting = false;
			memcpy(&mR_ReturnMessage, (byte *)byteMessage, sizeof(mR_ReturnMessage));
			SPIBufferPosition = 0;
			//displayMessage(&mR_ReturnMessage, false);
			messageTypeId = mR_ReturnMessage.messageTypeId;
			switch (messageTypeId)
			{
				case restartCommunication:
					_SERIAL_PRINTLN("Slave:Received Keep alive while waiting... *** ERROR ***");
					waiting = false;
					aux = false;
					break;
				case messageStatusReply:
					_SERIAL_PRINTLN("Slave:Received message status reply ");
					aux = (mR_ReturnMessage.content[0] == messageUnderstood);
					break;
				default:
					_SERIAL_PRINTLN("Slave:Received message out of sequence");
					rawContent[0] = messageOutOfSequence;
					rawContent[1] = mR_ReturnMessage.messageNumberHighByte;
					rawContent[2] = mR_ReturnMessage.messageNumberLowByte;
					messageNumber++;
					mS_messageStatusReply  = prepareSerialMessage ("RADIO", "MAINM", "MAINM", messageStatusReply,  1, senderCode, messageNumber, &rawContent[0]);
					sentSuccess = sendSPIMessageOneWay(IamSPISlave, CSorWakeUp, mS_messageStatusReply);
					displayMessage (&mS_messageStatusReply, true);
					SPIBufferPosition = 0;
					aux = false;
					break;
			}
		}
		else
		{
			if ((millis() - startWaitTime) > 3000)
			{
				aux = false;
				waiting = false;
			}	
		}
	}	
	//Serial.println("Exiting waitForMessageStatusSlave");
	return aux;
}

bool waitForMessageStatus(SPIApplicationType SPIType, int CSorWakeUp, byte senderCode)
{
	bool aux;

	
	if (SPIType == IamSPIMaster)
	{
		aux = waitForMessageStatusMaster(CSorWakeUp, senderCode);
	}
	else
	{
		aux = waitForMessageStatusSlave(CSorWakeUp, senderCode);
	}
	communicationErrorOccurred = !aux;
	
	return aux;

}

aMessage waitForRequestedAnswerMaster(int CSorWakeUp, byte senderCode, byte requestedAnswer)
{
	aMessage 	aux;
	bool 		waiting = true;
	aMessage 	mR_ReturnMessage;
	byte		rawContent[6];
	byte		messageTypeId;
	bool		sentSuccess;
	long		startWaitTime;
	
	startWaitTime = millis();
	
	while (waiting)
	{
		if (dataOnSPI)
		{
			dataOnSPI = false;
			waiting = false;
			mR_ReturnMessage = SPIReadMessageFromSlave(CSorWakeUp);
			//displayMessage(&mR_ReturnMessage, false);
			messageTypeId = mR_ReturnMessage.messageTypeId;
			
			switch (messageTypeId)
			{
				case restartCommunication:
					aux = createGeneralMessage(restartCommunication, noContent, 0);
					communicationErrorOccurred = true;	
					break;
				case messageStatusReply:
					aux = createGeneralMessage(messageStatusReply, messageUnknown, messageOutOfSequence);
					communicationErrorOccurred = true;	
					break;
				default:
					if (messageTypeId == requestedAnswer)
					{
						aux = mR_ReturnMessage;
						rawContent[0] = messageUnderstood;						
						communicationErrorOccurred = false;	
					}
					else
					{
						aux = createGeneralMessage(messageStatusReply, messageUnknown, messageOutOfSequence);
						rawContent[0] = messageOutOfSequence;						
						communicationErrorOccurred = true;	
					}
					rawContent[1] = mR_ReturnMessage.messageNumberHighByte;
					rawContent[2] = mR_ReturnMessage.messageNumberLowByte;
					messageNumber++;
					mS_messageStatusReply  = prepareSerialMessage ("----R", "----M", "----M", messageStatusReply,  1, senderCode, messageNumber, &rawContent[0]);
					sentSuccess = sendSPIMessageOneWay(IamSPIMaster, CSorWakeUp, mS_messageStatusReply);
					//displayMessage (&mS_messageStatusReply, true);
					break;
			}
		}
		else
		{
			if ((millis() - startWaitTime) > 3000)
			{
				aux = createGeneralMessage(messageStatusReply, messageTimeOutEncountered, 0);
				waiting = false;
				communicationErrorOccurred = true;	
			}		
		}
	}
	return aux;
}

aMessage waitForRequestedAnswerSlave(int CSorWakeUp, byte senderCode, byte requestedAnswer)
{
	aMessage	aux;
	bool 		waiting = true;
	aMessage 	mR_ReturnMessage;
	byte		rawContent[6];
	byte		messageTypeId;
	bool		sentSuccess;
	long		startWaitTime;
	
	startWaitTime = millis();
	
	while (waiting)
	{
		if (hasData)
		{
			hasData = false;
			waiting = false;
			memcpy(&mR_ReturnMessage, (byte *)byteMessage, sizeof(mR_ReturnMessage));
			SPIBufferPosition = 0;
			//displayMessage(&mR_ReturnMessage, false);
			messageTypeId = mR_ReturnMessage.messageTypeId;
			switch (messageTypeId)
			{
				case restartCommunication:
					aux = createGeneralMessage(restartCommunication, noContent, 0);
					communicationErrorOccurred = true;		
				break;
				case messageStatusReply:
					aux = createGeneralMessage(messageStatusReply, messageUnknown, messageOutOfSequence);
					communicationErrorOccurred = true;		
					break;
				default:
					if (messageTypeId == requestedAnswer)
					{
						aux = mR_ReturnMessage;
						rawContent[0] = messageUnderstood;		
						communicationErrorOccurred = false;						
					}
					else
					{
						aux = createGeneralMessage(messageStatusReply, messageUnknown, messageOutOfSequence);
						rawContent[0] = messageOutOfSequence;		
						communicationErrorOccurred = true;							
					}
					rawContent[1] = mR_ReturnMessage.messageNumberHighByte;
					rawContent[2] = mR_ReturnMessage.messageNumberLowByte;
					messageNumber++;
					mS_messageStatusReply  = prepareSerialMessage ("RADIO", "MAINM", "MAINM", messageStatusReply,  1, senderCode, messageNumber, &rawContent[0]);
					sentSuccess = sendSPIMessageOneWay(IamSPISlave, CSorWakeUp, mS_messageStatusReply);
					//displayMessage (&mS_messageStatusReply, true);
	
					break;
			}
		}
		else
		{
			if ((millis() - startWaitTime) > 3000)
			{
				aux = createGeneralMessage(messageStatusReply, messageTimeOutEncountered, 0);
				waiting = false;
				communicationErrorOccurred = true;							
			}		
		}
	}	

	return aux;
}

aMessage waitForRequestedAnswer(SPIApplicationType SPIType, int CSorWakeUp, byte senderCode, byte requestedAnswer)
{
	aMessage aux;


	if (SPIType == IamSPIMaster)
	{
		aux = waitForRequestedAnswerMaster(CSorWakeUp, senderCode, requestedAnswer);
	}
	else
	{
		aux = waitForRequestedAnswerSlave(CSorWakeUp, senderCode, requestedAnswer);
	}

	return aux;
}

bool sendSPIMessage (SPIApplicationType SPIType, int CSorWakeUp, aMessage SPIMessage)
{
	bool 		aux;
	long		intervalBegin;
	long 		intervalEnd;
	int			maxTries = 3;
	int			currentTry;

	currentTry = 1;
	while (currentTry < maxTries)
	{
		//Serial.print("--Sending up: ");
		intervalBegin = millis();
		aux = sendSPIMessageOneWay (SPIType, CSorWakeUp, SPIMessage);
		intervalEnd = millis();
		Serial.print("sendSPIMessageOneWay takes: ");
		//(aux)?Serial.println("OK"):Serial.println("Not OK");
		Serial.println (intervalEnd - intervalBegin);
		if (aux) 
		{
			intervalBegin = millis();
			aux = waitForMessageStatus (SPIType, CSorWakeUp, SPIMessage.senderCode);
			intervalEnd = millis();
			Serial.print("waitForMessageStatus takes: ");
			Serial.println (intervalEnd - intervalBegin);
		}
		(aux)?currentTry = maxTries:currentTry++;
	}
	return aux;
}

#ifndef ESP8266
ISR (SPI_STC_vect)
{
	// module on slave (main module of all equipmentNameList
  long CRCCheck;
  if (pushToMaster)
  {
    byte c = SPDR;
    if (c == 1)  // starting new sequence?
    {
      active = true;
      pos = 0;
	  endByteOne = 0;
      endByteTwo = 0;

	  endByteThree = byteMessage[pos];
      SPDR = byteMessage [pos];   // send first byte
	  pos++;
      return;
    }

    if (!active)
    {
      SPDR = 0;
      return;
    }

    SPDR = byteMessage [pos];
    endByteOne = endByteTwo;
    endByteTwo = endByteThree;
    endByteThree = byteMessage[pos];

    if (((endByteOne == 0xEF) && (endByteTwo == 0xF0)&&(endByteThree == 0xF1))|| ++pos >= sizeof (byteMessage))
    {
      active = false;
      pushToMaster = false;
    }
  }
  else
  {
	byte c = SPDR;
	
	if (SPIBufferPosition == 0)
	{
		endByteTwo = 0;
		endByteThree = 0;
	}
	
	endByteOne = endByteTwo;
	endByteTwo = endByteThree;
	endByteThree = c;

	if (!((SPIBufferPosition == 0) && (endByteThree == 0))) // check for rogue bye as the start of  a message
	{
		if (SPIBufferPosition <  sizeof(byteMessage)) // as long as ther is room in the message buffer accept message
		{	
			byteMessage[SPIBufferPosition] = c;
			SPIBufferPosition++;
			if ((endByteOne == 0xEF)&&(endByteTwo == 0xF0)&&(endByteThree == 0xF1))
//
// end of message reached because of end sequence encountered	
			{
				hasData = true;
				CRCCheck = 0;
				for (int i=0;i<SPIBufferPosition;i++)
				{
					CRCCheck += byteMessage[i];
				}
				CRCStatus = (byteMessage[5] == ((CRCCheck - byteMessage[5]) & 0x000000FF));

				endByteOne = 0;
				endByteTwo = 0;
				endByteThree = 0;
				SPIBufferPosition = 0;
			}
		}
		else
		{
			_SERIAL_PRINT("No end of line found ");
			hasData = true;
			CRCStatus = false;
			endByteOne = 0;
			endByteTwo = 0;
			endByteThree = 0;
			SPIBufferPosition = 0;
		}
	}
  }
} 
#endif

aMessage sendAndReceiveSPIMessage (SPIApplicationType SPIType, int CSorWakeUp, messageTypeIds requestedAnswer, aMessage SPIMessage)
{
	aMessage 			aux;
	int 				bytesPassed;
	bool 				sentSucces;
	messageTypeIds 		receivedAnswer = noMessageActive;
	long 				startTimeTransaction;
	
	sentSucces = sendSPIMessage (SPIType, CSorWakeUp, SPIMessage);
	
	if (sentSucces)
	{
		//Serial.println("request is successfull");
		aux = waitForRequestedAnswer (SPIType, CSorWakeUp, SPIMessage.senderCode, requestedAnswer);  
	}
	else
	{
		Serial.println("*** ERROR *** request failed");
		aux = createGeneralMessage (messageStatusReply, errorSerialCommunication, 2);
	}
	
	return aux;
}

operationPossibilities messageToState (messageCommands message)
{
	switch (message)
	{
		case noContent: 				return noChangeInStatus;
		case switchToStandByMode: 		return standByMode;
		case goStraight: 				return straightDrivingMode;
		case goLeft: 					return turnLeftMode;
		case goRight: 					return turnRightMode;
		case moveSpreaderToTop: 		return liftSpreaderToTopMode;
		case moveSpreaderToBottom: 		return dropSpreaderToBottomMode;                  // stop lowering at lowest position
		case requestPosition: 			return checkPositionMode;
		case stopImmediately: 			return abortedRunMode;
		case lockSpreader: 				return lockSpreaderMode;
		case unlockSpreader: 			return unlockSpreaderMode;
		case switchToForwardDriving: 	return forwardDrivingMode;
		case switchToBackwardDriving: 	return backwardDrivingMode;
		case switchToTestMode: 			return testMode;
		case switchToManualMode: 		return manualMode;
		case rotateRadarPole: 			return rotatePoleMode;
		case switchToLocalGuiding: 		return guidedDriveMode;
		case switchOff: 				return standByMode;
		case checkRadioStatus: 			return checkRadioStatusMode;
		case radioOn: 					return noChangeInStatus;
		case radioOff: 					return noChangeInStatus;
		case manualRouteList: 			return noChangeInStatus;
		case simpleAutoRouteList: 		return noChangeInStatus;
		case complexAutoRouteList: 		return noChangeInStatus;
	}
	return abortedRunMode;
}
	
aMessage prepareWIFIMessage (String __sender, String __addressee, String __finalAddress, messageTypeIds __aMessageType , byte __contentItems, byte __senderCode, int __messageNumber, byte *__content)
{
	aMessage __WIFIMessage;
    long aux = 0;
	
	moveStringToBytes(__WIFIMessage.medium, 0, String("WIFI"), 0, 4);
	moveStringToBytes(__WIFIMessage.sender, 0, __sender, 0, 5);    
	moveStringToBytes(__WIFIMessage.addressee, 0, __addressee, 0, 5);  
	moveStringToBytes(__WIFIMessage.finalAddress, 0, __finalAddress, 0, 5);  

	__WIFIMessage.messageTypeId      = (byte)__aMessageType;
	__WIFIMessage.totalMessageLength = messageHeaderLength + 3 + __contentItems * 3;
	__WIFIMessage.totalContentItems = __contentItems;
	__WIFIMessage.senderCode = __senderCode;
	__WIFIMessage.messageNumberLowByte = (__messageNumber & 0x00FF) ;
	__WIFIMessage.messageNumberHighByte = (__messageNumber & 0xFF00) >> 8;
	copyBytes(__WIFIMessage.content, 0, __content, 0, __contentItems * 3);
	
	__WIFIMessage.content[__contentItems*3]   = 0xEF;
	__WIFIMessage.content[__contentItems*3+1] = 0xF0;
	__WIFIMessage.content[__contentItems*3+2] = 0xF1;
	
	aux+=__WIFIMessage.totalMessageLength;
	aux+=__WIFIMessage.totalContentItems;
	aux+=__WIFIMessage.senderCode;
	aux+=__WIFIMessage.messageNumberLowByte;
	aux+=__WIFIMessage.messageNumberHighByte;
	for (int i=0;i<4;i++) aux+=__WIFIMessage.medium[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.sender[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.addressee[i];
	for (int i=0;i<5;i++) aux+=__WIFIMessage.finalAddress[i];
	aux+=__WIFIMessage.messageTypeId;
	for (int i=0;i<(__contentItems+1);i++) 
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
	int contentValue, contentPositionInString;
	String __msg = *__msgP;
	int WIFIMessageLength = __msg.length();
	 
	_WIFIMessage.totalMessageLength = __msg.substring(0, 3).toInt();
	_WIFIMessage.totalContentItems = __msg.substring(3, 6).toInt();  
	_WIFIMessage.senderCode = __msg.substring(6, 9).toInt();  
	_WIFIMessage.messageNumberHighByte = ((__msg.substring(9, 12).toInt()) & 0xFF00) >> 8;  
	_WIFIMessage.messageNumberLowByte = ((__msg.substring(12, 15).toInt()) & 0x00FF);  
	_WIFIMessage.CRCByte = __msg.substring(15, 18).toInt();  
	
	moveStringToBytes(_WIFIMessage.medium, 0, __msg.substring(18, 22), 0, 4);
	moveStringToBytes(_WIFIMessage.sender, 0, __msg.substring(22, 27), 0, 5);
	moveStringToBytes(_WIFIMessage.addressee, 0, __msg.substring(27, 32), 0, 5);
	moveStringToBytes(_WIFIMessage.finalAddress, 0, __msg.substring(32, 37), 0, 5);
	_WIFIMessage.messageTypeId  = __msg.substring(37, 40).toInt();
//	_SERIAL_PRINT("lib:addressee =>");
//	_SERIAL_PRINT(__msg.substring(29, 33));
//	_SERIAL_PRINT("< en Iam = >");
//	_SERIAL_PRINTLN(__Iam);
	if ((__msg.substring(27, 31) == __Iam))
	{
//		_SERIAL_PRINTLN("Voldoet aan Iam");
//		_SERIAL_PRINT("lib:sender =>");
//		_SERIAL_PRINT(__msg.substring(24, 28));
//		_SERIAL_PRINT("< en acceptFrom = >");
//		_SERIAL_PRINTLN(__acceptFrom);
		if (__acceptFrom == "*" || (__msg.substring(22, 26) == __acceptFrom))
		{
//			_SERIAL_PRINTLN("Voldoet aan acceptfrom");		  
			if ((_WIFIMessage.messageTypeId != messageStatusReply))
			{
				contentPositionInString = 40;
				for (int i=0;i<_WIFIMessage.totalContentItems+1;i++)
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
