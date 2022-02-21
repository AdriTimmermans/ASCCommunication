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


int SPInoControlLight=2;
int SPIcontrolIsMineLight=3;


const byte maxSerialBufferLength = 125; 
const byte messageHeaderLength = 26;
volatile int  timerMode;
volatile bool dataOnSPI = false;
volatile long lastInterruptAtMilliseconds = 0;
volatile long interruptCount = 0;
volatile int  SPIBufferPosition = 0;
volatile long triggerCount = 0;
volatile long dataOnSPICount = 0;
volatile byte TimeOutCount = 0;
volatile long spiInterruptCalls = 0;
volatile bool messageDataStarted = false;
long messageNumber = 0;
long keepAliveTime;

enum topicList
{
  spreaderLightsTimeOut,
  messageTimeOut,
  functionReplyTimeOut
};

/*enum messageDirection
{
  received,
  sent
};
*/

enum SPILineStates
{
	controlIsMineIamSending=1,
	controlIsMineIamReceiving=2,
	controlIsYoursIamSending=3,
	controlIsYoursIamReceiving=4,
	noControl=5
};

SPILineStates currentSPIStatus = noControl;
SPILineStates previousSPIStatus = controlIsYoursIamReceiving;

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
int localWakeUpPin;


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

aMessage mR_messageReceived;
bool	 slaveMessageAlreadyRead = false;

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

void showSPIStatus(bool force)
{
  if (force)
  {
    Serial.print ("SPI Status = ");
    switch (currentSPIStatus)
    {
      case noControl:
        Serial.println(" no control ");
        break;
      case controlIsMineIamSending:
        Serial.println(" control to ME - i am sending");
        break;
      case controlIsMineIamReceiving:
        Serial.println(" control to ME - i am receiving");
        break;	
	  case controlIsYoursIamSending:
        Serial.println(" control to YOU - i am sending");
        break;
      case controlIsYoursIamReceiving:
        Serial.println(" control to YOU - i am receiving");
        break;	
    }
  }
  else
  {

    if (previousSPIStatus != currentSPIStatus)
    {
      Serial.print ("SPI Status = ");
      switch (currentSPIStatus)
      {
		  case noControl:
			Serial.println(" no control ");
			break;
		  case controlIsMineIamSending:
			Serial.println(" control to ME - i am sending");
			break;
		  case controlIsMineIamReceiving:
			Serial.println(" control to ME - i am receiving");
			break;	
		  case controlIsYoursIamSending:
			Serial.println(" control to YOU - i am sending");
			break;
		  case controlIsYoursIamReceiving:
			Serial.println(" control to YOU - i am receiving");
			break;	
      }
      previousSPIStatus = currentSPIStatus;
    }
  }
}

void signalOtherSide(int triggerMasterPin)
{
	if ((currentSPIStatus == noControl) || (currentSPIStatus == controlIsYoursIamReceiving) || (currentSPIStatus == controlIsMineIamSending))
	{
		triggerInterrupt(triggerMasterPin);
	}
	else
	{
		Serial.print("No trigger allowed in following status:");
		showSPIStatus(true);
	}
}

bool waitForOtherSideReady()
{
	bool aux;
	long startTimeStamp;
	startTimeStamp = millis();
	while ((not dataOnSPI) && ((millis()-startTimeStamp)<100)){};
	aux = dataOnSPI;
	if (dataOnSPI) 
	{
		Serial.println("Other side ready");
	}
	else
	{
		Serial.println("Other side BUSY BUSY BUSY");
	}

	return aux;
}

void displayMessage(aMessage * msg, bool outgoing)
{

   if(outgoing)
   {
	   Serial.print("-------------->OUTGOING:\t ");
   }
   else
   {
	   Serial.print("<-----INCOMING:         \t");
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
volatile byte startByteOne = 0;
volatile byte startByteTwo = 0;
volatile byte startByteThree = 0;
aMessage createGeneralMessage (messageTypeIds msgType, messageCommands msgCmd, byte contentByte);
aMessage SPIReadMessageFromSlave(byte CSPin);
void pauze(long numberOfMiliseconds);

bool checkCRCByteCorrect (aMessage * __SerialMessage)
{
	
	long aux = 0;
	long CRCTotalValue = 0 ;
	
	aux+=__SerialMessage->totalMessageLength;
	aux+=__SerialMessage->totalContentItems;
	aux+=__SerialMessage->senderCode;
	aux+=__SerialMessage->messageNumberLowByte;
	aux+=__SerialMessage->messageNumberHighByte;
	for (int i=0;i<4;i++) aux+=__SerialMessage->medium[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage->sender[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage->addressee[i];
	for (int i=0;i<5;i++) aux+=__SerialMessage->finalAddress[i];
	aux+=__SerialMessage->messageTypeId;
	for (int i=0;i<(__SerialMessage->totalContentItems + 1) * 3;i++) aux+=__SerialMessage->content[i];

	return (__SerialMessage->CRCByte == (aux & 0x000000FF));	
	
}

void pauze(long numberOfMiliseconds)
{
	long starttime = millis();
	while ((millis() - starttime) < numberOfMiliseconds) {};
}

aMessage waitForAnswerMaster(int CSPin, messageTypeIds requestedAnswer, long allowedTime)
{
	aMessage aux;
	long startTime = millis();
	bool requestedAnswerIsReceived = false;

	while (!dataOnSPI && ((millis() - startTime) < allowedTime)) {};
	
	if (dataOnSPI)
	{
		signalOtherSide(localWakeUpPin);
		dataOnSPI = false;
		aux = SPIReadMessageFromSlave(CSPin);

		if (aux.messageTypeId == requestedAnswer)
		{
			startTime = millis() - allowedTime -10;
			requestedAnswerIsReceived = true;
		}
	}

	if (!requestedAnswerIsReceived)
	{
		aux = createGeneralMessage (noMessageActive, errorSerialCommunication, 6);		
	}
	//Serial.print("waitForAnswerMaster control change ");
	currentSPIStatus = noControl;
	showSPIStatus(true);

	digitalWrite(SPIcontrolIsMineLight, LOW);
	digitalWrite(SPInoControlLight, HIGH);	

	return aux;
}

aMessage waitForAnswerSlave(int wakeUp, messageTypeIds requestedAnswer, long allowedTime)
{
	aMessage aux;
	long startTime = millis();
	bool requestedAnswerIsReceived = false;
	
	while (!dataOnSPI && ((millis() - startTime) < allowedTime)) {};
	
	if (dataOnSPI)
	{
		signalOtherSide(localWakeUpPin);
		dataOnSPI = false;
		while ((millis() - startTime) < allowedTime)
		{
			if (hasData)
			{
				hasData = false;
				memcpy(&aux, (byte *)byteMessage, sizeof(aux));
				SPIBufferPosition = 0;
				if (aux.messageTypeId == requestedAnswer)
				{
					startTime = millis() - allowedTime -10;
					requestedAnswerIsReceived = true;
				}	
			}
			else
			{
				delay(5);
			}
		}
		dataOnSPI = false;
	}
	else
	{
		Serial.println("No trigger on SPI found");
	}
	
	if (!requestedAnswerIsReceived)
	{
		aux = createGeneralMessage (noMessageActive, errorSerialCommunication, 6);		
	}
	//Serial.print("waitForAnswerMaster control change ");
	currentSPIStatus = noControl;
	showSPIStatus(true);

	digitalWrite(SPIcontrolIsMineLight, LOW);
	digitalWrite(SPInoControlLight, HIGH);	

	return aux;
}

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
	showSPIStatus(true);
	if ((currentSPIStatus == noControl) || (currentSPIStatus == controlIsYoursIamReceiving) || (currentSPIStatus == controlIsMineIamReceiving) || (currentSPIStatus == controlIsMineIamSending))
	{
		if (interruptCount == 0)
		{
			lastInterruptAtMilliseconds = millis();
			interruptCount++;
		}
		dataOnSPICount++;
		dataOnSPI =((millis() - lastInterruptAtMilliseconds) > 500);
		
	}
	else
	{
		Serial.println("I am busy - no interrupt allowed");
		showSPIStatus(true);
		dataOnSPI = false;		
	}
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
	
	return __SerialMessage;
}

aMessage createGeneralMessage (messageTypeIds msgType, messageCommands msgCmd, byte contentByte)
{
	aMessage aux;
	long auxCRC = 0;
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
	
	SPI.transfer(0xF1);
	delay(5);
	SPI.transfer(0xF0);
	delay(5);
	SPI.transfer(0xEF);
	delay(5);
    for (i = 0; i < lengthMessage; i++)		
	{
        SPI.transfer(*p++);
		delay(5);
	}
    return lengthMessage;
	
  }  // end of SPI_writeAnything

aMessage SPIReadMessageFromSlave(byte CSPin)
{
	aMessage aux;
	byte byteStopOne;
	byte byteStopTwo;
	byte byteStopThree;
	byte byteStartOne;
	byte byteStartTwo;
	byte byteStartThree;
	byte firstByte;
	long startWaitTime;

	// enable Slave Select
    //_SERIAL_PRINTLN(" Start reading from slave in status ");//
	//Serial.print(currentSPIStatus);
	digitalWrite(SPIcontrolIsMineLight, LOW);
	digitalWrite(SPInoControlLight, LOW);			

	if ((currentSPIStatus == controlIsYoursIamReceiving) || (currentSPIStatus == controlIsMineIamReceiving))
	{
		digitalWrite (CSPin, LOW);
		delay(15);
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		SPI.transfer (1);   // initiate transmission
		delay(5);
		byteStopOne = 0;
		byteStopTwo = 0;
		byteStopThree = 0;
		byteStopOne = byteStopTwo;
		byteStopTwo = byteStopThree;

		byteStartOne = 0;
		byteStartTwo = 0;
		byteStartThree = 0;
		byteStartThree = SPI.transfer (2);


// first read bytes until start of message
		startWaitTime = millis();
		while (!((byteStartOne == 0xF1)&&(byteStartTwo == 0xF0)&&(byteStartThree == 0xEF)&&((millis()-startWaitTime)<200)))
		{
			byteStartOne = byteStartTwo;
			byteStartTwo = byteStartThree;
			byteStartThree = SPI.transfer (2);
			delay(5);
		}
		
// first read bytes until start of message
//
		if (!((byteStartOne == 0xF1)&&(byteStartTwo == 0xF0)&&(byteStartThree == 0xEF)))
		{
			aux = createGeneralMessage (noMessageActive, errorSerialCommunication, 6);
			_SERIAL_PRINTLN(" - No message");
		}
		else 
		{
//
// read rest of message
//
			for (int posLocal = 0; posLocal < sizeof (byteMessage) - 1; posLocal++)
			{
				byteStopOne = byteStopTwo;
				byteStopTwo = byteStopThree;
				byteMessage [posLocal] = SPI.transfer (posLocal+2); // was 0
				//Serial.print("- ");
				//Serial.print(posLocal);
				//Serial.print(",");
				//Serial.print(byteMessage[posLocal]);

				delay(5);
				byteStopThree = byteMessage [posLocal];
				if ((byteStopOne == 0xEF) && (byteStopTwo == 0xF0) && (byteStopThree == 0xF1))
				{
					//Serial.print("number of chars requested: ");
					//Serial.println(posLocal);
					break;
				}
			}
			memcpy (&aux, (byte *)byteMessage, sizeof(aux));
			//Serial.println("---");
		} 
		SPI.endTransaction();

	  // disable Slave Select
		digitalWrite (CSPin, HIGH);
		delay(5);  
	}
	else
	{
		aux = createGeneralMessage (noMessageActive, errorSerialCommunication, 7);
		_SERIAL_PRINTLN(" - Wrong SPI Status");
	}
	currentSPIStatus = noControl;
	return aux;
}
/*
void claimSPIchannel(int semafoor, int CSorWakeUp)
{
	Serial.println("Claim Channel");
	digitalWrite(SPIRequestLight, HIGH);
	while (digitalRead(semafoor) == LOW)
	{
		delay(10);
	}
	pinMode(semafoor, OUTPUT);
	digitalWrite(semafoor, LOW);
	digitalWrite(SPIRequestLight, LOW);
	digitalWrite(SPIControlLight, HIGH);
	delay(10);
	Serial.println("Channel claimed");
	delay(50);
}
*/
/*
void waitUntilSPIChannelClaimed(int semafoor)
{
	Serial.println("waitUntilSPIchannelClaimed");
	digitalWrite(SPIRequestLight, HIGH);
	while (digitalRead(semafoor) == HIGH)
	{
		delay(10);
	}
	digitalWrite(SPIRequestLight, LOW);
	digitalWrite(SPIControlLight, HIGH);
	delay(10);
	Serial.println("Channel claimed by other side");
}*/
/*
void releaseSPIChannel(int semafoor)
{
	Serial.println("Release Channel");
	pinMode(semafoor, INPUT);
	
// if error occurred in the communication, just let all communication time-outs run out and then start again
	digitalWrite(SPIControlLight, LOW);
	//delay(10);
}
*/

bool sendSPIMessageOneWayMaster (int CSPin, int WakeUpPin, aMessage SPIMessage)
{
	
	int 	bytesPassed;
	bool 	aux;
	long 	startTimeTransaction;
	bool	otherSideIsReady;

    //Serial.print("sendSPIMessageOneWayMaster: currentSPIStatus = ");
	showSPIStatus(true);
	//Serial.println("sendSPIMessageOneWayMaster -start- Change control: ");

	digitalWrite(CSPin, LOW);
	signalOtherSide (WakeUpPin);
	otherSideIsReady = waitForOtherSideReady();
	if (otherSideIsReady)
	{
		if (SPIMessage.messageTypeId == messageStatusReply)
		{
			currentSPIStatus = controlIsYoursIamSending;
		}
		else
		{
			currentSPIStatus = controlIsMineIamSending;
		}
		showSPIStatus(true);
		digitalWrite(SPIcontrolIsMineLight, HIGH);
		digitalWrite(SPInoControlLight, LOW);
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		bytesPassed = SPI_writeAnything (SPIMessage);
		SPI.endTransaction();
		//Serial.print("Number of bytes passed: ");
		//Serial.println(bytesPassed);
		//Serial.println("sendSPIMessageOneWayMaster -end- ");
		digitalWrite(CSPin, HIGH);
		delay(5);
		aux = (bytesPassed > 0);
	}
	else
	{
		Serial.print("sendSPIMessageOneWayMaster: Error - other side busy");
		aux = false;
		dataOnSPI = false;
	}
	
	return aux;
}

bool sendSPIMessageOneWaySlave (int WakeUp, aMessage SPIMessage)
{
	int 	bytesPassed;
	bool 	aux;
	long 	startTimeTransaction;
	bool	otherSideIsReady;

    //Serial.print("sendSPIMessageOneWaySlave: currentSPIStatus = ");
	showSPIStatus(true);
	memcpy((byte *)byteMessage, &SPIMessage, sizeof(byteMessage));
	for (int i=sizeof(byteMessage)+2;i>2;i--)
	{
		byteMessage[i] = byteMessage[i-3];
	}
	byteMessage[0] = 0xF1;
	byteMessage[1] = 0xF0;
	byteMessage[2] = 0xEF;
	pushToMaster = true;
	startTimeTransaction = millis();
	signalOtherSide(WakeUp);
	otherSideIsReady = waitForOtherSideReady();
	if (otherSideIsReady)
	{
		if (SPIMessage.messageTypeId == messageStatusReply)
		{
			currentSPIStatus = controlIsYoursIamSending;
		}
		else
		{
			currentSPIStatus = controlIsMineIamSending;
		}
		digitalWrite(SPIcontrolIsMineLight, HIGH);
		digitalWrite(SPInoControlLight, LOW);
		while (((millis()-startTimeTransaction)<3000) && (pushToMaster)) {}
		aux = !pushToMaster;
		//Serial.print("sendSPIMessageOneWaySLAVE -end- ");
	}
	else
	{
		Serial.print("sendSPIMessageOneWaySlave Error - other side busy");
		aux = false;
	}
	//waitUntilSPIChannelClaimed(semafoorLocal);
	while (((millis()-startTimeTransaction)<3000) && (pushToMaster)) {}
	aux = !pushToMaster;
	//Serial.print("sendSPIMessageOneWaySLAVE -end- ");

	pushToMaster = false;

	return aux;
}

bool sendSPIMessageMaster (int CSPin, int WakeUpPin, aMessage SPIMessage)
{
	bool 		aux;
	bool		success;
	long		intervalBegin;
	long 		intervalEnd;

	//Serial.print("--Sending up: ");
	intervalBegin = millis();
	success = sendSPIMessageOneWayMaster (CSPin, WakeUpPin, SPIMessage);

	if (success)
	{
		if (SPIMessage.messageTypeId == messageStatusReply)
		{	
			//Serial.println("messageStatusReply message sent successfully, now continue immediately");
			aux = true;
		}
		else
		{
			showSPIStatus(true);
			//Serial.println("non-messageStatusReply message sent successfully, now wait for answer");
			mR_messageReceived = waitForAnswerMaster(CSPin, messageStatusReply, 1500);
			success = (mR_messageReceived.messageTypeId == messageStatusReply);

			if (success)
			{
				//Serial.println("Master: Message Status reply received");
				aux = true;
			}
			else
			{
				Serial.println("Master: No correct answer");
				aux = false;
			}
		}
		
		intervalEnd = millis();
		//Serial.print("sendSPIMessageOneWayMaster takes: ");
		//Serial.println (intervalEnd - intervalBegin);
	}
	else
	{
		Serial.println("message failed");
		delay(1);
		aux = false;
	}
	currentSPIStatus = noControl;
	showSPIStatus(true);
	
	return aux;
}

bool sendSPIMessageSlave (int WakeUp, aMessage SPIMessage)
{
	bool 		aux;
	bool		success;	
	long		intervalBegin;
	long 		intervalEnd;

	intervalBegin = millis();
	success = sendSPIMessageOneWaySlave (WakeUp, SPIMessage);
	if (success)
	{
		if (SPIMessage.messageTypeId == messageStatusReply)
		{	
			//Serial.println("messageStatusReply message sent successfully, now continue immediately");
			aux = true;
		}
		else
		{
			showSPIStatus(true);
			//Serial.println("non-messageStatusReply message sent successfully, now wait for answer");
			mR_messageReceived = waitForAnswerSlave(WakeUp, messageStatusReply, 1500);
			success = (mR_messageReceived.messageTypeId == messageStatusReply);
			if (success)
			{
				//Serial.println("Slave: Message Status reply received");
				aux = true;
			}
			else
			{
				Serial.println("Slave: No correct answer");
				aux = false;
			}

		}
		intervalEnd = millis();
		//Serial.print("sendSPIMessageOneWaySlave takes: ");
		//Serial.println (intervalEnd - intervalBegin);
		//Serial.println("--exit sendSPIMessageSlave: ");
		}
	else
	{
		Serial.println("message sent not successfully");
		aux = false;
	}
	currentSPIStatus = noControl;
	showSPIStatus(true);

	return aux;
}

#ifndef ESP8266
ISR (SPI_STC_vect)
{
	// module on slave (main module of all equipmentNameList
	spiInterruptCalls++;
	if (pushToMaster)
	{
		byte c = SPDR;
		//Serial.print(" pTM:");
		//Serial.print(c);
		if (c == 1)  // starting new sequence?
		{
			active = true;
			pos = 0;
			//Serial.print(", ");
			//Serial.print(pos);
			//Serial.print(", ");			
			//Serial.print(byteMessage [pos]);
			endByteOne = 0;
			endByteTwo = 0;
			//Serial.print(",  First interrupt for send ");
			//Serial.print(currentSPIStatus);
			//Serial.print("->");
			//Serial.println(currentSPIStatus);	
			digitalWrite(SPIcontrolIsMineLight, HIGH);
			digitalWrite(SPInoControlLight, LOW);		

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
		//Serial.print(", ");
		//Serial.print(pos);
		//Serial.print(", ");
		//Serial.println(byteMessage [pos]);
		endByteOne = endByteTwo;
		endByteTwo = endByteThree;
		endByteThree = byteMessage[pos];

		if (((endByteOne == 0xEF) && (endByteTwo == 0xF0)&&(endByteThree == 0xF1))|| ++pos >= sizeof (byteMessage))
		{
			active = false;
			pushToMaster = false;
			//Serial.println("EOL send ");			  
		}
	}
	else
	{
		byte c = SPDR;
		Serial.print(" uns:");
		Serial.println(c);
		if (!messageDataStarted)
		{
			startByteOne = startByteTwo;
			startByteTwo = startByteThree;
			startByteThree = c;
			messageDataStarted = ((startByteOne == 0xF1)&&(startByteTwo == 0xF0)&&(startByteThree == 0xEF))	;		
		}
		else
		{

			if (SPIBufferPosition == 0)
			{
				Serial.println("First interrupt for unsoll. ");
				endByteTwo = 0;
				endByteThree = 0;
				digitalWrite(SPIcontrolIsMineLight, LOW);
				digitalWrite(SPInoControlLight, LOW);		
			}
			
			if (!dataOnSPI)
			{
				_SERIAL_PRINTLN("Rogue data found ");
				hasData = false;
				endByteOne = 0;
				endByteTwo = 0;
				endByteThree = 0;
				SPIBufferPosition = 0;		
			}
			else
			{
				endByteOne = endByteTwo;
				endByteTwo = endByteThree;
				endByteThree = c;

				if (SPIBufferPosition <  sizeof(byteMessage)) // as long as ther is room in the message buffer accept message
				{	
					byteMessage[SPIBufferPosition] = c;
					SPIBufferPosition++;
					if ((endByteOne == 0xEF)&&(endByteTwo == 0xF0)&&(endByteThree == 0xF1))
		//
		// end of message reached because of end sequence encountered	
					{
						hasData = true;
						messageDataStarted = false;
						//Serial.println("EOL received");			
						endByteOne = 0;
						endByteTwo = 0;
						endByteThree = 0;
						SPIBufferPosition = 0;
						dataOnSPI= false;
					}
				}
				else
				{
					_SERIAL_PRINTLN("No end of line found ");
					hasData = true;
					messageDataStarted = false;
					endByteOne = 0;
					endByteTwo = 0;
					endByteThree = 0;
					SPIBufferPosition = 0;
				}
			}
		}
	}
} 
#endif

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
