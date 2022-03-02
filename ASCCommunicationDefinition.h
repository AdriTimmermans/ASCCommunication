#define DEBUG_PRINT 1

#ifdef DEBUG_PRINT
	#define _SERIAL_BEGIN(x) Serial.begin(x);
	#define _SERIAL_PRINT(x) Serial.print(x);
	#define _SERIAL_PRINT_HEX(x) Serial.print(x, HEX);
	#define _SERIAL_PRINTLN(x) Serial.println(x);
	#define _SERIAL_PRINTLN_HEX(x) Serial.println(x, HEX);
	#define communicationDelay 5
#else
	#define _SERIAL_BEGIN(x)
	#define _SERIAL_PRINT(x)
	#define _SERIAL_PRINT_HEX(x)
	#define _SERIAL_PRINTLN(x)
	#define _SERIAL_PRINTLN_HEX(x)
	#define communicationDelay 5
#endif

#ifndef ESP8266 
	#ifndef ESP32
		#include <TimerOne.h>
	#endif
#endif

#define drivingForwards 1
#define drivingBackwards 2

byte semafoor;
byte SPILineStatusLight;

const 		byte 	maxSerialBufferLength	= 125; 
const 	 	byte 	messageHeaderLength 	= 13;
const		byte	maxEquipmentNumber	    = 29;
volatile 	int  	timerMode;
volatile 	bool 	dataOnSPI				= false;
volatile 	int  	SPIBufferPosition 		= 0;
volatile 	byte 	TimeOutCount 			= 0;
volatile 	bool 	messageDataStarted 		= false;
			long 	messageNumber 			= 0;
volatile 	int		byteToSend 				= 0;
volatile 	byte 	byteMessage[maxSerialBufferLength];
volatile 	bool 	hasData 				= false;
volatile 	int 	bytesReceived;
volatile 	bool 	messageTimedOut 		= false;
volatile 	int 	timeOutCount 			= 0;
volatile 	int 	pos;
volatile 	bool 	pushToMaster 			= false;
volatile 	bool 	active 					= false;

volatile	byte	slaveSendStopSequence[3];
volatile	byte	slaveReceiveStartSequence[3];
volatile	byte	slaveReceiveStopSequence[3];

bool	SPICommunicationOnSlaveIsInActive 	= true;
bool	SPICommunicationOnMasterIsInActive 	= true;
byte	thisCHEID							= 0;
String	thisCHEString						= "";
volatile 	bool	startingUp				= true;

enum topicList
{
  spreaderLightsTimeOut,
  messageTimeOut,
  functionReplyTimeOut
};

void moveStringToBytes (byte *_toBuffer, byte _startPositionTo, String _fromString, byte _StartPosition, byte _length);

typedef struct  positionTemplate gridPositionType;
struct positionTemplate {
  int x;
  int y;
  int orientationRelativeToGrid;
};
//                                               0        1        2        3        4        5        6        7        8        9        A        B        C        D        E        F        10       11       12       13       14       15       16       17       18       19       1A       1B       1C                                         
String equipmentNameList[maxEquipmentNumber] = {"GS00R", "S011R", "S012R", "A011R", "A012R", "T011R", "T012R", "L011R", "L012R", "C011R", "C012R", "Z001R", "Z002R", "GS00M", "S011M", "S012M", "A011M", "A012M", "T011M", "T012M", "L011M", "L012M", "C011M", "C012M", "Z001M", "Z002M", "TXRX ", "WIFI ", "UNDEF"};
// GS   = Groundstation
// Sxxx = AutoStraddle Carrier
// Axxx = Automated Guided Vehicle
// Txxx = Turtle
// Lxxx = Terminal Truck (or Turtle)
// Cxxx = ARTG (or Crane)
// Zxxx = Origin point (Zero)

String toDef(byte equipmentId)
{
		return equipmentNameList[min(equipmentId, maxEquipmentNumber)];
}

byte idOf (String equipmentName)
{
	int i;

	for (i=0;i<maxEquipmentNumber;i++)
	{
		if (equipmentName == equipmentNameList[i])
		{
			break;
		}
	}
	
	return i;
}

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
//                     0      1      2      3      4      5      6      7      8      9     10      11    12     13    14     15     16     17      18
String msgType[19] = {"nom","rrq", "rrp", "lrq", "lrp", "rac", "rna", "vcl", "vsl", "soq", "srp", "msr", "sdr", "mba", "lac", "tms", "tmr", "tmp", "rco"};

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
enum messageDirection
{
	sent,
	received
};
//                        0  1  2  3  4  5  6  7  8   9 10 11 12 13 14 15 16 17 18
byte responseTimes[19] = {0, 1, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 1, 0, 0};
//					   0	 1      2      3      4      5      6      7     8      9      10     11     12     13     14     15     16     17     18     19     20     21     22     23     24     25     26     27     28     29     30     31     32     33     34     35     36     37     38     39     40     41 
String msgCom[42] = {"noc","sts", "gos", "gol", "gor", "stt", "stb", "777", "888", "rqp", "sto", "exe", "122", "133", "144", "155", "166", "lsp", "uls", "sfd", "sbd", "stm", "smm", "rrp", "slg", "sof", "crs", "ron", "rof", "mus", "muk", "mos", "esc", "mto", "orf", "onf", "sfo", "rbu", "mdr", "smd", "dll", "tmd"};
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
	testMessageData				=41
//	manualRouteList				=97,
//	simpleAutoRouteList			=98,
//	complexAutoRouteList		=99
};

enum SPILineStates
{
	SPIinActive	=0,
	SPIActive		=1};

int mEngineCodes[11] = {0b10101010, 0b01010101, 0b10010110, 0b01101001, 0b10000010, 0b00101000, 0b00010100, 0b01000001, 0b01100110, 0b10011001, 0b00000000};

// motors order LF RF LR RR; 00 = stopped, 01 = forward, 10 = backward

typedef struct messageExchangeType aMessage;
struct messageExchangeType {

	byte startOfMessage[3];				 //	 Message start with 0xF1, 0xF0, 0xEF
	byte totalMessageLength;      		 //  Length of the message
	byte totalContentItems;		  		 //  Number of items to be added in the content of the message (3 bytes per item)
	byte messageNumberHighByte;	  		 //  two byte integer to identify the message, in combination with the sender
	byte messageNumberLowByte;	  		 //  two byte integer to identify the message, in combination with the sender
	byte CRCByte;				  		 //  Control byte. Least significant byte of sum of all bytes
	byte mediumId;               		     //  WIFI or TXRX
	byte senderId;               		     //  Main or radio moduleof center or of vehicle (4 characters + R for radiomodule or M for main module)
	byte addresseeId;            		     //  Main or radio moduleof center or of vehicle
	byte finalAddressId;         		     //  Endpoint of message 
	byte messageTypeId;           		 //  Type of message = of type "messageTypeIds" 
	byte content[maxMessageParameters];  //  1 message command of type "messageCommands" plus parameters (3bytes, chars), free text or (location x, y, orientation) depending on message Type
										 //  End of the message is signalled with 3 bytes 0xEF, 0xF0, 0xF1
};

aMessage mR_messageReceived;
aMessage mS_messageSend;

void waitFor(long numberOfMiliseconds);
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
  digitalWrite (intPin, LOW); //
  waitFor (2); // waitFor 2ms
  digitalWrite (intPin, HIGH); //
/*
  _SERIAL_PRINT("Trigger on pin ");
  _SERIAL_PRINT(intPin);
  _SERIAL_PRINT(", push to master = ");
  _SERIAL_PRINTLN(pushToMaster);
  */
}

void setSPIBusStatus (SPILineStates SPILineState)
{
	/*
	if (SPILineState == SPIinActive)
	{
		pinMode(semafoor, INPUT);
	}
	else
	{
		pinMode(semafoor, OUTPUT);
		digitalWrite(semafoor, LOW);
	}
	*/
}

SPILineStates getSPIBusStatus ()
{
	/*
	SPILineStates aux;
	if (digitalRead(semafoor) == HIGH)
	{
		aux = SPIinActive;
		//_SERIAL_PRINTLN("Semafoor = HIGH");
	}
	else
	{
		aux = SPIActive;
		//_SERIAL_PRINTLN("Semafoor = LOW");
	}
	*/
	return SPIinActive;
}

void displayMessage(aMessage * msg, bool outgoing)
{
	/*
start:      241 240 239 
length:     019 
params:     001 
messagnr:   000 045 
CRC:        037 
Medium:     027 
Sender:     001 
Receiver:   000 
Endstation: 000 
MsgTypeId:  011 
Parameter1: 029 000 000 
end:        239 240 241
*/
	if(outgoing)
	{
		_SERIAL_PRINTLN("-> OUTGOING:\t ");
	}
	else
	{
		_SERIAL_PRINTLN("<- INCOMING:\t ");
	}  
	
	_SERIAL_PRINT("start:      ");
	for (int i = 0; i < 3; i++)
	{
		_SERIAL_PRINT("\t 0x");
		_SERIAL_PRINT_HEX(msg->startOfMessage[i]);
	}
	_SERIAL_PRINTLN();
	
	_SERIAL_PRINT("length:     \t 0x");
	_SERIAL_PRINT_HEX(msg->totalMessageLength);    //  Length of the message
	_SERIAL_PRINT("\t ");
	_SERIAL_PRINTLN(msg->totalMessageLength);

	_SERIAL_PRINT("params:    \t 0x");
	_SERIAL_PRINT_HEX(msg->totalContentItems);     //  Number of items to be added in the content of the message (3 bytes per item)
	_SERIAL_PRINT("\t ");
	_SERIAL_PRINTLN_HEX(msg->totalContentItems);     //  Number of items to be added in the content of the message (3 bytes per item)
	
	_SERIAL_PRINT("messagenr:  \t 0x");
	_SERIAL_PRINT_HEX(msg->messageNumberHighByte);         //  two byte integer to identify the message, in combination with the sender
	_SERIAL_PRINT("\t 0x");
	_SERIAL_PRINT_HEX(msg->messageNumberLowByte);         //  two byte integer to identify the message, in combination with the sender
	_SERIAL_PRINT("\t (");
	long msgNr = (msg->messageNumberHighByte << 8) | msg->messageNumberLowByte;
	_SERIAL_PRINT(msgNr);
	_SERIAL_PRINTLN(")");
	
	_SERIAL_PRINT("CRC:        \t 0x");
	_SERIAL_PRINT_HEX(msg->CRCByte);               //  Control byte. Least significant byte of sum of all bytes
	_SERIAL_PRINT("\t ");
	_SERIAL_PRINTLN(msg->CRCByte);               //  Control byte. Least significant byte of sum of all bytes

	_SERIAL_PRINT("medium:     \t ");
	_SERIAL_PRINT(toDef(msg->mediumId));
	_SERIAL_PRINT("\t (");
	_SERIAL_PRINT(msg->mediumId);
	_SERIAL_PRINTLN(")");
	
	_SERIAL_PRINT("sender:     \t ");	
	_SERIAL_PRINT(toDef(msg->senderId));
	_SERIAL_PRINT("\t (");	
	_SERIAL_PRINT(msg->senderId);	
	_SERIAL_PRINTLN(")");

	_SERIAL_PRINT("receiver:   \t ");
	_SERIAL_PRINT(toDef(msg->addresseeId));
	_SERIAL_PRINT("\t (");	
	_SERIAL_PRINT(msg->addresseeId);	
	_SERIAL_PRINTLN(")");

	_SERIAL_PRINT("addressee:  \t ");
	_SERIAL_PRINT(toDef(msg->finalAddressId));
	_SERIAL_PRINT("\t (");	
	_SERIAL_PRINT(msg->finalAddressId);	
	_SERIAL_PRINTLN(")");
	
	_SERIAL_PRINT("msgTypeId:  \t ");
	_SERIAL_PRINT(msgType[msg->messageTypeId]);
	_SERIAL_PRINT("\t (");	
	_SERIAL_PRINT(msg->messageTypeId);	
	_SERIAL_PRINTLN(")");
	
	for (int i = 0; i < ((msg->totalContentItems + 1) * 3); i= i+3)
	{
		_SERIAL_PRINT("data:       ");
		if (msg->content[i] > 42)
		{
			_SERIAL_PRINT("\t 0x");
			_SERIAL_PRINT_HEX(msg->content[i]);
		}
		else
		{
			_SERIAL_PRINT("\t ");
			_SERIAL_PRINT(msgCom[msg->content[i]]);
		}
		_SERIAL_PRINT("\t 0x");
		_SERIAL_PRINT_HEX(msg->content[i+1]);
		_SERIAL_PRINT("\t 0x");
		_SERIAL_PRINTLN_HEX(msg->content[i+2]);
	}
}

aMessage createGeneralMessage (messageTypeIds msgType, messageCommands msgCmd, byte contentByte);
aMessage SPIReadMessageFromSlave(byte CSPin);

void dumpByteMessage(int bufferLength);

bool checkCRCByteCorrect (aMessage * __SerialMessage)
{
	
	////_SERIAL_PRINTLN("In checkCRCByteCorrect:");
	long aux = 0;
	bool auxReturn;
	
	for (int i=0;i<3;i++) aux+=__SerialMessage->startOfMessage[i];
	aux+=__SerialMessage->totalMessageLength;
	aux+=__SerialMessage->totalContentItems;
	aux+=__SerialMessage->messageNumberLowByte;
	aux+=__SerialMessage->messageNumberHighByte;
	aux+=__SerialMessage->mediumId;
	aux+=__SerialMessage->senderId;
	aux+=__SerialMessage->addresseeId;
	aux+=__SerialMessage->finalAddressId;
	aux+=__SerialMessage->messageTypeId;
	for (int i=0;i<(__SerialMessage->totalContentItems + 1) * 3;i++) aux+=__SerialMessage->content[i];
	//Serialprint("CRCValue = ");
	//Serialprint(aux, HEX);
	//Serialprint(", CRCValue & Mask =  ");
	//Serialprint(aux & 0x000000FF, HEX);
	//Serialprint(", CRCbyte =  ");	
	//Serialprint(__SerialMessage->CRCByte, HEX);	
	auxReturn = (__SerialMessage->CRCByte == (aux & 0x000000FF));
	//(auxReturn)?_SERIAL_PRINTLN("->CRC = OK"):_SERIAL_PRINTLN("->CRC = NOT OK");
	return auxReturn;	
	
}

void waitFor(long numberOfMiliseconds)
{
	long starttime = millis();
	while ((millis() - starttime) < numberOfMiliseconds) {};
}

void dumpByteMessage(int bufferLength)
{
	int dataLength;
	
	dataLength = (int)byteMessage[3];
	dataLength = min(dataLength, bufferLength);
	_SERIAL_PRINT("Message has ");
	_SERIAL_PRINT(dataLength);
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

#ifdef ESP8266 || ESP32
IRAM_ATTR void setDataOnSPIFlag()
#else
void setDataOnSPIFlag()
#endif	
{
	dataOnSPI = true;
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

aMessage prepareSerialMessage (String __sender, String __addressee, String __finalAddress, messageTypeIds __aMessageType, byte __contentItems, byte *__content)
{
	aMessage __SerialMessage;
    long aux = 0;
	
	messageNumber++;
	__SerialMessage.startOfMessage[0] 		= 0xF1;
	__SerialMessage.startOfMessage[1] 		= 0xF0;
	__SerialMessage.startOfMessage[2] 		= 0xEF;
	
	__SerialMessage.mediumId 				= idOf("TXRX ");
	__SerialMessage.senderId 				= idOf(__sender);    
	__SerialMessage.addresseeId			 	= idOf(__addressee);  
	__SerialMessage.finalAddressId 			= idOf(__finalAddress);   

	__SerialMessage.messageTypeId      		= (byte)__aMessageType;

	__SerialMessage.totalMessageLength 		= __contentItems * 3 + messageHeaderLength + 3;
	__SerialMessage.totalContentItems 		= __contentItems;
	__SerialMessage.messageNumberLowByte 	= (messageNumber & 0x00FF) ;
	__SerialMessage.messageNumberHighByte 	= (messageNumber & 0xFF00) >> 8;
	memset(__SerialMessage.content, 0, sizeof(__SerialMessage.content));
	copyBytes(__SerialMessage.content, 0, __content, 0, __contentItems * 3);
	__SerialMessage.content[__contentItems*3]   = 0xEF;
	__SerialMessage.content[__contentItems*3+1] = 0xF0;
	__SerialMessage.content[__contentItems*3+2] = 0xF1;
	
	for (int i=0;i<3;i++) aux+=__SerialMessage.startOfMessage[i];
	aux+=__SerialMessage.totalMessageLength;
	aux+=__SerialMessage.totalContentItems;
	aux+=__SerialMessage.messageNumberLowByte;
	aux+=__SerialMessage.messageNumberHighByte;
	aux+=__SerialMessage.mediumId;
	aux+=__SerialMessage.senderId;
	aux+=__SerialMessage.addresseeId;
	aux+=__SerialMessage.finalAddressId;
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
	
	aux.startOfMessage[0] 		= 0xF1;
	aux.startOfMessage[1] 		= 0xF0;
	aux.startOfMessage[2] 		= 0xEF;
	
	aux.totalMessageLength 		= messageHeaderLength + 6;
	aux.totalContentItems 		= 0x1;
	aux.messageNumberHighByte 	= 0x0;
	aux.messageNumberLowByte 	= 0x0;
	aux.mediumId 				= 0x1C;
	aux.senderId				= 0x1C;
	aux.addresseeId 			= 0x1C;
	aux.finalAddressId 			= 0x1C;
	aux.messageTypeId 			= msgType;
	memset (aux.content, 0, sizeof(aux.content));
	aux.content[0] 				= msgCmd;
	aux.content[1] 				= contentByte;	
	aux.content[2] 				= 0;
	aux.content[3] 				= 0xEF;
	aux.content[4] 				= 0xF0;
	aux.content[5] 				= 0xF1;
	
	for (int i=0;i<3;i++) auxCRC +=aux.startOfMessage[i];
	auxCRC +=aux.totalMessageLength;
	auxCRC +=aux.totalContentItems;
	auxCRC +=aux.messageNumberHighByte;
	auxCRC +=aux.messageNumberLowByte;
	auxCRC +=aux.mediumId;
	auxCRC +=aux.senderId;
	auxCRC +=aux.addresseeId;
	auxCRC +=aux.finalAddressId;
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

    for (i = 0; i < lengthMessage; i++)		
	{
		if (i == 3)
		{
			lengthMessage = *p;
		}
        SPI.transfer(*p++);
		waitFor(communicationDelay);
	}
    return lengthMessage;
	
  }  // end of SPI_writeAnything

bool SPIMasterPassDataFromMeToYou (int CSPin, aMessage SPIMessage)
{
	
	int 	bytesPassed;
	bool 	aux;
	long 	startTimeTransaction;

	digitalWrite(CSPin, LOW);
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	bytesPassed = SPI_writeAnything (SPIMessage);
	SPI.endTransaction();
	digitalWrite(CSPin, HIGH);
	waitFor(communicationDelay);
	aux = (bytesPassed > 0);
	return aux;
}

aMessage SPIMasterPassDataFromYouToMe (int CSPin, int allowedResponseTime)
{
	
	aMessage 	aux;
	byte 		stopSequence[3];
	long		startWaitTime;
	byte        startByte;
	int			maxTries 			= 250;
	int			tryCount 			= 0;
	
	//_SERIAL_PRINT("CS PIN = ");
	//_SERIAL_PRINTLN(CSPin);
	memset(stopSequence, 0, 3);
	
	for (int i=0;i<sizeof(byteMessage);i++)
	{
		byteMessage[i]=0;
	}
	if (dataOnSPI)
	{
		//_SERIAL_PRINTLN("SPI trigger already received");
	}
	else
	{
		//_SERIAL_PRINTLN("Wait for SPI trigger");
	}
	startWaitTime = millis();
	while ((!dataOnSPI)&&((millis()-startWaitTime)<allowedResponseTime)) 
	{
		delay(5);
	}
	
	if (dataOnSPI)
	{
		//_SERIAL_PRINTLN("SPI trigger received");
		digitalWrite (CSPin, LOW);
		waitFor(communicationDelay);
		SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
		startByte = SPI.transfer (1);   // initiate transmission
		tryCount++;
		waitFor(communicationDelay);
		//_SERIAL_PRINTLN("SPI.transfer (1);");
		//first read bytes until start of message
		while ( (startByte != 0xF1)                            &&
				((millis()-startWaitTime)< allowedResponseTime)&&
				(tryCount < maxTries))
		{
			startByte = SPI.transfer (1) ;
			tryCount++;
			waitFor(communicationDelay);
			//_SERIAL_PRINT_HEX(startByte);
			//_SERIAL_PRINT(" ");
		}

		byteMessage[2] = startByte;
		//_SERIAL_PRINTLN("Out of F1 Search");
		
		while ( !((byteMessage[0] == 0xF1)&&(byteMessage[1] == 0xF0)&&(byteMessage[2] == 0xEF)) &&
		         ((millis()-startWaitTime)<allowedResponseTime)                                 &&
				 (tryCount < maxTries))
		{
			byteMessage[0] = byteMessage[1];
			byteMessage[1] = byteMessage[2];
			byteMessage[2] = SPI.transfer (2);
			tryCount++;
			waitFor(communicationDelay);
			//_SERIAL_PRINT_HEX(byteMessage[2]);
			//_SERIAL_PRINT(" ");
		}
		//_SERIAL_PRINTLN("Out of F1 F0 EF Serach");
		if (!((byteMessage[0] == 0xF1)&&(byteMessage[1] == 0xF0)&&(byteMessage[2] == 0xEF)))
		{
			//_SERIAL_PRINTLN("Startsearch failed");
			aux = createGeneralMessage (messageStatusReply, errorSerialCommunication, 8);
		}
		else 
		{
			//
			// read rest of message
			//
			//_SERIAL_PRINTLN("Startsearch completed successfully");
			for (int posLocal = 3; posLocal < sizeof (byteMessage) - 1; posLocal++)
			{
				stopSequence[0] = stopSequence[1];
				stopSequence[1] = stopSequence[2];
				byteMessage [posLocal] = SPI.transfer (posLocal+2); // was 0
				//_SERIAL_PRINT(" 0x");
				//_SERIAL_PRINT_HEX(byteMessage[posLocal]);
				waitFor(communicationDelay);
				stopSequence[2] = byteMessage [posLocal];
				if ((stopSequence[0] == 0xEF) && (stopSequence[1] == 0xF0) && (stopSequence[2] == 0xF1))
				{
					break;
				}
				if (tryCount > maxTries) 
				{
					break;;
				}
			}
		}
		//_SERIAL_PRINTLN(" -- Message completed");
		memcpy (&aux, (byte *)byteMessage, sizeof(aux));
		//displayMessage(&aux, false);
		SPI.endTransaction();
		digitalWrite (CSPin, HIGH);
		waitFor(communicationDelay);  
	}
	else
	{
		_SERIAL_PRINTLN("NO TRIGGER received");
		 aux = createGeneralMessage (messageStatusReply, errorSerialCommunication, 9);	
	}
	
	return aux;
}

bool SPISlavePassDataFromMeToYou (int wakeUpPin, aMessage SPIMessage)
{
	
	int 	bytesPassed;
	bool 	aux;
	long 	startTimeTransaction;
	bool	otherSideIsReady;

	//_SERIAL_PRINTLN("SPISlavePassDataFromMeToYou with message: ");
	//displayMessage(&SPIMessage, true);
	memcpy((byte *)byteMessage, &SPIMessage, sizeof(byteMessage));
	pushToMaster = true;
	startTimeTransaction = millis();
	triggerInterrupt (wakeUpPin);
	while (((millis()-startTimeTransaction)<3000) && (pushToMaster)) {};
	/*
	_SERIAL_PRINT("SPISlavePassDataFromMeToYou:pushToMaster =>");	
	if (pushToMaster)
	{
		_SERIAL_PRINTLN(" true, no data requested");
	}
	else
	{
		_SERIAL_PRINTLN(" false, data retrieved");
	}
	*/
	aux = !pushToMaster;
	pushToMaster = false;
	//_SERIAL_PRINTLN("SPISlavePassDataFromMeToYou:force pushToMaster to false");	
	
	return aux;
}

aMessage SPISlavePassDataFromYouToMe (int allowedTime)
{
	
	int 		bytesPassed;
	aMessage 	aux;
	long 		startTimeTransaction;
	bool		waitOnData = true;

	//_SERIAL_PRINTLN("SPISlavePassDataFromYouToMe");
	startTimeTransaction = millis();
	aux = createGeneralMessage (messageStatusReply, errorSerialCommunication, 8);
	hasData = false;
	while ((waitOnData)&&((millis() - startTimeTransaction) < allowedTime))
	{
		if (hasData)
		{
			hasData = false;
			waitOnData = false;
			memcpy(&aux, (byte *)byteMessage, sizeof(aux));
			//_SERIAL_PRINTLN("Message received : ");
			//dumpByteMessage(40);
			SPIBufferPosition = 0;
		}
		else
		{
			waitFor(5);
		}
	}
	
	return aux;
}

aMessage sendMessageSPIFromMaster(int CSPin, aMessage SPIMessage)
{
	aMessage returnMessage;
	int allowedWaitingTime;
	bool comSucces;
	
	if (getSPIBusStatus() == SPIinActive)
	{
		setSPIBusStatus (SPIActive);
		displayMessage(&SPIMessage, true);
		comSucces = SPIMasterPassDataFromMeToYou (CSPin, SPIMessage);
		if (comSucces)
		{
			//_SERIAL_PRINTLN("Send was succesful");
			returnMessage = SPIMasterPassDataFromYouToMe (CSPin, 1000);
			//_SERIAL_PRINTLN(" Message status reply received from slave: ");
			displayMessage(&returnMessage, false);
			dataOnSPI = false;
			if (returnMessage.messageTypeId == messageStatusReply)
			{
				allowedWaitingTime = returnMessage.content[2] * 1000 ;
				if (allowedWaitingTime > 0)   // now the message send by the Master triggeres a content response from the slave, so wait
				{				
					_SERIAL_PRINT("Allowed waiting time: ");
					_SERIAL_PRINTLN(allowedWaitingTime);
					while (!dataOnSPI)
					{
						waitFor(5);
					}
					returnMessage = SPIMasterPassDataFromYouToMe (CSPin, allowedWaitingTime);
					displayMessage(&returnMessage, false);
					_SERIAL_PRINTLN(" Message reply to previous request received from slave: ");
					if (returnMessage.messageTypeId != errorSerialCommunication)
					{
						byte rawContentLocal[3];
						rawContentLocal[0] = messageUnderstood;
						rawContentLocal[1] = 0;
						rawContentLocal[2] = 0;			
						SPIMessage  = prepareSerialMessage (toDef(SPIMessage.senderId), toDef(SPIMessage.addresseeId), toDef(SPIMessage.finalAddressId), messageStatusReply,  1, &rawContentLocal[0]);
						displayMessage(&SPIMessage, true);
						comSucces = SPIMasterPassDataFromMeToYou (CSPin, SPIMessage); 
						if (!comSucces)
						{
							_SERIAL_PRINTLN("ERROR -- Send of reply of reply failed ");
							returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 3);
							displayMessage(&returnMessage, false);
						}
					}
					else
					{
						_SERIAL_PRINTLN("ERROR -- No valid answer withing allowed time ");						
						returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 4);
						displayMessage(&returnMessage, false);
					}
				}
			}
			else
			{
					_SERIAL_PRINTLN("ERROR -- Reply Status expected ");
					returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 5);
					displayMessage(&returnMessage, false);
			}
			setSPIBusStatus (SPIinActive);
		}
		else
		{
			_SERIAL_PRINTLN("ERROR --- Send failed");			
			returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 6);
			displayMessage(&returnMessage, false);
		}
	}
	else
	{
		_SERIAL_PRINTLN("ERROR --- SPIBus on slave = Active ");
		returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 7);
		displayMessage(&returnMessage, false);
	}
	
	return returnMessage;
}

bool sendStatusMessageSPIFromSlave (int wakeUp, aMessage SPIStatusMessage)
{
	bool comSucces;

	if (getSPIBusStatus() == SPIinActive)
	{
		setSPIBusStatus (SPIActive);
		displayMessage(&SPIStatusMessage, true);
		comSucces = SPISlavePassDataFromMeToYou (wakeUp, SPIStatusMessage);
	}
	else
	{
		comSucces = false;
	}
	
	return comSucces;
}

bool sendStatusMessageSPIFromMaster (int CSPin, aMessage SPIStatusMessage)
{
	bool comSucces;

	if (getSPIBusStatus() == SPIinActive)
	{
		setSPIBusStatus (SPIActive);
		displayMessage(&SPIStatusMessage, true);
		comSucces = SPIMasterPassDataFromMeToYou (CSPin, SPIStatusMessage);
	}
	else
	{
		comSucces = false;
	}
	
	return comSucces;
}

aMessage sendMessageSPIFromSlave(int wakeUp, aMessage SPIMessage)
{
	aMessage returnMessage;
	int allowedWaitingTime;
	bool comSucces;
	
	if (getSPIBusStatus() == SPIinActive)
	{
		setSPIBusStatus (SPIActive);
		displayMessage(&SPIMessage, true);
		comSucces = SPISlavePassDataFromMeToYou (wakeUp, SPIMessage);
		if (comSucces)
		{
			//_SERIAL_PRINTLN("Send was succesful");
			returnMessage = SPISlavePassDataFromYouToMe (1000);
			//_SERIAL_PRINTLN(" Message status reply received from master: ");
			displayMessage(&returnMessage, false);
			if (returnMessage.messageTypeId == messageStatusReply)
			{
				allowedWaitingTime = returnMessage.content[2] * 1000 ;
				if (allowedWaitingTime > 0)   // now the message send by the slave triggered a content response from the master, so wait
				{
					_SERIAL_PRINT("Waiting for reply : Allowed waiting time: ");
					_SERIAL_PRINTLN(allowedWaitingTime);
					returnMessage = SPISlavePassDataFromYouToMe (allowedWaitingTime);
					_SERIAL_PRINTLN("Message reply to previous request received from master: ");
					displayMessage(&returnMessage, false);
					if (returnMessage.messageTypeId != errorSerialCommunication)
					{
						byte rawContentLocal[3];
						rawContentLocal[0] = messageUnderstood;
						rawContentLocal[1] = 0;
						rawContentLocal[2] = 0;			
						SPIMessage  = prepareSerialMessage (toDef(SPIMessage.senderId), toDef(SPIMessage.addresseeId), toDef(SPIMessage.finalAddressId), messageStatusReply,  1, &rawContentLocal[0]);
						displayMessage(&SPIMessage, true);
						comSucces = SPISlavePassDataFromMeToYou (wakeUp, SPIMessage); 
						if (comSucces)
						{
							_SERIAL_PRINTLN("Message successful ");
						}
						else
						{
							_SERIAL_PRINTLN("ERROR -- Send of reply of reply failed ");
							returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 3);
							displayMessage(&returnMessage, false);
						}
					}
					else
					{
						_SERIAL_PRINTLN("ERROR -- No valid answer withing allowed time ");
						returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 4);
						displayMessage(&returnMessage, false);
					}
				}
			}
			else
			{
				_SERIAL_PRINTLN("ERROR -- Reply Status expected ");
				returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 5);
				displayMessage(&returnMessage, false);
			}
			setSPIBusStatus (SPIinActive);
		}
		else
		{
			_SERIAL_PRINTLN("ERROR --- Send failed");
			returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 6);
			displayMessage(&returnMessage, false);
		}
	}
	else
	{
		_SERIAL_PRINTLN("ERROR --- SPIBus on slave = Active ");
		returnMessage = createGeneralMessage (messageStatusReply, errorSerialCommunication, 7);
		displayMessage(&returnMessage, false);
	}

	return returnMessage;
}

aMessage receiveSPIMessageOnMaster(int CSPin)
{
	aMessage returnMessage;
	aMessage SPIMessage;

	returnMessage = SPIMasterPassDataFromYouToMe (CSPin, 1000);
	displayMessage(&returnMessage, false);
	if ((returnMessage.messageTypeId > 0) && (returnMessage.messageTypeId != messageStatusReply))
	{
		byte rawContentLocal[3];
		rawContentLocal[0] = messageUnderstood;
		rawContentLocal[1] = 0;
		rawContentLocal[2] = responseTimes[returnMessage.messageTypeId];			
		SPIMessage  = prepareSerialMessage (toDef(returnMessage.addresseeId), toDef(returnMessage.senderId), toDef(returnMessage.senderId), messageStatusReply,  1, &rawContentLocal[0]);
		displayMessage(&SPIMessage, true);
		SPIMasterPassDataFromMeToYou (CSPin, SPIMessage);
	}
	return returnMessage;
}
void initialiseSPICommunicationOnSlave (int wakeUpPin, aMessage initCommunicationMessage)
{
	displayMessage (&initCommunicationMessage, true);	
	while (SPICommunicationOnSlaveIsInActive)
	{
		mR_messageReceived = sendMessageSPIFromSlave(wakeUpPin, initCommunicationMessage);
		displayMessage (&mR_messageReceived, false);	
		SPICommunicationOnSlaveIsInActive = !((mR_messageReceived.messageTypeId == messageStatusReply) && (mR_messageReceived.content[0] == messageUnderstood));
		if (SPICommunicationOnSlaveIsInActive)
		{
			waitFor (random(1,5)*500);
		}
	}
}

void initialiseSPICommunicationOnMaster (int CSPin, aMessage initCommunicationMessage)
{
	//displayMessage (&initCommunicationMessage, true);	
	while (SPICommunicationOnMasterIsInActive)
	{
		mR_messageReceived = sendMessageSPIFromMaster(CSPin, initCommunicationMessage);
		SPICommunicationOnMasterIsInActive = !((mR_messageReceived.messageTypeId == messageStatusReply) && (mR_messageReceived.content[0] == messageUnderstood));
		if (SPICommunicationOnMasterIsInActive)
		{
			waitFor (random(1,5)*500);
		}
	}
}

#ifndef __AVR_ATmega2560__

#else
ISR (SPI_STC_vect)
{
	if (startingUp)
	{
		return;
	}
	// module on slave (main module of all equipmentNameList
	if (pushToMaster)
	{
		byte c = SPDR;
		//_SERIAL_PRINT_HEX(c);
		if (c == 1)  // starting new sequence?
		{
			//_SERIAL_PRINT(" starts new sequence with a '1' ");
			active = true;
			pos = 0;
			slaveSendStopSequence[0] = 0;
			slaveSendStopSequence[1] = 0;
			slaveSendStopSequence[2] = byteMessage[pos];
			SPDR = byteMessage [pos];   // send first byte
			//_SERIAL_PRINT(" position = ");
			//_SERIAL_PRINT(pos);
			//_SERIAL_PRINT(" byte 1: = 0x");
			//_SERIAL_PRINTLN_HEX(byteMessage [pos]);
			pos++;
			return;
		}
		
		if (!active)
		{
			//_SERIAL_PRINTLN("NOT ACTIVE");
			SPDR = 0;
			return;
		}
		//_SERIAL_PRINT(" byte x: = 0x");	
		//_SERIAL_PRINTLN_HEX(c);	
		SPDR = byteMessage [pos];
		slaveSendStopSequence[0] = slaveSendStopSequence[1];
		slaveSendStopSequence[1] = slaveSendStopSequence[2];
		slaveSendStopSequence[2] = byteMessage[pos];
		//_SERIAL_PRINT(" position = ");
		//_SERIAL_PRINT(pos);
		//_SERIAL_PRINT(" byte = 0x");
		//_SERIAL_PRINTLN_HEX(byteMessage [pos]);
		pos++;
		
		if (((slaveSendStopSequence[0] == 0xEF) && (slaveSendStopSequence[1] == 0xF0)&&(slaveSendStopSequence[2] == 0xF1))|| pos >= sizeof (byteMessage))
		{
			active = false;
			pushToMaster = false;
			//_SERIAL_PRINTLN("ISR(end found): pushToMaster => false");
		}
	}
	else
	{
		byte c = SPDR;
		//_SERIAL_PRINT("unsol : position ");
//		if ((c == 1) && (SPIBufferPosition == 0))
//		{
//			//
//			// Master expects a message, let us give one
//			// 
//			_SERIAL_PRINTLN("!!!!!!!!!!!!!!!!!!!");
//			_SERIAL_PRINTLN("Unexpected push");
//			_SERIAL_PRINTLN("!!!!!!!!!!!!!!!!!!!");
//			//mS_messageSend = createGeneralMessage (messageStatusReply, errorSerialCommunication, 9);
//			//memcpy((byte *)byteMessage, &mS_messageSend, sizeof(byteMessage));
//			pushToMaster = true;
//			_SERIAL_PRINTLN("force pushToMaster => true");
//			active = true;
//			pos = 0;
//			slaveSendStopSequence[0] = 0;
//			slaveSendStopSequence[1] = 0;
//			slaveSendStopSequence[2] = byteMessage[pos];
//			_SERIAL_PRINT(pos);
//			SPDR = byteMessage [pos];   // send first byte
//			pos++;
//			return;
//		}
		if (hasData)
		{
			//_SERIAL_PRINTLN("previous message not yet processed - ignore data");
		}
		else
		{
		//_SERIAL_PRINT_HEX(SPIBufferPosition);	
		//_SERIAL_PRINT(" getting data...  ");	
		//_SERIAL_PRINT_HEX(c);	
			if (!messageDataStarted)
			{
				slaveReceiveStartSequence[0] = slaveReceiveStartSequence[1];
				slaveReceiveStartSequence[1] = slaveReceiveStartSequence[2];
				slaveReceiveStartSequence[2] = c;
				messageDataStarted = ((slaveReceiveStartSequence[0] == 0xF1)&&(slaveReceiveStartSequence[1] == 0xF0)&&(slaveReceiveStartSequence[2] == 0xEF))	;
				//_SERIAL_PRINTLN(": search for start data  ");				
			}
			else
			{
				//_SERIAL_PRINTLN(": collect data  ");
				if (SPIBufferPosition == 0)
				{
					byteMessage[0] = 0xF1;
					byteMessage[1] = 0xF0;
					byteMessage[2] = 0xEF;
					SPIBufferPosition = 3;
					slaveReceiveStopSequence[1] = 0;
					slaveReceiveStopSequence[2] = 0;
				}
				
				slaveReceiveStopSequence[0] = slaveReceiveStopSequence[1];
				slaveReceiveStopSequence[1] = slaveReceiveStopSequence[2];
				slaveReceiveStopSequence[2] = c;

				if (SPIBufferPosition <  sizeof(byteMessage)) // as long as ther is room in the message buffer accept message
				{	
					byteMessage[SPIBufferPosition] = c;
					if ((slaveReceiveStopSequence[0] == 0xEF)&&(slaveReceiveStopSequence[1] == 0xF0)&&(slaveReceiveStopSequence[2] == 0xF1))
		//
		// end of message reached because of end sequence encountered	
					{
						//_SERIAL_PRINTLN(" End of message reached");
						hasData = true;
						messageDataStarted = false;
						memset(slaveReceiveStopSequence, 0, 3);
						memset(slaveReceiveStartSequence, 0, 3);
						SPIBufferPosition = 0;
					}
					else
					{
						SPIBufferPosition++;
					}
				}
				else
				{
					//_SERIAL_PRINTLN(" No valid message, no action (has data = false)");
					
					hasData = false;
					messageDataStarted = false;
					memset(slaveReceiveStopSequence, 0, 3);
					memset(slaveReceiveStartSequence, 0, 3);
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
//		case manualRouteList: 			return noChangeInStatus;
//		case simpleAutoRouteList: 		return noChangeInStatus;
//		case complexAutoRouteList: 		return noChangeInStatus;
	}
	return abortedRunMode;
}
	
aMessage prepareWIFIMessage (String __sender, String __addressee, String __finalAddress, messageTypeIds __aMessageType , byte __contentItems, byte *__content)
{
	aMessage __WIFIMessage;
    long aux = 0;
	
	messageNumber++;
	__WIFIMessage.startOfMessage[0]   = 0xF1;
	__WIFIMessage.startOfMessage[1]   = 0xF0;
	__WIFIMessage.startOfMessage[2]   = 0xEF;
	__WIFIMessage.totalMessageLength = messageHeaderLength + 3 + __contentItems * 3;
	__WIFIMessage.totalContentItems = __contentItems;
	__WIFIMessage.messageNumberLowByte = messageNumber & 0x00FF;
	__WIFIMessage.messageNumberHighByte = messageNumber & 0xFF00;
	__WIFIMessage.mediumId = idOf("WIFI ");
	__WIFIMessage.senderId= idOf(__sender);
	__WIFIMessage.addresseeId= idOf(__addressee);
	__WIFIMessage.finalAddressId= idOf(__finalAddress);
	__WIFIMessage.messageTypeId      = (byte)__aMessageType;
	copyBytes(__WIFIMessage.content, 0, __content, 0, __contentItems * 3);
	
	__WIFIMessage.content[__contentItems*3]   = 0xEF;
	__WIFIMessage.content[__contentItems*3+1] = 0xF0;
	__WIFIMessage.content[__contentItems*3+2] = 0xF1;
	
	for (int i=0;i<3;i++) 
	{
		aux+=__WIFIMessage.startOfMessage[i];
	}	
	aux+=__WIFIMessage.totalMessageLength;
	aux+=__WIFIMessage.totalContentItems;
	aux+=__WIFIMessage.messageNumberLowByte;
	aux+=__WIFIMessage.messageNumberHighByte;
	aux+=__WIFIMessage.mediumId;
	aux+=__WIFIMessage.senderId;
	aux+=__WIFIMessage.addresseeId;
	aux+=__WIFIMessage.finalAddressId;
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

bool acceptFrom (byte __You)
{
	return (EEPROM.read(__You+1) == 1);
}

aMessage receiveWIFIUnsolicitedData(String *__msgP)
{
	aMessage _WIFIMessage;
	int contentValue, contentPositionInString;
	String __msg = *__msgP;
	int WIFIMessageLength = __msg.length();
	 
	_WIFIMessage.startOfMessage[0] 			= __msg.substring( 0,  3).toInt();
	_WIFIMessage.startOfMessage[1] 			= __msg.substring( 3,  6).toInt();
	_WIFIMessage.startOfMessage[2] 			= __msg.substring( 6,  9).toInt();
	_WIFIMessage.totalMessageLength 		= __msg.substring( 9, 12).toInt();
	_WIFIMessage.totalContentItems 			= __msg.substring(12, 15).toInt();  
	_WIFIMessage.messageNumberHighByte 		= __msg.substring(15, 18).toInt();  
	_WIFIMessage.messageNumberLowByte 		= __msg.substring(18, 21).toInt();  
	_WIFIMessage.CRCByte 					= __msg.substring(21, 24).toInt();  
	_WIFIMessage.mediumId 					= __msg.substring(24, 27).toInt();  
	_WIFIMessage.senderId 					= __msg.substring(27, 30).toInt();  
	_WIFIMessage.addresseeId 				= __msg.substring(30, 33).toInt();  
	_WIFIMessage.finalAddressId 			= __msg.substring(33, 36).toInt();  
	_WIFIMessage.messageTypeId 				= __msg.substring(36, 39).toInt();  

	if (_WIFIMessage.addresseeId == EEPROM.read(0))
	{
		if (acceptFrom(_WIFIMessage.senderId))
		{
			if ((_WIFIMessage.messageTypeId != messageStatusReply))
			{
				contentPositionInString = 39;
				for (int i=0;i<_WIFIMessage.totalContentItems+1;i++)
				{   
					_WIFIMessage.content[i*3]   = __msg.substring(contentPositionInString,   contentPositionInString+3).toInt();					
					_WIFIMessage.content[i*3+1] = __msg.substring(contentPositionInString+3, contentPositionInString+6).toInt();					
					_WIFIMessage.content[i*3+2] = __msg.substring(contentPositionInString+6, contentPositionInString+9).toInt();					
					contentPositionInString = contentPositionInString + 9;
				}
			}
		}
		else
		{
			_WIFIMessage.messageTypeId  = noMessageActive;
		}
	}
	else
	{
		_WIFIMessage.messageTypeId  = noMessageActive;			
	}

	displayMessage(&_WIFIMessage, false);
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
