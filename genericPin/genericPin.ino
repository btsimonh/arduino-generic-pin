#include "Arduino.h"
#include "LowLevel.h"
#include "OneWireSlave.h"
#include <EEPROM.h>
#include "idDHT11.h"


#define idDHT11pin 3        //Digital pin for comunications
#define idDHT11intNumber 1  //interrupt number (must be the one that use the previus defined pin (see table above)

//declaration
void dht11_wrapper(); // must be declared before the lib initialization

// Lib instantiate
idDHT11 DHT11(idDHT11pin,idDHT11intNumber,dht11_wrapper);


//#define SERIAL_TRACE // Comment to disable trace to serial port

#if defined SERIAL_TRACE
#define serialTrace(message) (Serial.println(message))
#else
#define serialTrace(message)
#endif

#define PIN_CONFIG 0x40 // Pin required to be configured
#define PIN_DIGITAL_OUTPUT 0x20 // Pin is digital output
#define PIN_PWM_OUTPUT 0x10 // Pin is PWM output
#define PIN_DIGITAL_INPUT 0x08 // Pin is digital input
#define PIN_DIGITAL_INPUT_PULLUP 0x04 // Enable internal Pull-up resistor (digital input)

// for nano,
// PWM: 3, 5, 6, 9, 10, and 11


#include <avr/wdt.h>
#ifndef _SOFT_RESTART_H
#define _SOFT_RESTART_H
#define soft_restart() \
do \
{ \
	wdt_enable(WDTO_15MS); \
	for(;;) { \
	} \
} while(0)
#endif

// setup pin 2 as 1-wire (requires interrupts)
Pin oneWireData(2);
// storage for our 1-wire address
byte owROM[7] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
const byte default_owROM[7] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
// This sample emulates a DS18B20 device (temperature sensor), so we start by defining the available commands
const byte DS18B20_START_CONVERSION = 0x44;
const byte DS18B20_READ_SCRATCHPAD = 0xBE;
const byte DS18B20_WRITE_SCRATCHPAD = 0x4E;
enum DeviceState
{
  DS_WaitingReset,
  DS_WaitingCommand,
  DS_ConvertingTemperature,
  DS_TemperatureConverted,
  DS_WaitingData,
};
volatile DeviceState state = DS_WaitingReset;

// scratchpad, with the CRC byte at the end
volatile byte scratchpad[20] = "scratchpad";
volatile byte scratchposn = 0;
volatile byte scratchpadin[20];
volatile byte scratchinposn = 0;


// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
	MCUSR = 0;
 	wdt_disable();
 	return;
}

const char *FIRMWARE_VERSION = "1.0.0";
const char *FIRMWARE_NAME = "GenericPin1Wire";

// these values are saved in EEPROM
const byte EEPROM_ID1 = 'G'; // used to identify if valid data in EEPROM
const byte EEPROM_ID2 = 'e'; // used to identify if valid data in EEPROM
const byte EEPROM_ID3 = 'n'; // used to identify if valid data in EEPROM
const byte LED_PIN = 14; // the number of the LED pin

const int ID_ADDR = 0x0000; // the EEPROM address used to store the ID
const int VER_ADDR = 0x0008; // the EEPROM address used to store the Firmware version
const int DEVICE_NAME_ADDR = 0x0040;
const int DEVICE_NAME_ADDR_END = 0x0080;

const int DIGITAL_PINS = 20;
const int ANALOG_PINS = 6;

// Each byte configs the digital pins 0-19 (14-19 are analog pins, if needed)
const int CONFIG_DIGITAL_ADDR = DEVICE_NAME_ADDR_END; // start address of digital pins
const int CONFIG_DIGITAL_ADDR_END = CONFIG_DIGITAL_ADDR + DIGITAL_PINS; // exlusive

const int STATE_DIGITAL_ADDR = CONFIG_DIGITAL_ADDR_END; // the state of digital pins stored here
const int STATE_DIGITAL_ADDR_END = STATE_DIGITAL_ADDR + (DIGITAL_PINS * 2); // exclusive

const int PWM_PINS[] = {3, 5, 6, 9, 10, 11};
const int PWM_PINS_SIZE = (sizeof(PWM_PINS) / sizeof(int));
const int STATE_ANALOG_ADDR = STATE_DIGITAL_ADDR_END; // state of PWM pins, 
const int STATE_ANALOG_ADDR_END = STATE_ANALOG_ADDR + DIGITAL_PINS; // state of PWM pins, 


const int MAX_COMMAND_LEN = 200;

byte digitalConfig[DIGITAL_PINS];
int digitalState[DIGITAL_PINS];


class CommandBuffer {
public:
	CommandBuffer() : p(0), ready(false) {
	}
	boolean append(const char c) {
		if((c == '\n') || (c == '\r')) {
			ready = true;
		} else if(p < MAX_COMMAND_LEN) {
			buffer[p++] = c;
		} else {
			ready = true;
		}
		return ready;
	}

	boolean append(const char *str) {
    int l = strlen(str);
    for (int i = 0; i < l; i++){
      char c = str[i];
      if(p < MAX_COMMAND_LEN-1) {
        buffer[p++] = c;
        if (c < 0x20)
          ready = true;
      }      
    }
    return ready;
  }

  boolean append(int num) {
    char t[6];
    sprintf(t, "%d", num);
    return append(t);
  }


	int length() const {
		return p;
	}
	char charAt(int pos) const {
		return ((pos < p) ? buffer[pos] : '\n');
	}
	void clean() {
		p = 0;
		ready = false;
	}
	boolean isReady() const {
		return ready;
	}
	int getDebugCommandLength() const {
		return ((length() >= 5) && (strncmp("DEBUG", buffer, 5) == 0)) ? 5 : 0;
	}

  // allow insert of whole command
  void setCommand(char *b, int len){
    memcpy(buffer, b, len);
    p = len;
    ready = true;
  }

  char *get(){
    buffer[p] = 0;
    return buffer;
  }
  
private:
	int p;
	boolean ready;
	char buffer[MAX_COMMAND_LEN];
};

CommandBuffer command;
CommandBuffer response;

class CommandProcessor {
private:	
	CommandBuffer& command;
	int p;
	boolean debug;
	int pin;
public:
	CommandProcessor(CommandBuffer& aCommand)
	: command(aCommand), p(command.getDebugCommandLength()), debug(p > 0), pin(-1) {
	} 

	boolean isDebug() const {
		return debug;
	}
	
	boolean process() {
		if(isDebug() && !skipSpaces()) {
			sendError("Whitespace required after DEBUG command");
			return false;
		}
		switch (getChar() | 0x20) {
		case 'c':
			return processConfigCommand();
		case 'd':
			return processDigitalCommand();
		case 'p':
			return processPWMCommand();
		case 'a':
			return processAnalogCommand();
		case 'n':
			return processNameCommand();
		case 'f':
			return processFirmwareCommand();
    case 'h':
      return processDHTCommand();
		default:
			return processUnknownCommand();
		}
	}
	
protected:
	char getChar() {
		return command.charAt(p++);
	}

	char peekChar() {
		return command.charAt(p);
	}

	void forwardChar() {
		++p;
	}

	// TODO: ? make stricter, error if space not found where it is required in command
	boolean skipSpaces() {
		int result = 0;
		for(char c = peekChar(); c == ' ' || c == '\t'; c = peekChar()) {
			++result;
			forwardChar();
		}
		return (result > 0);
	}
	
	boolean skipSpacesAndP() {
		skipSpaces();
		return (getChar()|0x20 == 'p');
	}

	char getAfterSpace() {
		skipSpaces();
		return getChar();
	}

	boolean parsePin() {
		if(!skipSpacesAndP()) {
			sendError("Pin not found");
			return false;
		}
		pin = parseInt();
		if(pin < 0) {
			sendError("Pin number not found");
			return false;
		} else if(pin < 2) {
			sendError("Access denied to serial pins");
			return false;
		} else if(pin >= DIGITAL_PINS) {
			sendError("There are no such many pins");
			return false;
		}
		return true;
	}

	boolean assertPWM() {
		for(int i = 0; i < PWM_PINS_SIZE; ++i) {
			if(PWM_PINS[i] == pin) return true;
		}
		sendError("Pin can't be used for PWM");
		return false;
	}

	boolean isDigit(char c) {
		return (c >= '0' && c <= '9');
	}

	int parseInt() {
		if(!isDigit(peekChar())) {
			sendError("Can't parse number");
			return -1;
		}
		int value = 0;
		int digits = 0;
		for(char c = peekChar(); c >= '0' && c <= '9'; c = peekChar()) {
			if(++digits > 4) {
				sendError("Number is too big");
				return -1;
			} 
			value = value * 10 + (c - '0');
			forwardChar();
		}
		return value;
	}

	void sendError(const char* message) {
		if(!debug) {
			//Serial.println(); // ESP8266 doesn't like it too (infinite loop)
			return;
		}
    response.append("ERROR at ");
		if(!message) {
      response.append("\r\n");
		} else {
			response.append(p);
			response.append(": ");
      response.append(message);
      response.append("\r\n");
		}
	}

	void sendOkPart() {
		response.append("OK ");
	}	

	boolean processConfigCommand() {
		char c = getChar() | 0x20;
		switch(c) {
		case 's':
		case 'e':
			if(!parsePin()) return false;
			return processConfigSetCommand(c == 'e');
		case 'g':
			if(!parsePin()) return false;
			return processConfigGetCommand();
		default:
			sendError("Unknown Config command");
			return false;
		}
	}

	boolean processConfigSetCommand(boolean saveToEEPROM) {
		byte config = PIN_CONFIG;
		int state = 0;
		char c;
		switch(getAfterSpace() | 0x20) {
		case 'i': // Pin is configured to be INPUT
			config |= PIN_DIGITAL_INPUT;
			c = getChar();
			if(c == '1') { // Turn on pullup resistor
				config |= PIN_DIGITAL_INPUT_PULLUP;
				pinMode(pin, INPUT_PULLUP);
			} else {
				pinMode(pin, INPUT);
			}
			state = digitalRead(pin);
			break;
		case 'o': // Pin is configured to be OUTPUT
			c = getChar() | 0x20;
			pinMode(pin, OUTPUT);
			if(c == 'd') {
				config |= PIN_DIGITAL_OUTPUT;
			} else if(c = 'p') {
				config |= PIN_PWM_OUTPUT;
				if(!assertPWM()) return false;
			} else {
				config |= PIN_DIGITAL_OUTPUT;
				sendError("Unknown output pin configuration");
				return false; // be strict
			}
			break;
		default:
			sendError("Unknown pin configuration");
			return false;
		}
		digitalConfig[pin] = config;
		digitalState[pin] = state;
		if(saveToEEPROM) {
			EEPROM.write(CONFIG_DIGITAL_ADDR + pin, config);
			saveInt(STATE_DIGITAL_ADDR + pin * 2, state);
		}
		return processConfigGetCommand();
	}

	boolean processConfigGetCommand() {
		byte config = digitalConfig[pin];
		sendOkPart();
		if(config & PIN_CONFIG) {
			if(config & PIN_DIGITAL_INPUT) {
				response.append("I");
				if(config & PIN_DIGITAL_INPUT_PULLUP) {
					response.append('1');
				} else {
					response.append('0');
				}
			} else if(config & PIN_PWM_OUTPUT) {
				response.append("OP");
			} else if(config & PIN_DIGITAL_OUTPUT) {
				response.append("OD");
			}
		}
		response.append("\r\n");
		return true;
	}

	boolean processDigitalCommand() {
		char c = getChar() | 0x20;
		switch(c) {
		case 's':
		case 'e':
			if(!parsePin()) return false;
			return processDigitalSetCommand(c == 'e');
		case 'g':
			if(!parsePin()) return false;
			return processDigitalGetCommand();
    case 'a': // get all pins input values
      return processDigitalGetAll();
		default:
			sendError("Unknown Digital command");
			return false;
		}
	}

	boolean processDigitalSetCommand(boolean saveToEEPROM) {
		char c = getAfterSpace() | 0x20;
		if(c != 'v') {
			sendError("Digital value to set not specified");
			return false;
		}
		int state = parseInt();
		if(state < 0) return false;
		if(state > 0) {
			state = 1;
		}
		pinMode(pin, OUTPUT);
		digitalWrite(pin, (state > 0) ? HIGH : LOW);
		byte config = PIN_CONFIG | PIN_DIGITAL_OUTPUT;
		digitalConfig[pin] = config;
		digitalState[pin] = state;
		if(saveToEEPROM) {
			EEPROM.write(CONFIG_DIGITAL_ADDR + pin, config);
			saveInt(STATE_DIGITAL_ADDR + pin * 2, state);
		}
		return processDigitalGetCommand();
	}


  boolean processDigitalGetAll(){
    sendOkPart();
    response.append("A");
    char t[10];
    sprintf(t, "%02.2X%02.2X%02.2X",
      PIND, PINB, PINC);
    response.append(t);
    response.append("\r\n");
    return true;
  }

	boolean processDigitalGetCommand() {
		byte config = digitalConfig[pin];
		sendOkPart();
		response.append("V");
		int state = 0;
		if(config & PIN_CONFIG) {
			if(config & PIN_DIGITAL_INPUT) {
				state = digitalRead(pin);
			} else if(config & (PIN_PWM_OUTPUT | PIN_DIGITAL_OUTPUT)) {
				state = digitalState[pin];
			}
		} else {
			pinMode(pin, INPUT);
			state = digitalRead(pin);
			digitalState[pin] = state;
		}
		response.append(state);
    response.append("\r\n");
		return true;
	}

	boolean processPWMCommand() {
		char c = getChar() | 0x20;
		switch(c) {
		case 's':
		case 'e':
			if(!parsePin()) return false;
			if(!assertPWM()) return false;
			return processPWMSetCommand(c == 'e');
		case 'g':
			if(!parsePin()) return false;
			return processDigitalGetCommand();
		default:
			sendError("Unknown PWM command");
			return false;
		}
	}

	boolean processPWMSetCommand(boolean saveToEEPROM) {
		char c = getAfterSpace() | 0x20;
		if(c != 'v') {
			sendError("PWM value to set not specified");
			return false;
		}
		int state = parseInt();
		if(state < 0) return false;
		if(state > 1023) state = 1023;
		analogWrite(pin, state / 4);
		byte config = PIN_CONFIG | PIN_PWM_OUTPUT;
		digitalConfig[pin] = config;
		digitalState[pin] = state;
		if(saveToEEPROM) {
			EEPROM.write(CONFIG_DIGITAL_ADDR + pin, config);
			saveInt(STATE_DIGITAL_ADDR + pin * 2, state);
		}
		return processDigitalGetCommand();
		
	}

	boolean processAnalogCommand() {
		char c = getChar() | 0x20;
		if(c != 'g') {
			sendError("Unknown Analog command");
			return false;
		}
		if(!parsePin()) return false;
		if(pin >= ANALOG_PINS) {
			sendError("Analog pin number is too big");
		}
		int state = analogRead(pin);
		sendOkPart();
		response.append("V");
		response.append(state);
    response.append("\r\n");
	}

  boolean processDHTCommand() {
    char c = getChar() | 0x20;
    if(c == 'a') {
      DHT11.acquire();
      sendOkPart();
      response.append("\r\n");
      return true;
    }
    
    if(c == 'g') {
      char res = DHT11.getStatus();
      sendOkPart();
      switch(res){
        case IDDHTLIB_READ: 
          response.append("R");
        case IDDHTLIB_OK: 
          {
          float h, t, d;
          h = DHT11.getHumidity();
          t = DHT11.getCelsius();
          d = DHT11.getDewPoint();
          char tmp[20];
          sprintf(tmp, "%d %d %d",
            (int)h*10, (int)t*10, (int)d*10); 
          response.append(tmp);
          DHT11.setread();
          }
          break;
        case IDDHTLIB_ERROR_CHECKSUM: 
          response.append("ECS");
          break;
        case IDDHTLIB_ERROR_ISR_TIMEOUT: 
          response.append("ETOI");
          break;
        case IDDHTLIB_ERROR_RESPONSE_TIMEOUT: 
          response.append("ETOR");
          break;
        case IDDHTLIB_ERROR_DATA_TIMEOUT: 
          response.append("ETOD");
          break;
        case IDDHTLIB_ERROR_ACQUIRING: 
          response.append("EA");
          break;
        case IDDHTLIB_ERROR_DELTA: 
          response.append("ED");
          break;
        case IDDHTLIB_ERROR_NOTSTARTED: 
          response.append("ES");
          break;
        default: 
          response.append("ERR");
          break;
      }
      response.append("\r\n");
      return true;
    }
    sendError("Unknown DHT command");
    return false;
  }

	boolean processNameCommand() {
		char c = getChar() | 0x20;
		switch(c) {
		case 's':
		case 'e':
			return processNameSetCommand();
		case 'g':
			return processNameGetCommand();
		default:
			sendError("Unknown Name command");
			return false;
		}
	}

	boolean processNameSetCommand() {
		skipSpaces();
		int addr = DEVICE_NAME_ADDR;
		for(char c = getChar(); addr < DEVICE_NAME_ADDR_END && c != '\0' && c != '\n'  && c != '\r' ; c = getChar(), ++addr) {
			saveChar(addr, c);
		}
		if(addr < DEVICE_NAME_ADDR_END) saveChar(addr, '\0');

    // reload 1wire address
    loadOneWire();
   
		return processNameGetCommand();
	}

	boolean processNameGetCommand() {
		sendOkPart();
		int addr = DEVICE_NAME_ADDR;
		for(char c = loadChar(addr); addr < DEVICE_NAME_ADDR_END && c != '\0'; c = loadChar(++addr)) {
			response.append(c);
		}
		response.append("\r\n");
	}

	boolean processFirmwareCommand() {
		char c = getChar() | 0x20;
		switch(c) {
		case 's':
		case 'e':
			return processFirmwareSetCommand();
		case 'g':
			return processFirmwareGetCommand();
		default:
			sendError("Unknown Firmware command");
			return false;
		}
	}

	boolean processFirmwareGetCommand() {
		skipSpaces();
		sendOkPart();
		char c = getChar() | 0x20;
		switch(c) {
		case 'v':
			response.append(FIRMWARE_VERSION);
			break;
		case 'n':
			response.append(FIRMWARE_NAME);
			break;
		default:
			response.append(FIRMWARE_NAME);
			response.append(' ');
			response.append(FIRMWARE_VERSION);
		}
		response.append("\r\n");
		return true;
	}

	boolean processFirmwareSetCommand() {
		skipSpaces();
		char c = getChar() | 0x20;
		switch(c) {
		case 'a':
			sendOkPart();
			initEEPROM();
			response.append("All configuration reset\n");
			break;
		case 'p':
			sendOkPart();
			initConfigEEPROM();
			response.append("Pin configuration reset\n");
			break;
		case 'r':
			sendOkPart();
			response.append("Restart\n");
			break;
		default:
			sendError("Unknown Firmware command");
			return false;
		}
		//soft_restart(); //infinite restart loop
		resetState();
		loadEEPROM();
	}

	boolean processUnknownCommand() {
		sendError("Unknown Command");
		return false;
	}
};

boolean checkEEPROM() {
	return (EEPROM.read(ID_ADDR) == EEPROM_ID1
		&& EEPROM.read(ID_ADDR + 1) == EEPROM_ID2
		&& EEPROM.read(ID_ADDR + 2) == EEPROM_ID3);
}

void initEEPROM() {
	serialTrace("Writing default data to EEPROM");
	EEPROM.write(ID_ADDR,EEPROM_ID1);
	EEPROM.write(ID_ADDR + 1,EEPROM_ID2);
	EEPROM.write(ID_ADDR + 2,EEPROM_ID3);
	for(int i = ID_ADDR + 3; i < CONFIG_DIGITAL_ADDR; ++i) {
		EEPROM.write(i, 0x00);
	}

	initConfigEEPROM();
  initOneWireEEPROM();
}

void initConfigEEPROM() {
	for(int i = CONFIG_DIGITAL_ADDR; i < STATE_ANALOG_ADDR_END; ++i) {
		EEPROM.write(i, 0x00);
	}
	for(int i = 0; i < DIGITAL_PINS; ++i) {
		digitalConfig[i] = 0x00;
	}
	for(int i = 0; i < DIGITAL_PINS; ++i) {
		digitalState[i] = 0;
	}
}

void initOneWireEEPROM(){
  loadOneWire();
}

void loadEEPROM() {
	serialTrace("Using data from EEPROM");
	loadPinConfig();
  loadOneWire();
}

void resetState() {
	for(int i = 2; i < DIGITAL_PINS; ++i) {
		pinMode(i, INPUT);
	}
}

void loadPinConfig() {
	for(int i = CONFIG_DIGITAL_ADDR; i < CONFIG_DIGITAL_ADDR_END; ++i) {
		byte b = EEPROM.read(i);
		int pin = i - CONFIG_DIGITAL_ADDR;
		int stateAddr = STATE_DIGITAL_ADDR + pin * 2;
		digitalConfig[pin] = b;
		if(b & PIN_CONFIG) {
			int state = loadInt(stateAddr);
			digitalState[pin] = state;
			if(b & PIN_DIGITAL_OUTPUT) {
				pinMode(pin, OUTPUT);
				digitalWrite(pin, state);
			} else if(b & PIN_PWM_OUTPUT) {
				analogWrite(pin, state);
			} else if(b & PIN_DIGITAL_INPUT_PULLUP) {
				pinMode(pin, INPUT_PULLUP);
			} else if(b & PIN_DIGITAL_INPUT) {
				pinMode(pin, INPUT);
			}
			if(b & PIN_DIGITAL_INPUT) {
				digitalState[pin] = digitalRead(pin);
			}
		} else {
			digitalState[pin] = 0;
		}
	}
}


void loadOneWire(){
  owROM[0] = 0xF0;
  // use name for 1wire address
  for(int i = DEVICE_NAME_ADDR; i < DEVICE_NAME_ADDR+6; ++i) {
    byte b = EEPROM.read(i);
    owROM[i-DEVICE_NAME_ADDR+1] = b;
  }

  // reset the 1-wire slave with new address  
  OWSlave.begin(owROM, oneWireData.getPinNumber()); 
}


int loadInt(int addr) {
	byte hiByte = EEPROM.read(addr);
	byte lowByte = EEPROM.read(addr+1);
	return word(hiByte, lowByte);
}

void saveInt(int addr, int value) {
	byte hiByte = highByte(value);
	byte loByte = lowByte(value);
	EEPROM.write(addr, hiByte);
	EEPROM.write(addr + 1, loByte);
}

char loadChar(int addr) {
	return (char) EEPROM.read(addr);
}

void saveChar(int addr, char value) {
	EEPROM.write(addr, (byte) value);
}


volatile unsigned long conversionStartTime = 0;

// This function will be called each time the OneWire library has an event to notify (reset, error, byte received)
void owReceive(OneWireSlave::ReceiveEvent evt, byte data);


void setup() {
  //memset(scratchpad, 0, sizeof(scratchpad));
  memset(scratchpadin, 0, sizeof(scratchpadin));
	Serial.begin(115200);

	// Setup the OneWire library
  OWSlave.setReceiveCallback(&owReceive);
  
  if(checkEEPROM()) {
		loadEEPROM();
	} else {
		initEEPROM();
	}

  // Setup the OneWire library
  OWSlave.begin(owROM, oneWireData.getPinNumber()); 


  
}

void dht11_wrapper() {
  DHT11.isrCallback();
}


void loop() {
	if(command.isReady()) {
    //Serial.print("\r\ngot\r\n");
    //Serial.print(command.get());
		CommandProcessor processor(command);
		processor.process();
		command.clean();

    // there should be a response ready.
    // put in scratchpad for onewire collection, and send to serial anyway.
    if(response.isReady()) 
    {
      char *t = response.get();
      // copy up until first cr/lf/nul
      for (char i = 0; i < sizeof(scratchpad); i++){
        if (t[i] < 0x20){
          scratchpad[i] = 0;
          scratchposn = i+1;
          break;
        } else {
          scratchpad[i] = t[i];
          scratchposn = i+1;
        }
      }
      Serial.print(t);
      response.clean();
    }
	}

}

void serialEvent() {
	while(Serial.available()) {
    char c = (char)Serial.read();
    // temp echo
    //Serial.print(c);
    
		if(command.append(c)) {
			if(Serial.available()){
        char n = (char)Serial.peek();
			
			  if ((n == '\r') || (n == '\n')) {
			
  				// Workaround for esp8266 sending welcome gibberish on start.
  				// Side effect of the workaround - if "\n\n" happened
  				// after entering command, the command is ignored.
  				Serial.read();
  				command.clean();
  			}
			}
			break;
		}
	}
}

void owReceive(OneWireSlave::ReceiveEvent evt, byte data)
{
  switch (evt)
  {
  case OneWireSlave::RE_Byte:
    switch (state)
    {
    case DS_WaitingData:
      if (scratchinposn < sizeof(scratchpadin)){
        scratchpadin[scratchinposn] = data;
        scratchinposn++;
        if (data < 0x20){
          command.setCommand(scratchpadin, scratchinposn);
          scratchinposn = 0;
        }
      }
      break;
    case DS_WaitingCommand:
      switch (data)
      {
      case DS18B20_START_CONVERSION:
        Serial.print('c');
        //state = DS_ConvertingTemperature;
        //conversionStartTime = millis();
        //OWSlave.beginWriteBit(0, true); // send zeros as long as the conversion is not finished
        break;

      case DS18B20_READ_SCRATCHPAD:
        Serial.print('r');
        state = DS_WaitingReset;
        OWSlave.beginWrite((const byte*)scratchpad, strlen(scratchpad)+1, 0);
        break;

      case DS18B20_WRITE_SCRATCHPAD:
        Serial.print('w');
        scratchinposn = 0;
        state = DS_WaitingData;
        break;
      }
      break;
    }
    break;

  case OneWireSlave::RE_Reset:
    state = DS_WaitingCommand;
    break;

  case OneWireSlave::RE_Error:
    state = DS_WaitingReset;
    break;
  }
}

