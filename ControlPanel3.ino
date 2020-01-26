/*
Management of track switches:
1) Monitor manual change push buttons
2) Monitor manual track switch state sensors
3) Control track switch state LED's
4) Control reversing of switch power
5) Control mechanical movement of track switch
*/

///////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////
// PIN Assignments
const int PIN_ONBOARDLED =  LED_BUILTIN;	// built in pin for onboard LED
const int PIN_DATA = 12;					// pin assigned to read and write data (1 bit design, I/O)
const int PIN_ADDSEL0 = 2;					// pins drive decoding of selected item (a relay, a switch, an LED control, a sensor
const int PIN_ADDSEL1 = 3;					// 3 pins enable selection of up to 8 items per clock (read or write)
const int PIN_ADDSEL2 = 4;					// 3 pins enable selection of up to 8 items per clock (read or write)

const int PIN_CLKADDR0 = 5;					// pins allow up to 8 clocks (read or write)
const int PIN_CLKADDR1 = 6;					// same
const int PIN_CLKADDR2 = 7;					// same
const int PIN_CLKACTION = 8;				// pin used to generate a clock pulse (to read, or write or close relay to switch machine)
const int PIN_SWTPWRREVERSE = 9;			// pin used to reverse polarity of switch machine power 

const int CLK_READCTRLPANEL = 0;			// CLK used to gate a button read onto the data bus
const int CLK_CHANGESWITCH = 1;				// CLK used to pulse a relay closed
const int CLK_READSWITCHSTATE = 2;			// CLK used to gate a read of the switch sensors
const int CLK_READTRACKSENSORS = 3;
const int CLK_WRITESIGNALSTATE = 4;

// Time-based constants in msec
const unsigned long HEARTBEAT = 1000;				// provide heartbeat pules on PIN_ONBOARDLED (led changes state every second)
const unsigned long HEARTBEAT_PANELCHECK = 30;		// read control panel at this frequency when idle
const unsigned long HEARTBEAT_TRACKSENSECHECK = 5000;// time between checks of track sensors

const unsigned long BUS_SETTLETIME = 5;				// time to wait for bus to settle into a state

const unsigned long RELAY_PULSETIME = 100;			// time a switch machine receives power
const unsigned long RELAY_REVERSETIME = 100;		// time to wait after we reverse polarity
const unsigned long RELAY_SETTLETIME = 20;
const unsigned long WATCHDOG_TIMEOUT = 1000;		// State machine should complete in less the this time
const unsigned long HARDWARE_DEBUG_HEARTBEAT = 2000;		// time between hardware check

// states to track switch movement through a lifecycle
const int STATE_IDLE = 0;								// looking for work to do
const int STATE_INIT = STATE_IDLE+1;					// get ready to do work
const int STATE_CLOSEREVERSE = STATE_INIT+1;			// reverse power
const int STATE_REVERSECLOSED = STATE_CLOSEREVERSE+1;	// give reverse time to close
const int STATE_CLOSERELAY = STATE_REVERSECLOSED+1;		// close selected relay
const int STATE_RELAYCLOSEPULSE = STATE_CLOSERELAY+1;	// stay here for pulse duration
const int STATE_OPENRELAY = STATE_RELAYCLOSEPULSE+1;
const int STATE_OPENREVERSE = STATE_OPENRELAY+1;
const int STATE_ERROR = STATE_OPENREVERSE+1;			// no longer used
const int STATE_DONE = STATE_ERROR+1;

const int MAX_SWITCHMACHINES = 4;
const int MAX_TRACKSENSORS = 2;

// for news stand and signal
const unsigned long ACCESSORY_ON_TIME = 7000;		// duration of time to keep the accessory on / running
const unsigned long ACCESSORY1_MINBETWEEN = 30000;	// not faster than this
const unsigned long ACCESSORY2_MINBETWEEN = 45000;	// not faster than this

/////////////////////////////////////////////////////////////
// Runtime variables
///////////////////////////////////////////////////////////
unsigned long HeartBeatPrev = 0;        // will store last time LED was updated
unsigned long RelayCloseTime = 0;		// use this to hold time relay was closed
unsigned long LastButttonRead = 0;		// track time since we last read control panel buttons
unsigned long WatchDogStartTime = 0;	// Time that switch change state machine started to handle a request
unsigned long LastTrackSensorCheck = 0;	// time we last checked track sensors
unsigned long LastHardwareDebugCheck = 0;	// time we last walked through hardware debug loop
unsigned long accessory1_closetime = 0;	// time when this accessory was turned on
unsigned long accessory2_closetime = 0;	// time when this accessory was turned on
unsigned long accessory1_waitime = 0;	// time to wait before we turn this on again
unsigned long accessory2_waitime = 0;	// time to wait before we turn this on again

int HeartBeatState = LOW;               // flips LED on / off per heart beet change
int HandleReverse = 0;					// flag if polarity is currently reversed
int RelayStateMachine = STATE_IDLE;		// holds current of state tracking requests from control panel
int ButtonAddress = 0;					// holds address of button found pushed, while looking for panel state change

int InDebug = 0;

// called by API, once on start up
void setup() {
	pinMode(PIN_ONBOARDLED, OUTPUT);
	
	pinMode(PIN_CLKACTION, OUTPUT);
	digitalWrite(PIN_CLKACTION, LOW);	

	pinMode(PIN_SWTPWRREVERSE, OUTPUT);
	digitalWrite(PIN_SWTPWRREVERSE, LOW);	

	pinMode(PIN_ADDSEL0, OUTPUT);
	pinMode(PIN_ADDSEL1, OUTPUT);
	pinMode(PIN_ADDSEL2, OUTPUT);
	
	pinMode(PIN_CLKADDR0, OUTPUT);
	pinMode(PIN_CLKADDR1, OUTPUT);
	pinMode(PIN_CLKADDR2, OUTPUT);
	
	digitalWrite(PIN_ADDSEL0, HIGH);	
	digitalWrite(PIN_ADDSEL1, HIGH);	
	digitalWrite(PIN_ADDSEL2, HIGH);	

	digitalWrite(PIN_CLKADDR0, HIGH);	
	digitalWrite(PIN_CLKADDR1, HIGH);	
	digitalWrite(PIN_CLKADDR2, HIGH);

	// for now, until better power on reset... drive to low so signal and news stand are off by default
	WriteSignalStates(0);
	unsigned long CurrentTime = millis();
	accessory1_waitime = CurrentTime;
	accessory2_waitime = CurrentTime;

	Serial.begin(9600);
}

// called by API, as fast as possible
void loop() {
	unsigned long CurrentTime = millis();	// using this to measure duration of long-running events (heart beet, relay pulse)

	if (CurrentTime - HeartBeatPrev >= HEARTBEAT) {
		HeartBeatPrev = CurrentTime;
		HeartBeatState = !HeartBeatState;
		digitalWrite(PIN_ONBOARDLED, HeartBeatState);
    }

	// if in hardware debug, only run hardware test cases
	if( InDebug == 2 ) {
		handleHardwareDebug(CurrentTime);
		return;
	}
	
	// handle any request to change a turnout
	if( handleAnyTurnoutButtons(CurrentTime) == STATE_IDLE ) {
		// if not busy with turnout, check for other work
		handleAccessory1(CurrentTime);
		handleAccessory2(CurrentTime);
	}
}

void handleHardwareDebug(unsigned long CurrentTime) {
	if (CurrentTime - LastHardwareDebugCheck >= HARDWARE_DEBUG_HEARTBEAT) {
		LastHardwareDebugCheck = CurrentTime;
		
		Serial.print("\n\nIn hardware debug:");

		// read buttons
		int nButtons = ReadPanelButtons();

		// read switch sensors
		int nSwitches = ReadSwitchStates();

		// read signal sensors
		int nSensors = ReadTrackSensors() & 0x03;

		// drive switch LED's
		if( nButtons ) {
			SetItemSelect(AddressFromState(nButtons));
			SetClockHigh(CLK_CHANGESWITCH);
			delay(500);
			SetClockLow();
		}
		if( nSwitches ) {
			digitalWrite(PIN_SWTPWRREVERSE,HIGH);
			SetItemSelect(AddressFromState(nSwitches));
			SetClockHigh(CLK_CHANGESWITCH);
			delay(500);
			SetClockLow();
			delay(500);
			SetClockHigh(CLK_CHANGESWITCH);
			delay(500);
			SetClockLow();
			digitalWrite(PIN_SWTPWRREVERSE,LOW);
		}
		// drive sensor LED's
		if( nSensors ) {
			pinMode(PIN_DATA, OUTPUT);
			SetItemSelect(AddressFromState(nSensors));

			digitalWrite(PIN_DATA, 1);
			SetClockHigh(CLK_WRITESIGNALSTATE);
			SetClockLow();
			delay(500);
			digitalWrite(PIN_DATA, 0);
			SetClockHigh(CLK_WRITESIGNALSTATE);
			SetClockLow();
		}
	}
}

void handleAccessory1(unsigned long CurrentTime) {
	if( accessory1_closetime > 0 ) {
		if(CurrentTime - accessory1_closetime >= ACCESSORY_ON_TIME) {
			// open relay
			SetAccessoryState(0, 0);
			// remember when we turn this off, then wait before turning it on again
			accessory1_waitime = CurrentTime;
			accessory1_closetime = 0;
		}
	} else if( CurrentTime - accessory1_waitime >= ACCESSORY1_MINBETWEEN ) {
		// turn on accessory
		SetAccessoryState(0, 1);
		accessory1_closetime = CurrentTime;
	}
}

void handleAccessory2(unsigned long CurrentTime) {
	if( accessory2_closetime > 0 ) {
		if(CurrentTime - accessory2_closetime >= ACCESSORY_ON_TIME) {
			// open relay
			SetAccessoryState(1, 0);
			// remember when we turn this off, then wait before turning it on again
			accessory2_waitime = CurrentTime;
			accessory2_closetime = 0;
		}
	} else if( CurrentTime - accessory2_waitime >= ACCESSORY2_MINBETWEEN ) {
		// turn on accessory
		SetAccessoryState(1, 1);
		accessory2_closetime = CurrentTime;
	}
}

void SetAccessoryState( int nAccessory, int nState ) {
	pinMode(PIN_DATA, OUTPUT);
	digitalWrite(PIN_DATA, nState);
	SetItemSelect(nAccessory);
	SetClockHigh(CLK_WRITESIGNALSTATE);
	SetClockLow();
}

// this will get called as fast as possible, all relay movement timing controlled within this state machine
int handleAnyTurnoutButtons(unsigned long CurrentTime) {
	int Buttons = 0;

	if(InDebug) {
		if(RelayStateMachine != STATE_IDLE) {
			Serial.print("\nINFO: State machine in state ");
			Serial.print(RelayStateMachine);
		}
	}

	switch(RelayStateMachine) {
		case STATE_IDLE:
			// might want to change here, read as quick as we can for more responsiveness, but then wait for button up for time before we handle it again.
			if(CurrentTime - LastButttonRead >= HEARTBEAT_PANELCHECK) {
				LastButttonRead = CurrentTime;		
				Buttons = ReadPanelButtons();
				if(Buttons != 0) {
					ButtonAddress = AddressFromState(Buttons);
					if(ButtonAddress > -1) RelayStateMachine = STATE_INIT;
				
				// if no buttons, check to see if S1 was moved manually (want to keep S1 and S2 movement aligned to avoid derailing)
				} else {
				}
			}
			// might want to also look for turnout miss-alignment between S1 and S2, keeping them insync as they should always move together
			// would need to capture state change so we know which one to align to
			// if S1 state != S2 state, set this machine up to change the S that wasn't just modified by either manual or code. Need to know last time S1 or S2 changed by either code or manual. If manual then change other with code. If code, then we should know to do other.
			break;	

		case STATE_INIT:
			HandleReverse = ReadSwitchState(ButtonAddress);
			if(HandleReverse){
				RelayStateMachine = STATE_CLOSEREVERSE;
				if(InDebug) Serial.print("\nINFO: Switch set to turnout");
			} else {
				if(InDebug) Serial.print("\nINFO: Switch set to main");
				RelayStateMachine = STATE_CLOSERELAY;
			}
			WatchDogStartTime = CurrentTime;
			break;
			
		case STATE_CLOSEREVERSE:
			digitalWrite(PIN_SWTPWRREVERSE,HIGH);
			RelayCloseTime = CurrentTime;
			RelayStateMachine = STATE_REVERSECLOSED;
			break;
			
		case STATE_REVERSECLOSED:
			if(CurrentTime - RelayCloseTime >= RELAY_SETTLETIME)
				RelayStateMachine = STATE_CLOSERELAY;
			break;
			
		case STATE_CLOSERELAY:
			SetItemSelect(ButtonAddress);
			SetClockHigh(CLK_CHANGESWITCH);
			RelayCloseTime = CurrentTime;
			RelayStateMachine = STATE_RELAYCLOSEPULSE;
			break;
			
		case STATE_RELAYCLOSEPULSE:
			if(CurrentTime - RelayCloseTime >= RELAY_PULSETIME)
				RelayStateMachine = STATE_OPENRELAY;
			break;
			
		case STATE_OPENRELAY:
			SetClockLow();
			if(HandleReverse) {
				RelayStateMachine = STATE_OPENREVERSE;
			} else {
				// force a check for all buttons up
				LastButttonRead = CurrentTime + HEARTBEAT_PANELCHECK;		
				RelayStateMachine = STATE_DONE;
			}
			break;
			
		case STATE_OPENREVERSE:
			digitalWrite(PIN_SWTPWRREVERSE,LOW);
			// force a check for all buttons up
			LastButttonRead = CurrentTime + HEARTBEAT_PANELCHECK;		
			RelayStateMachine = STATE_DONE;
			break;
			
		case STATE_DONE:
			// all relay movement is done, kill watchdog
			WatchDogStartTime = 0;
			
			// stay here until we see no button pushed
			if(CurrentTime - LastButttonRead >= HEARTBEAT_PANELCHECK) {
				LastButttonRead = CurrentTime;	
				// check for button to be released
				if(ReadPanelButtons() == 0) {
					// no buttons pushed so now put machine in idle state, looking for next button push
					RelayStateMachine = STATE_IDLE;
				}
			}
			break;
	}
	
	// watchdog check, if overall time exceeds WATCHDOG_TIMEOUT, then force all open and trace findings
	if( WatchDogStartTime != 0 && CurrentTime - WatchDogStartTime >= WATCHDOG_TIMEOUT) {
		SetClockLow();
		digitalWrite(PIN_SWTPWRREVERSE,LOW);
		WatchDogStartTime = 0;
		RelayStateMachine = STATE_IDLE;
		Serial.print("\nERROR: Watchdog timed out");
	}

	return RelayStateMachine;
}

int ReadPanelButtons() {
	int result = 0;
	
	pinMode(PIN_DATA, INPUT);
	for(int i = 0; i < MAX_SWITCHMACHINES; i++){
		SetItemSelect(i);
		SetClockHigh(CLK_READCTRLPANEL);
		if(!digitalRead(PIN_DATA)) {
			result |= 1 << i;
		}
		SetClockLow();
	}

	if(InDebug) {
		Serial.print("\nButton States = ");
		for(int i = 0; i < MAX_SWITCHMACHINES; i++){
			int x = (result & (1 << i)) ? 1 : 0;
			Serial.print(x);
		}
	}
	
	return result;
}

int ReadSwitchStates() {
	int result = 0;
	
	pinMode(PIN_DATA, INPUT);
	for(int i = 0; i < MAX_SWITCHMACHINES; i++){
		SetItemSelect(i);
		SetClockHigh(CLK_READSWITCHSTATE);
		if(!digitalRead(PIN_DATA))
			result |= 1 << i;
		SetClockLow();
	}
	
	if(InDebug) {
		Serial.print("\nSwitch States = ");
		for(int i = 0; i < MAX_SWITCHMACHINES; i++){
			int x = (result & (1 << i)) ? 1 : 0;
			Serial.print(x);
		}
	}	
	
	return result;
}

int ReadSwitchState(int SwitchAddress) {
	int result = 0;
	pinMode(PIN_DATA, INPUT);
	SetItemSelect(SwitchAddress);
	SetClockHigh(CLK_READSWITCHSTATE);
	result = !digitalRead(PIN_DATA);
	SetClockLow();
	return result;
}

void WriteSignalStates(int States) {
	pinMode(PIN_DATA, OUTPUT);
	for(int i = 0; i < MAX_TRACKSENSORS; i++){
		digitalWrite(PIN_DATA, (States & (1 << i)) ? 1 : 0);
		SetItemSelect(i);
		SetClockHigh(CLK_WRITESIGNALSTATE);
		SetClockLow();
	}
}

int ReadTrackSensors() {
	int result = 0;
	
	pinMode(PIN_DATA, INPUT);
	for(int i = 0; i < MAX_TRACKSENSORS; i++){
		SetItemSelect(i);
		SetClockHigh(CLK_READTRACKSENSORS);
		if(!digitalRead(PIN_DATA))
			result |= 1 << i;
		SetClockLow();
	}
	
	if(InDebug) {
		Serial.print("\nSensor States = ");
		for(int i = 0; i < MAX_TRACKSENSORS; i++){
			int x = (result & (1 << i)) ? 1 : 0;
			Serial.print(x);
		}
	}	
	return result;
}

void SetItemSelect(int Address) {
	digitalWrite(PIN_ADDSEL0, (Address & 1) ? 1 : 0);
	digitalWrite(PIN_ADDSEL1, (Address & 2) ? 1 : 0);	
	digitalWrite(PIN_ADDSEL2, (Address & 4) ? 1 : 0);	
}

void SetClockHigh(int Address) {
	digitalWrite(PIN_CLKADDR0, (Address & 1) ? 1 : 0);
	digitalWrite(PIN_CLKADDR1, (Address & 2) ? 1 : 0);
	digitalWrite(PIN_CLKADDR2, (Address & 4) ? 1 : 0);
	delay(BUS_SETTLETIME);
	digitalWrite(PIN_CLKACTION, HIGH);
	delay(BUS_SETTLETIME);
}

void SetClockLow() {
	digitalWrite(PIN_CLKACTION, LOW);
	delay(BUS_SETTLETIME);
}

int AddressFromState(int States) {
	for(int i = 0; i < MAX_SWITCHMACHINES; i++){
		if(States & (1 << i)) return i;
	}
    Serial.print("\nNo AddressFromState");
	return -1;
}
