

#include <Servo.h>
#include <IRLibSendBase.h>    
#include <IRLib_P01_NEC.h>    
#include <IRLibCombo.h>   
#include <Keypad.h>




IRsend irSender;

//future implementation of IR recieve to extend remote range
//IRrecvPCI irReceiver(3); 
//IRdecode irDecoder;   



//*** Blast Gate/Servo Globals ***// 

const int NUMBER_OF_GATES = 8;
const int NUMBER_OF_TOOLS = 6;

String tools[NUMBER_OF_TOOLS] = {"JOINTER", "PLANER", "MITERSAW", "BANDSAW", "TABLESAW", "ROUTER"}; 
String gateLabels[NUMBER_OF_GATES] = {"Jointer", "Planer", "Miter", "Band Saw", "Table Saw", "Router", "South Y", "North Y"};


//Gate pins
const int jointerPin = 8;
const int planerPin = 10;
const int miterPin = 6;
const int bandsawPin = 13;
const int tablesawPin = 5;
const int routerPin = 11;
const int northYPin = 12;
const int southYPin = 7;

int blastGatePins[NUMBER_OF_GATES] = {jointerPin, planerPin, miterPin, bandsawPin, tablesawPin, routerPin, northYPin, southYPin}; // {"8-Jointer", "10-Planer", "6-Miter", "13-Band Saw", "5-Table Saw", "11-Router", "12-South Y", "7-North Y"}


//create servo array
Servo blastGates[NUMBER_OF_GATES];

// Enabling voltage regulator for servos
const int blastGateEnablePin = 4;
bool blastGatesEnabled = false; 

// throw limits for each gate (open/closed)
int gateMinMax[NUMBER_OF_GATES][2] = {
  // {SERVOMIN, SERVOMAX}
  {45, 90}, // Jointer
  {45, 90}, // Planer
  {45, 90}, // Miter Saw
  {45, 90}, // Band Saw
  {45, 90}, // Table saw
  {45, 90}, // Table saw
  {45, 90}, // South Y
  {45, 90}, // North Y
  
};

int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
  // {jointer, planer, miter, bandsaw, tablesaw, router, south y, north y}
  {1,0,0,0,0,0,1,0}, // jointer
  {0,1,0,0,0,0,1,0}, // planer
  {0,0,1,0,0,0,1,0}, // miter
  {0,0,0,1,0,0,0,1}, // bandsaw
  {0,0,0,0,1,0,0,0}, // table
  {0,0,0,0,0,1,0,1}, // router
};

//*** AC Globals ***//
const int NUMBER_OF_SENSORS = 6;
int voltSensor[NUMBER_OF_SENSORS] = {A1,A2,A5,A3,A4,A0}; // jointer(A0), planer(A3), miter(A1), bandsaw(A2), tablesaw(A4), router(A5)
String ampLabels[NUMBER_OF_SENSORS] = {"jointer", "planer", "miter", "bandsaw", "tablesaw", "router"};
double ampBaselines[NUMBER_OF_SENSORS] = {0, 0, 0, 0, 0};
double ampThresholds[NUMBER_OF_SENSORS] = {7, 7, 6, 6, 7, 6};
int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module
double standardThreshold = 4.5;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
int voltageSampleTime = 100;
int activeTool = 50;// 50 is default for no tool active

//*** Manual Button Globals ***//


//button pin identifies
const int NUMBER_OF_BUTTONS = 6;
const int tablesawButton = 14;
const int mitersawButton = 15;
const int dcButton = 16;
const int routerButton = 17;
const int enableButton = 20;
const int gateResetButton = 19;

int buttons[NUMBER_OF_BUTTONS] = {tablesawButton, mitersawButton, dcButton, routerButton, enableButton, gateResetButton}; 


//*** DC Globals ***//
int DC_spindown = 2000;
int DC_spinup = 1000; //used if amp draw is too much on breaker

bool collectorIsOn = false;



//*** Program Control ***//
bool DEBUG = true;
bool serialActive = false;

bool initiated = false; //run at startup to close gates and allow AC readings to stabilize
bool serialControlMode = false;


int idleCount = 0;


const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3', 'A'},
  {'4','5','6', 'B'},
  {'7','8','9', 'C'},
  {'*','0','#', 'D'}
};

byte rowPins[ROWS] = {36, 38, 40, 42}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {44, 46, 48, 50}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );  


bool programActive = true;


void setup(){ 
  Serial.begin(9600);
  
  //enable pin setup
  pinMode(blastGateEnablePin, OUTPUT);
  digitalWrite(blastGateEnablePin, LOW);
  
  //button setup
  for(int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    pinMode(buttons[i], INPUT);
  }

  //attach at set blast gate (servo) posiotions
  for(int j = 0; j < NUMBER_OF_GATES; j++) {
    blastGates[j].write(70); //set a neutral position to avoid a large first jump
    blastGates[j].attach(blastGatePins[j]);
  }
  

  enableBlastGates();
  delay(1500);
  disableBlastGates();
  
}

void loop(){
  // initiate program by getting current sensor values and setting gates to middle position
  // while(1) {
  //   keypadControl();
  //   Serial.println("RESTART");
  //   delay(1000);
  // }
  if(!initiated) {
    // enableBlastGates();
    // setGatesToNeutral();
    // delay(1000);
    setAmpBaselines();
    initiated = true;
    // delay(1000);
  }

  // check if servos are disabled when they shouldn't be
  // if(!digitalRead(blastGateEnablePin) && !blastGatesEnabled) {
  //   enableBlastGates();
  //   Serial.println("ENABLED BLAST GATES FROM LOOP CHECK");
  // }

  // if serial control mode is active, loop until disabled
  while(serialControlMode) {
    serialControl();
  }
 

  watcherFunction();
  
   //loop through tools and check
  activeTool = 99;// a number that will never happen
  readToolAmps();
  
	idleCount++;
	if(idleCount > 100) {
		disableBlastGates();
	}
	if(collectorIsOn) {
		idleCount = 0;
	}
	
}


void watcherFunction() {
	keypadControl();
	checkButtons();
}

void readToolAmps() {
  activeTool = 99;
  //add functionality if two tools are turned on
  for(int i = 0; i < NUMBER_OF_SENSORS; i++){
  	watcherFunction();
	if(checkForAmperageChange(i)){
		activeTool = i;
		exit;
	}
  }

  // activate tool and set gate positions if a current change is detected
  if(activeTool != 99){
 
    if(collectorIsOn == false){
      Serial.println(tools[activeTool] + " activated (readToolAmps)");
      activateTool(activeTool);
    }
  } 
  else {
      if(collectorIsOn == true){
        Serial.println("DEACTIVATE SYSTEM (readToolAmps)");
        delay(DC_spindown);
        deactivateTool();
    }
  }
}

void activateTool(int tool) {
	if(blastGatesEnabled == false) {
		enableBlastGates();
	}
  if(tool > NUMBER_OF_TOOLS) {
    Serial.println("INVALID TOOL TO ACTIVATE (activateTool)");
  }
  for(int i = 0; i < NUMBER_OF_GATES; i++){
    int pos = gates[tool][i];
    if(pos == 1){
      openGate(i);    
    } else {
      closeGate(i);
    }
  }
      
  turnOnDustCollection();
}


void deactivateTool() {
  turnOffDustCollection();
  closeAll();
}


void turnOnDustCollection(){
idleCount = 0;
  if(DEBUG) {
    printLine("DC ON");
  }
  for(int i = 0; i<5; i++) {
  	irSender.send(NEC,0x10EFC03F,32);
  	delay(50);
  }
  collectorIsOn = true;
}
void turnOffDustCollection(){
	idleCount = 0;
  if(DEBUG) {
    printLine("DC OFF");
  }
  for(int i = 0; i<5; i++) {
  	irSender.send(NEC,0x10EFE01F,32);
  	delay(50);
  }
  
  collectorIsOn = false;
  // closeAll();
}








void checkButtons() {
  int activeButton = 99; //a button that will never happen
  for(int i=0; i<NUMBER_OF_BUTTONS; i++) {
  	
    if(digitalRead(buttons[i])) {
      activeButton = buttons[i];
      
    }
  }

  // button array {tablesawButton, mitersawButton, dcButton, routerButton, enableButton, gateResetButton}; 
  int toolIndex = 99;
  if(activeButton != 99) {
  	switch (activeButton) {
      case tablesawButton:
        toolIndex = findToolIndex("TABLESAW");
        Serial.println("TABLESAW BUTTON");
        activateTool(toolIndex);
        while(digitalRead(activeButton)) {}
        turnOffDustCollection();
        break;

      case mitersawButton:
        toolIndex = findToolIndex("MITERSAW");
        Serial.println("MITERSAW BUTTON");
        activateTool(toolIndex);
        while(digitalRead(activeButton)) {}
        turnOffDustCollection();
        break;

      case routerButton:
        toolIndex = findToolIndex("ROUTER");
        Serial.println("ROUTER BUTTON");
        activateTool(toolIndex);
        while(digitalRead(activeButton)) {}
        turnOffDustCollection();
        break;

      case dcButton:
      	Serial.println("DC BUTTON");
        turnOnDustCollection();
        while(digitalRead(activeButton)) {}
        turnOffDustCollection();
        break;
      case gateResetButton:
        Serial.println("RESETING GATES (checkButtons)");
        for(int j = 0; j < NUMBER_OF_GATES; j++) {
          //(index, position)
          setGatePosition(j, 60);
          delay(200);
          setGatePosition(j, 80);
          delay(200);
          setGatePosition(j, 70);
          delay(200);
        }
        break;
      default:
        // Serial.println("INVALID ACTIVE BUTTON (checkButtons)");
        break;
        
        
    }
  }
  
  
}

//String gateLabels[NUMBER_OF_GATES] = {"Jointer", "Planer", "Miter", "Band Saw", "Table Saw", "Router", "South Y", "North Y"};

void testGates() {
	int currentGate = 0;
  char key = keypad.getKey();

  while(key != '*') {
    key = keypad.getKey();
    if(key) {

      switch (key) {
        case '1':
	        currentGate = 0;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '2':
          currentGate = 1;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '3':
          currentGate = 2;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '4':
        	currentGate = 3;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;
        case '5':
        	currentGate = 4;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '6':
          	currentGate = 5;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '7':
          	currentGate = 6;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case '8':
          	currentGate = 7;
	        openGate(currentGate);
	        delay(1000);
	        closeGate(currentGate);
          break;

        case 'A':
     		enableBlastGates();
          break;

        case 'B':
        	disableBlastGates();
          break;
        case 'C':
        	closeGate(currentGate);
        	break;
        case 'D':
        	
        	break;
        case '#':
        	
        	break;

        case '*':
          Serial.println("EXIT");
          break;

        default:
          Serial.print("INVALID KEY: ");
          Serial.println(key);
        
      }
    }
  }
}


void keypadControl() {
  
  int toolIndex = 99;
  char key = keypad.getKey();

  // while(key != '*' || programActive) {
  //   key = keypad.getKey();
    if(key) {

      switch (key) {
        case '1':
          toolIndex = findToolIndex("TABLESAW");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();
          break;

        case '2':
          toolIndex = findToolIndex("PLANER");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();
          break;

        case '3':
          toolIndex = findToolIndex("JOINTER");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();
          break;

        case '4':
          toolIndex = findToolIndex("MITERSAW");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();

          break;
        case '5':
          toolIndex = findToolIndex("ROUTER");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();
          break;

        case '6':
          toolIndex = findToolIndex("BANDSAW");
          Serial.print("TURNING ON: ");
          Serial.println(toolIndex);
          activateTool(toolIndex);
          turnOnDustCollection();
          waitForKey();
          turnOffDustCollection();
          break;

        case '7':
          
          
          break;

        case '8':
        delay(100);
          testGates();
          
          break;

        case 'A':
          Serial.println("DC ON");
          turnOnDustCollection();
          break;

        case 'B':
          Serial.println("DC OFF");
          turnOffDustCollection();
          break;
        case 'C':
        	setAmpBaselines();
        case 'D':
        	enableBlastGates();
        	break;
        case '#':
        	disableBlastGates();
        	break;

        case '*':
          Serial.println("EXIT");
          break;

        default:
          Serial.print("INVALID KEY: ");
          Serial.println(key);
        
      }
    }
  // }
}


void waitForKey() {
	char key = keypad.getKey();

	while(key != '*') {
		key = keypad.getKey();
	}
	Serial.println("EXIT waitforKey");
	turnOffDustCollection();
}


  
  



void closeGate(uint8_t gateIndex){
  blastGates[gateIndex].write(gateMinMax[gateIndex][0]);
}

void openGate(uint8_t gateIndex){
  blastGates[gateIndex].write(gateMinMax[gateIndex][1]);
}

void closeAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    blastGates[i].write(gateMinMax[i][0]);
    delay(100);
  }
}

void openAll() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    blastGates[i].write(gateMinMax[i][1]);
    delay(100);
  }
}

void setGatesToNeutral() {
  for(int i=0; i<NUMBER_OF_GATES; i++) {
    blastGates[i].write(70);
    delay(100);
  }
}

void setGatePosition(int gateIndex, int pos) {
  blastGates[gateIndex].write(pos);
}

void disableBlastGates() {
  digitalWrite(blastGateEnablePin, LOW);
  blastGatesEnabled = false;
  Serial.println("BLAST GATES DISABLED (disableBlastGates)");
}

void enableBlastGates() {
  digitalWrite(blastGateEnablePin, HIGH);
  blastGatesEnabled = true;
  Serial.println("BLAST GATES ENABLED (enableBlastGates)");
}

void setAmpBaselines() {
  Serial.println("---- AMP BASELINES ----");
  for(int i = 0; i < NUMBER_OF_SENSORS; i++) {
  	float avgerageReading = 0;
  	for(int j = 0; j < 10; j++) {
  		Voltage = getVPP(voltSensor[i]);
    	VRMS = (Voltage/2.0) *0.707; 
    	avgerageReading += (VRMS * 1000)/mVperAmp;
  	}
    AmpsRMS = avgerageReading / 10;
    ampBaselines[i] = AmpsRMS;
    Serial.print(AmpsRMS);
    Serial.println(" - " + ampLabels[i]);
  }

}

bool checkForAmperageChange(int sensor) {
  Voltage = getVPP(voltSensor[sensor]);

  //convert sensor RMSvoltage to ampos
  VRMS = (Voltage/2.0) *0.707; 
  AmpsRMS = (VRMS * 1000)/mVperAmp;

  //returns true if amperage exceeded set threshold for tool  
  if(AmpsRMS>ampThresholds[sensor]) {
    Serial.print(tools[sensor]+": ");
    Serial.print(AmpsRMS);
    Serial.println(" Amps RMS");
    return true;
  }

  //return false if current amerage falls below threshold
  else {
    return false; 
  }
}

float getVPP(int sensor) {
  float result;
  
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < voltageSampleTime) //sample for 250 mSec
   {
       readValue = analogRead(sensor);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
 }

int findToolIndex(String tool) {
  for(int i = 0; i<NUMBER_OF_TOOLS; i++) {
    if(tool == tools[i]) {
      return i;
    }
  }
  Serial.println("TOOL NOT FOUND IN ARRAY");
  return 99;
}

void serialControl() {
  printLine(" 0) Jointer , 1) Planer , 2) Miter , 3) Band Saw , 4) Table Saw, 5) SOUTH ON, 6) NORTH ON, 10) EXIT");
  activeTool = getSerial();
  if(activeTool > NUMBER_OF_GATES) {
    serialControlMode = false;
    Serial.println("SERIAL CONTROL DISABLED (serialControl)");
  }
    
  else {
    activateTool(activeTool); 
    Serial.println("ENTER AND CHARACTER TO DISABLE");
    getSerial(); // haults program until serial data is recieved
    deactivateTool();
  }
}


long getSerial() {
  unsigned long serialdata;
  int inbyte;
  long data;
  serialdata = 0;
  //continue until a newline is recieved
  while (inbyte != '\n')
  {
    inbyte = Serial.read();  

    if (inbyte > 0 && inbyte != '\n') 
    { 
      serialdata = serialdata * 10 + inbyte - '0';

    }
  }
  inbyte = 0;
  return serialdata;
  
}

void printString(String message) {
  if(serialActive) {
    Serial.print(message);
  }
}


void printLine(String message) {
  if(serialActive) {
    Serial.println(message);
  }
}

void printNum(double num) {
  if(serialActive) {
    Serial.print(num);
  }
}

void printNumLine(double num) {
  if(serialActive) {
    Serial.println(num);
  }
}

