#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include <CayenneLPP.h>
#include <ESP32Time.h>
#include <math.h>
#include<ArduinoJson.h>

//ESP32Time rtc;
ESP32Time rtc(3600);  // offset in seconds GMT+1

// HELTEC LORA
const int GPIObuttonMenu = 42;
const int GPIObuttonPump1 = 45;
const int GPIObuttonPump2 = 46;
const int GPIOrelayPump1 = 4;
const int GPIOrelayPump2 = 5;
const int GPIObatVoltage = 6;
const int GPIOpvVoltage = 7;
const int GPIOrainSensor = 2;
const int GPIOhumiditySensor = 3;
const int GPIOwaterLevelTrig = 47;
const int GPIOwaterLevelEcho = 48;

const int autoReturnScreenDuration = 10000;

const int buttonShortInterval = 150;  // Temps entre 2 lectures (évite les micro-changements)
const int buttonLongInterval = 1200;  // Temps d'un appui long


// Variables
//   LOW = off
byte buttonPump1 = LOW;
byte buttonPump2 = LOW;
byte buttonMenu = LOW;
byte pump1Active = LOW;
byte pump2Active = LOW;

float batVoltage = 0;
float pvVoltage = 0;
int rain = 0;
int humidity = 0;
int waterLevel = 0;

long currentMillis = millis();  // Timestamp courant: mis à jour à chaque loop, référence pour l'ensemble du code à chaque loop

long previousButtonPump1Millis = 0;  // timestamp du dernier bouton pressé
long previousButtonPump2Millis = 0;  // timestamp du dernier bouton pressé
long previousButtonMenuMillis = 0; // timestamp du dernier bouton pressé

long pump1ActiveMillis = 0;  // timestamp cible pour timer pompe 1
long pump2ActiveMillis = 0;  // timestamp cible pour timer pompe 2

const int addTimeMillis = 2 * 60 * 1000;  // Temps ajouté à chaque appui

int lastLoraTransmissionMillis = 0; // Dernière transmission.

const int waterTankHeight = 120; // hauteur de la cuve à eau

long lastMeasure = 0;  // Dernière mesure
const int measureInterval = 5000; // interval de mesure.

int activeScreen = 0;

//-------------------------------------------------------------------------------------------------------------------
//--  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA --  LORA
//-------------------------------------------------------------------------------------------------------------------
/* OTAA para*/
uint8_t devEui[] = { 0xce, 0x75, 0x54, 0xdc, 0x00, 0x00, 0x9c, 0x5f };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
uint8_t appKey[] = { 0x60, 0xf5, 0x4e, 0xb8, 0x16, 0xbc, 0xfd, 0x0a, 0xcd, 0x08, 0x09, 0x5e, 0xe9, 0x04, 0xeb, 0xfe };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_C;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
    CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE);
    lpp.reset();
    
    lpp.addDigitalInput(1,pump1Active);
    lpp.addDigitalInput(2,pump2Active);
    
    lpp.addAnalogInput(6, getTimeRemaining(pump1ActiveMillis, true)); // durée restante en min
    lpp.addAnalogInput(7, getTimeRemaining(pump2ActiveMillis, true)); // durée restante en min

    lpp.addAnalogInput(1,pvVoltage);
    lpp.addAnalogInput(2,batVoltage);
    lpp.addAnalogInput(3,rain);
    lpp.addAnalogInput(4,humidity);
    lpp.addAnalogInput(5,waterLevel);

    lpp.getBuffer(), 
    appDataSize = lpp.getSize();
    memcpy(appData,lpp.getBuffer(),appDataSize);
}

//if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ 
RTC_DATA_ATTR bool timeReq = true;

void dev_time_updated()
{
  printf("Heure obtenue: \r\n");
  TimerSysTime_t sysTimeCurrent = TimerGetSysTime( );
  rtc.setTime((unsigned int)sysTimeCurrent.Seconds);
  Serial.println(rtc.getTime());
  Serial.println(rtc.getDate()); 
}

// Réception de data
//downlink data handle function example
void decodeDownlinkMsg( uint8_t *buf, uint8_t bufsize, uint8_t port )
{
    StaticJsonDocument<256> jsonBuffer;
    CayenneLPP lpp(0);

    JsonArray array = jsonBuffer.to<JsonArray>();
    lpp.decode( buf, bufsize, array );

    serializeJson(array, Serial);

    for ( JsonObject item : array ) {
    
      int lpp_type = item["type"];

      switch ( lpp_type )
      {
        case LPP_ANALOG_OUTPUT:
            
            if ( item["channel"] == 1 ){
            int add = item["value"];
            long addTime = add * 60 * 1000;
              pump1ActiveMillis = currentMillis + addTime;
            }

            if ( item["channel"] == 2 ){
            int add = item["value"];
            long addTime = add * 60 * 1000;
              pump2ActiveMillis = currentMillis + addTime;
            }

        break;

        default:
        break;
      } 
    }
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
	printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
	printf("+REV DATA:");
	for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
	{
		printf("%02X",mcpsIndication->Buffer[i]);
	}
	printf("\r\n");
  
  if ( mcpsIndication->RxData )
  {
    decodeDownlinkMsg( mcpsIndication->Buffer, mcpsIndication->BufferSize, mcpsIndication->Port );
  }

}

void loopLORA(){
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      if(timeReq)
      {
        timeReq = false;
        MlmeReq_t mlmeReq;  
        mlmeReq.Type = MLME_DEVICE_TIME;
        LoRaMacMlmeRequest( &mlmeReq );
        Serial.println("Demande d'obtention de l'heure de la passerelle...");
      }

      prepareTxFrame( appPort );
      LoRaWAN.send();
      lastLoraTransmissionMillis = currentMillis;
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
//--  Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display -- Display   --
//-------------------------------------------------------------------------------------------------------------------
extern SSD1306Wire display; // use display defined in LoRaWan_APP.h

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

// Variables
typedef void (*Screens)(void);
Screens screens[] = {displayScreen1, displayScreen2};
//Screens screens[] = {displayScreen1};
int screensLength = (sizeof(screens) / sizeof(Screens));

String getTimeRemainingForDisplay(long time) {
  long allSecRemaining = getTimeRemaining(time, false);

  if (allSecRemaining <= 0) {
    return "";
  } else {
    int minRemaining = allSecRemaining / 60;
    int secRemaining = allSecRemaining % 60;
    
    if(minRemaining > 0){
      return String(minRemaining) + "min " + String(secRemaining) + "s";
    }else{
      return String(secRemaining) + "s";
    }
  }
}

String getStateForDisplay(byte state){
  if(state == LOW){
    return "OFF";
  }else{
    return "ON";
  }
}

void drawHeader(){
  display.drawHorizontalLine(0, 12, 128);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  // batterie
  display.fillRect(0, 4, 8, 6);
  display.fillRect(8, 5, 2, 4);
  display.drawString(11, 0, String(batVoltage,1) + "v");
  // PV
  display.fillRect(40, 7, 6, 4);
  display.fillCircle(52, 2, 3);
  display.drawLine(46, 4, 48, 2);
  display.drawLine(47, 6, 49, 4);
  display.drawLine(48, 8, 50, 6);
  display.drawLine(50, 9, 52, 7);
  display.drawString(56, 0, String(pvVoltage,1) + "v");
  // Heure
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, rtc.getTime());
}

// Ecran 1
void displayScreen1() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  drawHeader();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawHorizontalLine(0, 44, 128);
  display.drawVerticalLine(64, 12, 32);

  // Pompe 1
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(32, 13, "Pompe 1");
  display.drawString(32, 23, getStateForDisplay(pump1Active));
  display.drawString(32, 33, getTimeRemainingForDisplay(pump1ActiveMillis));

  // Pompe 2
  display.drawString(96, 13, "Pompe 2");
  display.drawString(96, 23, getStateForDisplay(pump2Active));
  display.drawString(96, 33, getTimeRemainingForDisplay(pump2ActiveMillis));

  // Footer
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 44, "Pluie: " + String(rain) + "%");
  display.drawString(128, 54, "Sol: " + String(humidity) + "%");

  display.setTextAlignment(TEXT_ALIGN_LEFT);

  if((currentMillis - lastLoraTransmissionMillis) < (appTxDutyCycle + APP_TX_DUTYCYCLE_RND)){
    display.drawString(0, 44, "LORA: OK");
  }else{
    display.drawString(0, 44, "LORA: NOK");
  }
  
  display.drawString(0, 54, "Eau: " + String(waterLevel) + "cm");
    
  display.display();
}

struct MenuItem {
    String name;
    byte screenDest;
};

MenuItem menu[3] {
  {"Menu 1", 0},
  {"Menu 2", 1},
  {"Menu 3", 2},
};

// Ecran 2
void displayScreen2() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  
  drawHeader();

  for(int i = 0;  sizeof(menu); i++) {
    display.drawString(0, 10 + i*10, " - " + menu[i].name);
  }

  display.display();
}


void refreshDisplay() {
  // Retour automatique à l'écran de base si on a pas touché au bouton depuis 10sec
  if (currentMillis - previousButtonMenuMillis > autoReturnScreenDuration) {
    activeScreen = 0;
  }

  screens[activeScreen]();
}

//-------------------------------------------------------------------------------------------------------------------
//--  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons --  Boutons 
//-------------------------------------------------------------------------------------------------------------------

//////////////////////
// BOUTON 1
//////////////////////
void button1ShortPress() {
  Serial.println("B1 Short Press");
  addPumpTimer(pump1ActiveMillis);
}

void button1LongPress() {
  Serial.println("B1 Long Press");
  resetPumpTimer(pump1ActiveMillis);
}

void readButton1() {
  readButton(GPIObuttonPump1, buttonPump1, previousButtonPump1Millis, &button1ShortPress, &button1LongPress);
}


//////////////////////
// BOUTON 2
//////////////////////
void button2ShortPress() {
  Serial.println("B2 Short Press");
  addPumpTimer(pump2ActiveMillis);
}

void button2LongPress() {
  Serial.println("B2 Long Press");
  resetPumpTimer(pump2ActiveMillis);
}

void readButton2() {
  readButton(GPIObuttonPump2, buttonPump2, previousButtonPump2Millis, &button2ShortPress, &button2LongPress);
}

//////////////////////
// BOUTON MENU
//////////////////////
void buttonMenuShortPress() {
  Serial.println("BMenu Short Press");
  activeScreen = (activeScreen + 1) % screensLength;
}

void buttonMenuLongPress() {
  Serial.println("BMenu Long Press");
}

void readButtonMenu() {
  readButton(GPIObuttonMenu, buttonMenu, previousButtonMenuMillis, &buttonMenuShortPress, &buttonMenuLongPress);
}

//////////////////////
// BOUTON COMMUN
//////////////////////
// Lecture de l'état d'un bouton
/*
  GPIObutton: GPIO du bouton
  buttonState: dernier état du bouton
  previousButtonMillis: timestamp du dernier appui
  onShortPress: callback appui court
  onLongPress: callback appui long
*/
void readButton(int GPIObutton, byte& buttonState, long& previousButtonMillis, void (*onShortPress)(), void (*onLongPress)()) {
  // read the state of the switch/button:
  byte currentState = digitalRead(GPIObutton);

  if (currentState == HIGH && buttonState == LOW)  // button is pressed
    previousButtonMillis = currentMillis;
  else if (currentState == LOW && buttonState == HIGH) {  // button is released
    long pressDuration = currentMillis - previousButtonMillis;

    if (pressDuration > buttonShortInterval && pressDuration <= buttonLongInterval) {
      onShortPress();
    }

    if (pressDuration > buttonLongInterval) {
      onLongPress();
    }
  }

  // save the the last state
  buttonState = currentState;
}

// Ajouter du temps au timer
void addPumpTimer(long& pumpActiveMillis){
  if(pumpActiveMillis < currentMillis){
    pumpActiveMillis = currentMillis + addTimeMillis;
  }else{
    pumpActiveMillis = pumpActiveMillis + addTimeMillis;
  }
}

// Remettre à zéro (timestamp courant) le timer
void resetPumpTimer(long& pumpActiveMillis){
  pumpActiveMillis = currentMillis;
}

//-------------------------------------------------------------------------------------------------------------------
//--  Voltages --  Voltages --  Voltages --  Voltages --  Voltages --  Voltages --  Voltages --  Voltages --  Voltages 
//-------------------------------------------------------------------------------------------------------------------
void readVoltages(){
  // Pont Diviseur R1=100k, R2=10k: C = 10/(100+10) = 0,0909090909090833
  // Vréel = VMesurée / C

  batVoltage = (3.3 / 4096) * analogRead(GPIObatVoltage) / 0.0909090909090833;
  pvVoltage = (3.3 / 4096) * analogRead(GPIOpvVoltage) / 0.0909090909090833;

}

//-------------------------------------------------------------------------------------------------------------------
//--  Sensors  --  Sensors  --  Sensors  --  Sensors  --  Sensors  --  Sensors  --  Sensors  --  Sensors  --  Sensors   
//-------------------------------------------------------------------------------------------------------------------
void readWaterLevel(){
    digitalWrite(GPIOwaterLevelTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(GPIOwaterLevelTrig, LOW);

    int mesure = 0.017 * pulseIn(GPIOwaterLevelEcho, HIGH);

    waterLevel = max(0, waterTankHeight - mesure); // hauteur d'eau = hauteur de la cuve - hauteur de "vide" mesuré. Max 0 si on mesure + que la hauteur de la cuve, on renvoie 0...
  }

void readHumiditySensor(){
  humidity = (100 - (100 * analogRead(GPIOhumiditySensor) / (4095))) / 1;  // / 0.7 car max 70% quand capteur immergé
}

void readrainSensor(){
  rain = (100 - (100 * analogRead(GPIOrainSensor) / (4095))) / 0.7;  // / 0.7 car max 70% quand capteur immergé
}


//////////////////////
// ETAT DES POMPES
//////////////////////
// Temps restant en seconde, ou en minute
long getTimeRemaining(long time, bool inMin){
 long secRemaining = (time - currentMillis)/1000;

 if(secRemaining < 0){
  return 0;
 }else{
  if(inMin){
    int minRemaining = secRemaining / 60;
    return minRemaining;
  }else{
    return secRemaining;
  }
 }
}

void checkPumpStates(){
  if(getTimeRemaining(pump1ActiveMillis, false) > 0){
    pump1Active = HIGH;
  }else{
    pump1Active = LOW;
  }

  if(getTimeRemaining(pump2ActiveMillis, false) > 0){
    pump2Active = HIGH;
  }else{
    pump2Active = LOW;
  }

  // checkTension
  
  digitalWrite(GPIOrelayPump1, !pump1Active); // ! --> Relais Low Level Trigger
  digitalWrite(GPIOrelayPump2, !pump2Active); // ! --> Relais Low Level Trigger
}

void setup() {
  // initialize OLED
  VextON();
  delay(100);
  display.init();

  Serial.begin(115200);

  pinMode(GPIObuttonMenu, INPUT_PULLUP);

  pinMode(GPIObuttonPump1, INPUT_PULLUP);
  pinMode(GPIObuttonPump2, INPUT_PULLUP);

  pinMode(GPIOrelayPump1, OUTPUT);
  pinMode(GPIOrelayPump2, OUTPUT);

  pinMode(GPIObatVoltage, INPUT);
  pinMode(GPIOpvVoltage, INPUT);
  pinMode(GPIOrainSensor, INPUT);
  pinMode(GPIOhumiditySensor, INPUT);
  
  pinMode(GPIOwaterLevelTrig, OUTPUT);
  pinMode(GPIOwaterLevelEcho, INPUT);

  Mcu.begin();
  deviceState = DEVICE_STATE_INIT;
}

void loop()
{
  // Capture le temps courant
  currentMillis = millis();

  // Lecture des boutons
  readButton1();
  readButton2();
  readButtonMenu();

  // Lecture des sensors
  if(currentMillis > lastMeasure + measureInterval){
    readVoltages();
    readWaterLevel();
    readHumiditySensor();
    readrainSensor();
    lastMeasure = currentMillis;
  }

  // Actualisation état calculé des pompes
  checkPumpStates();

  // Rafraichissement de l'écran
  refreshDisplay();

  // Communication LORA:
  loopLORA();
}