// Chimera Control System - BASE (v1.0)
// Created by : Theo Ajuyah, Jack Kent, #ADD THE REST#

// RF Code Courtesy: https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
// LCD Wiring Courtesy: https://www.arduino.cc/en/Tutorial/HelloWorld
// LoRa 9x_TX
// -*- mode: C++ -*-


// Include libraries
#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal.h>
#include <Adafruit_RGBLCDShield.h>

// Initialise LCD library with desired pins (see courtesy)
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);

//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Radio Module Pins:
#define RFM95_CS 4
#define RFM95_RST 3
#define RFM95_INT 2 // G0
#define ID "C-"
#define ARM_ACK "ARMACK"
#define FIRE_ACK "FIREACK"

// Switch & Button Pins:
const int armSwitchPin = 20;
const int fireSwitchPin = 21;
const int estopButtonPin = 29;
 

#define RF95_FREQ 868.0 // 718 lowest frequency that is working- tested
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RH_RF95_MAX_MESSAGE_LEN 300


/////Global vars
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool armStatus = 0;
bool disarmStatus = 0;
bool fireStatus = 0;
bool estopStatus = 0;
bool emergencyStopped = 0;

int armCmdCount = 0;
int fireCmdCount = 0;

/////Functions

void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);  // Setup LCD dimensions

  // Pin Setup
  pinMode(armSwitchPin, INPUT);  // RED arm swtich
  pinMode(fireSwitchPin, INPUT);  // ORANGE fire switch
  pinMode(estopButtonPin, INPUT);  // GREEN estop button

  // Attach Interrupt:
  attachInterrupt(digitalPinToInterrupt(estopButtonPin), estop, FALLING); // Estop interrupt

  // Get current pin values:
  armStatus = digitalRead(armSwitchPin);
  disarmStatus = !(digitalRead(armSwitchPin));
  fireStatus = digitalRead(fireSwitchPin);
  //estopStatus = digitalRead(estopButtonPin);

  lcd.print("CCS - HOUSTON");
  lcd.setCursor(0, 1);
  lcd.print("RADIO STARTING...");
  if(init_transmitter()) {
    delay(500);
    lcd.clear();
    lcd.print("CHIMERA CONTROL");
    lcd.setCursor(0,1);
    lcd.print("READY");
  }
}
 

void loop()
{
  armStatus = digitalRead(armSwitchPin);
  disarmStatus = !digitalRead(armSwitchPin);
  fireStatus = digitalRead(fireSwitchPin);
  //estopStatus = digitalRead(estopButtonPin);
  
  Serial.print("ARM STATUS: ");
  Serial.print(armStatus);
  Serial.print(" DISARM STATUS: ");
  Serial.print(disarmStatus);
  Serial.print(" FIRE STATUS: ");
  Serial.print(fireStatus);
  Serial.print(" ESTOP STATUS: ");
  Serial.println(estopStatus);

  lcd.setCursor(0,1);
  lcd.print("READY           ");
  
  if (estopStatus || emergencyStopped) {
    //estop();
    //emergencyStopped = 1;
  } else {
    if (fireStatus && armStatus  && (fireCmdCount <= 5)) {
      fire();
      fireStatus = 0;
    }
    if (armStatus && (armCmdCount <= 5)) {
      arm();
      armStatus = 0;
    }
    if (disarmStatus) {
      disarm();
      disarmStatus = 0;
    }
  }
  delay(1000); // Remove when the idle message sending is removed
}


bool init_transmitter() {
  bool radioReady = 1;
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  lcd.setCursor(0, 1);
   
  while (!Serial);
  
  delay(100);
 
  Serial.println("Arduino LoRa TX Test!");
 
  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    lcd.print("! RADIO FAILED !");
    radioReady = 0;
    while (1);
  }
  
  Serial.println("LoRa radio init OK!");
  lcd.print("RADIO STARTED...");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    lcd.print("! FREQ FAILED !");
    radioReady = 0;
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  return radioReady;
}


// Send Message function transmits/broadcasts
void sendMessage(String receiverID, String msg) {
//void sendMessage() {
  Serial.println("Sending to rf95_server");
  String message = receiverID + msg;
  char radiopacket[50];
  strncpy(radiopacket, message.c_str(), 50);//**** 3rd url
  
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[49] = 0;
  
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 50);
 
  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
}


void arm() {
  Serial.println("SENDING ARM COMMAND");
  lcd.setCursor(0,1);
  sendMessage("C-", "ARM");
  lcd.print("ARM CMD SENT...");
  armCmdCount++;
}


void disarm() {
  Serial.println("disarm");
}


void fire() {
  Serial.println("SENDING FIRE COMMAND");
  lcd.setCursor(0,1);
  sendMessage("C-", "FIRE");
  lcd.print("FIRE CMD SENT...");
  fireCmdCount++;
}


void estop() {
  lcd.setCursor(0,1);
  sendMessage("C-", "ESTOP");
  estopStatus = 1;
  lcd.print("!!!  ESTOP  !!!");
  Serial.println("!!  EMERGENCY STOP  !!");
}

// In use: Only called if `if (rf95.available()) {}` statement passes, so is guaranteed to have a message
String getMessage() {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      //RH_RF95::printBuffer("Received: ", buf, len); // suppresed/commented out printing binary info
      //Serial.println("Got: " + (char*)buf); // suppresed/commented out message received
      // Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC); // suppresed/commented out Received Signal Strength Indicator

      return (char*)buf;
      
      // Send a reply
      uint8_t data[] = "Message received";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
    }else {
      Serial.println("Receive failed");
      return "-1";
    }
}
