#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_MAX31855.h"

#define RFM95_CS 5
#define RFM95_RST 4 
#define RFM95_INT 3
#define RF95_FREQ 868.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define RH_RF95_MAX_MESSAGE_LEN 300

#define solenoid 14
#define igniter 15
const int stepPin = 16;   //pin to pulse for steps
const int dirPin = 17;    //pin to change step direction
#define StartPosition 18 //the current code does not use interrupts, butten is high when steppermotor is in starting position
#define temp thermocouple.readCelsius()
#define MAXDO   8
#define MAXCS   7
#define MAXCLK  6
// This tranceivers id
#define ID "C-"
//Global Variables
int16_t packetnum = 0;
unsigned short interrupt = 0;

// Command codes
#define ARM "ARM"
#define FIRE "FIRE"
#define ESTOP "ESTOP"

bool acking = false;
bool igniterrun = false;
bool EmergencyStop = false;

enum State{Idle, PreBurnWait, MainBurnWait}state;
#define durationMillis_preBurn (10 * 1000)
#define durationMillis_mainBurn (17 * 1000)
unsigned long startTimeMillis;

Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  
  pinMode(solenoid, OUTPUT);
  pinMode(igniter, OUTPUT);
  digitalWrite(solenoid, HIGH);
  digitalWrite(igniter, HIGH);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(StartPosition, INPUT);
  go_to_start();
  Serial.begin(9600);
  Serial.println("Hello");
  init_transceiver();
}

void loop() 
{
  acking = false;
  //solenoidFun();
  String msg = "";
  //Serial.println(thermocouple.readCelsius());
  if (rf95.available()) {
    msg = getMessage();
    if(msg.substring(0,sizeof(ID)-1) == ID) { // If starts with id e.g. .substring(0,2) == "C-" checks if 1st 2 letters match
      if(msg.substring(sizeof(ID)-1,2 + sizeof(ARM)-1) == ARM) {
        // Do this
        solenoidFun();
        //sendMessage("C-", "ARMACK");
        acking = true;
        //
      } else {
        Serial.println("\""  ARM  "\" check failed");
        Serial.println("Got this instead: " + msg);
        if(msg.substring(sizeof(ID)-1,2 + sizeof(FIRE)-1) == FIRE) {
          Serial.println("\""  FIRE  "\" was sent to \""  ID  "\"(that's me!)");
          
          if(igniterrun == false){
            //igniterFun();
            igniterFun_part1();
            startTimeMillis = millis(); // set start time for 4s wait reference
            state = PreBurnWait;
            Serial.println("     About to go to PreBurnWait");
          }
          igniterrun = true;
        } else {
          Serial.println("\""  FIRE  "\" check failed");
          Serial.println("Got this instead: " + msg);
        } 
         if(msg.substring(sizeof(ID)-1,2 + sizeof(ESTOP)-1) == ESTOP) {
          Serial.println("\""  ESTOP  "\" was sent to \""  ID  "\"(that's me!)");
          //eFun(); // called when a wait state is broken out of below based on EmergencyStop flag
          EmergencyStop = true;
        } else {
          Serial.println("\""  ESTOP  "\" check failed");
          Serial.println("Got this instead: " + msg);
        } 
      }
    } else {
      Serial.println("\""  ID  "\" check failed");
      Serial.println("Got this instead: " + msg);
    }

//    if (acking) {
//      //sendMessage("C-", "ARMACK");
//    }else {
//      sendMessage("C-", "T-" + String(thermocouple.readCelsius()));
//    }
    sendMessage("C-", "T-" + String(thermocouple.readCelsius()));
  }

  switch(state) {
    case Idle:
      // statements
      //delay(2000);
      break;

    case PreBurnWait:
      // statements
      if (millis() - startTimeMillis < durationMillis_preBurn) { // if we haven't elapsed the desired duration...
        // if e-stop... do what u need to do, then big delay to 'stop' (more elegant solution later)
        if(EmergencyStop) { //{delay(60 *60 * 1000);} // 1hr
          eFun();
          state = Idle;
          delay(60 *60 * 1000); // leave this if e-stop is still being sent continuously
        }
      }else {
        // change state
        igniterFun_part2();
        startTimeMillis = millis(); // set start time for 17s wait reference
        state = MainBurnWait;
        Serial.println("     About to leave PreBurnWait to go to MainBurnWait");
      }
      break;

    case MainBurnWait:
      // statements
      if (millis() - startTimeMillis < durationMillis_mainBurn) { // if we haven't elapsed the desired duration...
        // if e-stop... do what u need to do, then big delay to 'stop' (more elegant solution later)
        if(EmergencyStop) {// {delay(60 *60 * 1000);} // 1hr
          eFun();
          state = Idle;
          delay(60 *60 * 1000); // leave this if e-stop is still being sent continuously
        }
      }else {
        // change state
        igniterFun_part3(); // no more waits after this
        state = Idle;
        Serial.println("     About to leave MainBurnWait");
      }
      break;

    default:
      // defult statements
      ;
  }
}

void solenoidFun() {
  Serial.println("fun times - Got ARM");
  digitalWrite(solenoid, LOW);
  //delay(2000);
}
void igniterFun() {
  Serial.println("Entered igniter fun");
  turn_right_degrees(45);
  delay(4000);   
  digitalWrite(igniter, LOW);
  turn_right_degrees(45);
  Serial.println("About to delay!");
  
  delay(3000);
  Serial.println("Done delay");
  turn_right_degrees(-80);  //force close in case of an error
  go_to_start();
  Serial.println("fun flames");
 
  
  delay(2000);
  Serial.println("Left Igniter Fun");
}

// ========= ========= ========= ========= ========= ========= ========= ========= ========= =========
void igniterFun_part1() {
  Serial.println("Entered igniter fun");
  digitalWrite(igniter, LOW);
  turn_right_degrees(45);
  
  // to wait 4s outside
}

void igniterFun_part2() {
  // after waiting 4s b4 here
  
  
  turn_right_degrees(45);
  Serial.println("About to delay!");
  // to wait 17s outside
}

void igniterFun_part3() {
  // after waiting 17s outside
  
  Serial.println("Done delay");
  turn_right_degrees(-80);  //force close in case of an error
  go_to_start();
  Serial.println("fun flames");
  Serial.println("Left Igniter Fun part 3");
}

// ========= ========= ========= ========= ========= ========= ========= ========= ========= =========

void eFun(){
  Serial.println("Fun stop");
  digitalWrite(solenoid, HIGH);
  digitalWrite(igniter, HIGH);
  go_to_start();
}

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






// __________________________________________________| = New functions = |__________________________________________________
void init_transceiver() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  delay(100);
 
  Serial.println("Tranceiver Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
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
// __________________________________________________| = Helper functions = |__________________________________________________

// Go left to starting position
void go_to_start() {
  for(int i = 0; (i< 360) & (digitalRead(StartPosition) == 0); i++) turn_right_degrees(-1);  //go till button closes or a full rotation if button fails
}


/* Fire procecure:
 *  open 45 degrees
 *  wait t1
 *  fully open
 *  wait 17s
 *  fully close
 */
void fire_procedure() {
  turn_right_degrees(45);
  delay(4000);
  turn_right_degrees(45);
  delay(17000);
  turn_right_degrees(-80);  //force close in case of an error
  go_to_start();
}

// Change the stepper direction to forward
void stepperFWD() {
  digitalWrite(dirPin, HIGH);
}

// Change the stepper direction to reverse
void stepperREV() {
  digitalWrite(dirPin, LOW);

}

// Have the stepper motor take one step
void motorStep() {
  digitalWrite(stepPin, HIGH);
  delay(30); // Don't go below 15 for the big one! With the 
  digitalWrite(stepPin, LOW);
}



// Turn right by approximately the angle specified
// Note: Unlike servo motor, can't tell if moved by external force!
float turn_right_degrees(float angle) {
  if(angle > 0) // if +ve angle
    stepperFWD();
  else {
    stepperREV();
    angle*=-1;// make +ve for rest of calculation
  }
  
  int steps = angle / 1.8; // truncated # of steps
  float err = angle - steps*1.8; // error(+ve) for under-shot
  if(err > 1.8/2) {
    steps++;
    err = err - 1.8; // error(-ve) for over-shot
  }
  err *= -1; // negates err so +ve means over-shot & vice versa
  err = (err == 0) ? 0 : err; // Just makes it display 0 & not -0
  
  for(int i = 0; i < steps; i++) {
    
    motorStep();
    //delay(1);
  }
  
  return err;
}


void turn_right_degrees_90() {
  turn_right_degrees(90);
  Serial.println("Turning right...");
}
