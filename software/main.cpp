
#include <RFM12B.h>
#include <SPI.h>
#include "bma2XX_regs.h"

#define NODEID        42   //network ID used for this unit
#define NETWORKID     137  //the network ID we are on
#define GATEWAYID     1    //the node ID we're sending to

#define LED       //disbable modules by uncommenting
#define RADIO
#define BUZZER
#define ACC


#define START               digitalWrite(CS_BMA, 0)
#define STOP                digitalWrite(CS_BMA, 1)
#define READ                0x80

#define LED_G P2_4 
#define LED_R P2_2
#define LED_B P2_1

#define BUZZ P2_5

#define BUTTON P2_3

#define CS_RFM12 P3_0
#define CS_BMA   P3_1

#define ADC P1_4

/* VBAT = ADC * 0,00244V
 * 614  = 1,5V
 * 574  = 1,4V
 * 533  = 1,3V
 * 492  = 1,2V
 */


#define ACK_TIME     1500  // # of ms to wait for an ack
#define SERIAL_BAUD  9600  // serial debug baud rate
#define requestACK 0       //request ack 

RFM12B radio;
byte sendSize = 0;
char payload[28] = ""; // max. 127 bytes


byte data[3];
boolean buttonInterrupt = false, wireInterrupt = false, accInterrupt = false;

uint16_t batVoltage = 0;

  
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED_R, OUTPUT);    
  pinMode(LED_G, OUTPUT); 
  pinMode(LED_B, OUTPUT);  
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  pinMode(BUZZ, OUTPUT);  
  digitalWrite(BUZZ, LOW);

  pinMode(ADC, INPUT); 
  analogReference(INTERNAL2V5);

  pinMode(CS_RFM12, OUTPUT);  
  digitalWrite(CS_RFM12, HIGH);

  pinMode(CS_BMA, OUTPUT);
  digitalWrite(CS_BMA, HIGH);

  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(BUTTON, buttonFunction, FALLING);  //interrupt for button
  
  Serial.begin(SERIAL_BAUD);
  Serial.print("start...");

  SPI.begin();
  SPI.setClockDivider(2); //8MHz SPI clock

  #ifdef RADIO
  radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);
  radio.Sleep(); //sleep right away to save power
  #endif

  #ifdef ACC
  bma2XXclearInterrupts(); //clear existing interrupts
  delay(100);
  bma2XXsetProfile(); //initialize Accelerometer
  attachInterrupt(P1_3, accFunction, FALLING); //interrupt for BMA280
  #endif

  suspend(); //sleep, wait for interrupts
}

void loop() {
  if (buttonInterrupt) {
    buttonInterrupt = false;
    for (byte i = 0; i < 255; i++) { //wait ~5s
      if (digitalRead(BUTTON) != LOW) {
        return;
      }
      delay(20);
    }

    for (byte i = 0; i < 2; i++) { //indicator flashing
      digitalWrite(LED_B, HIGH);
      delay(100);
      digitalWrite(LED_B, LOW);
      delay(100);
    }
    
    delay(1000);
    
    if (digitalRead(BUTTON) != LOW) { //button still pressed?
      deepSleep(); //...then go to deep sleep
      
      #ifdef ACC   //This is the point we wake up again later
      bma2XXclearInterrupts(); 
      delay(100);
      bma2XXsetProfile();
      #endif
      digitalWrite(LED_G, HIGH);
      delay(200);
      digitalWrite(LED_G, LOW);
      #ifdef BUZZER
      startMelody(); //Windows XP boot melody ;)
      #endif
      return;
    }

    delay(1000); //not pressed?
    selfTest();  //then do a self test
  }

  if (wireInterrupt) {
    wireInterrupt = false;   
    sendPackage(2);
    fail();
    #ifdef LED
     for(int j=0; j<256; j++) {
      Wheel((j) & 255, data);
      analogWrite(LED_R, data[0]);
      analogWrite(LED_G, data[1]);
      analogWrite(LED_B, data[2]);
    delay(3);
     }
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    #endif

    //TODO
  }

  if (accInterrupt) {
    accInterrupt = false;
    char intType = readBMA2XX(BMAREG_INTSTAT0); //Read the Interrupt Reason
    if (bitRead(intType, INTSTAT0_FLATINT)) {   //Oriantation changed
      sendPackage(11);
      fail();
    }
    else if (bitRead(intType, INTSTAT0_SLOPEINT)) { //Fast motion Interrupt
      sendPackage(12);
      fail();
    }
    else if (bitRead(intType, INTSTAT0_SLO_NO_MOT_INT)) { //Slow Motion interrupt
      sendPackage(13);
      //fail();
    }
  }

  interrupts();//reenable interrupts
  suspend();   //go back to sleep
}

void selfTest() {
  uint16_t batVoltage = readBat();

  #ifdef LED
  analogWrite(LED_R, constrain(map(batVoltage, 490, 615, 255, 0), 0, 255)); //display the Battery voltage (1,1V - 1,5V ^= Red - Green)
  analogWrite(LED_G, constrain(map(batVoltage, 490, 615, 0, 255), 0, 255));
  delay(1000);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
    
  for(int j=0; j<512; j++) { //test LED's
    if (j % 512 < 256)
      analogWrite(LED_R, j % 256);
    else
      analogWrite(LED_R, 256 - j % 256);
  delay(1);
  }
  digitalWrite(LED_R, LOW);
  for(int j=0; j<512; j++) {
    if (j % 512 < 256)
      analogWrite(LED_G, j % 256);
    else
      analogWrite(LED_G, 256 - j % 256);
  delay(1);
  digitalWrite(LED_G, LOW);
  }
  for(int j=0; j<512; j++) {
    if (j % 512 < 256)
      analogWrite(LED_B, j % 256);
    else
      analogWrite(LED_B, 256 - j % 256);
  delay(1);
  }
  digitalWrite(LED_B, LOW);
  delay(500);
  #endif

  #ifdef BUZZER //test Buzzer
  tone(BUZZ, 2200);
  delay(100);
  tone(BUZZ, 2700);
  delay(100);
  tone(BUZZ, 3200);
  delay(100);
  noTone(BUZZ);
  delay(500);
  #endif
  
  delay(500);

  sendPackage(1); //test Radio 
  
  digitalWrite(LED_G, HIGH);
  delay(200);
  digitalWrite(LED_G, LOW);
}

void deepSleep() {
  #ifdef BUZZER
  shutdownMelody(); //Windows XP shutdown melody ;)
  #endif
  #ifdef RADIO
  radio.Sleep();
  #endif
  #ifdef ACC
  writeBMA2XX(BMAREG_SLEEP_DURATION, 0b10 << BMA_LOWPOWER_ENA); //go to suspend mode (~2µA)
  #endif
  interrupts(); //enable interrupts to capture button press
  suspend();    //low power mode
}


void Wheel(byte WheelPos, byte pdata[]) { //HSV color table thingy
  if(WheelPos < 85) {
    pdata[0] = WheelPos * 3;
    pdata[1] = 255 - WheelPos * 3;
    pdata[2] = 0;
   return;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
    pdata[0] = 255 - WheelPos * 3;
    pdata[1] = 0;
    pdata[2] = WheelPos * 3;
   return;
  } else {
   WheelPos -= 170;
   pdata[0] = 0;
   pdata[1] = WheelPos * 3;
   pdata[2] = 255 - WheelPos * 3;
   return;
  }
}

void fail() { //fail sound...
  for (byte i = 0; i < 5; i++) {
    digitalWrite(LED_R, HIGH);
    #ifdef BUZZER
    tone(BUZZ, 3200);
    #endif
    delay(100);
    digitalWrite(LED_R, LOW);
    noTone(BUZZ);
    delay(100);
  } 
}


void buttonFunction() //ISR
{
  wakeup();
  noInterrupts();
  buttonInterrupt = true;
}

void accFunction() //ISR
{
  wakeup();
  noInterrupts();
  accInterrupt = true;
}

void wireFunction() //ISR
{
  wakeup();
  noInterrupts();
  wireInterrupt = true;
}

uint16_t readBat() {
  for (byte i = 0; i < 3; i++) { //read multiple times for better Accuracy, espacially after deep sleep
    batVoltage = analogRead(ADC);
    delay(5);
  }  

  if (batVoltage < 490) {
    errorBlink(4);
    deepSleep();
  }
  return batVoltage;
}

void sendPackage(byte reason) { //package handler
   
  #ifdef ACC
  int8_t temp = readBMA2XX(BMAREG_TEMPERATURE);
  temp = (temp*0.5)+24;
  #endif

  #ifndef ACC
  int8_t temp = 0;
  #endif
  
  uint8_t bat = constrain(map(readBat(), 490, 615, 0, 99), 0, 99);
  #ifdef RADIO
  snprintf(payload, 28, "ID:%02d;INT:%02d;BAT:%02d;TMP:%02d;", NODEID, reason, bat, temp);
  radio.Wakeup();
  radio.Send(GATEWAYID, payload, strlen(payload) + 1, requestACK);
  memset(payload, 0, sizeof(payload));
  #if requestACK
  if (waitForAck()) Serial.print("ok!");
  else {
    errorBlink(3);
  }
  #endif
  #endif
}

void errorBlink(byte code) { //error blink sequence
  for (byte i = 0; i < code; i++) {
    digitalWrite(LED_R, HIGH);
    #ifdef BUZZER
    tone(BUZZ, 3200);
    #endif
    delay(100);
    digitalWrite(LED_R, LOW);
    noTone(BUZZ);
    delay(100);
  }
}

void shutdownMelody() { 
  tone(BUZZ, 1661);
  delay(300);
  tone(BUZZ, 1244);
  delay(300);
  tone(BUZZ, 830);
  delay(300);
  tone(BUZZ, 932);
  delay(300);
  noTone(BUZZ);
}

void startMelody() {
  tone(BUZZ, 1661);
  delay(225);
  tone(BUZZ, 622);
  delay(150);
  tone(BUZZ, 932);
  delay(300);
  tone(BUZZ, 830);
  delay(450);
  tone(BUZZ, 1661);
  delay(300);
  tone(BUZZ, 932);
  delay(600);
  noTone(BUZZ);  
}

#if requestACK
// wait a few milliseconds for proper ACK, return true if received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}
#endif

void bma2XXclearInterrupts()
{
  // clear interrupt enable flags
  writeBMA2XX(BMAREG_DETECT_OPTS1, 0x00);
  writeBMA2XX(BMAREG_DETECT_OPTS2, 0x00);
  // clear mapping
  writeBMA2XX(BMAREG_INT_MAP1, 0x00);
  writeBMA2XX(BMAREG_INT_MAP2, 0x00);
  writeBMA2XX(BMAREG_INT_MAP3, 0x00);
  // set pins tri-state 
  writeBMA2XX(BMAREG_INTPIN_OPTS, (INTPIN_ACTIVE_LO<<INTPIN_INT1_ACTLVL) | (INTPIN_OPENDRIVE<<INTPIN_INT1_DRV) | // make pins active low and open-collector ~ tri-state
                     (INTPIN_ACTIVE_LO<<INTPIN_INT2_ACTLVL) | (INTPIN_OPENDRIVE<<INTPIN_INT2_DRV) ); 
                   
  writeBMA2XX(BMAREG_INT_CTRL_LATCH, 1 << BMA_RESET_INT); // setting this clears any latched interrupts  
}

void bma2XXsetProfile()
{

  writeBMA2XX(BMAREG_SLEEP_DURATION, 0x00);// deactivate sleep mode
  delayMicroseconds(500); //SUSPEND/LPM1: a > 450us delay is required between writeBMA2XX transactions
  bma2XXclearInterrupts();
  
  //perform a soft reset, wait >30ms
  writeBMA2XX(BMAREG_SOFTRESET, BMA_SOFTRESET_MAGICNUMBER);
  delay(50); //wait for soft reset to complete
  
  writeBMA2XX(BMAREG_BANDWIDTH, BW_500Hz);     //500 Hz BW, 1000 samples per second
  writeBMA2XX(BMAREG_ACC_RANGE, ACC_2g);
  
  writeBMA2XX(BMAREG_INT_CTRL_LATCH, 0b1011); //set interrupt resetting behavior: temporary 1ms (active low)
  
  writeBMA2XX(BMAREG_DETECT_OPTS1, (1<<DO_SLOPE_Z_EN) | (1<<DO_SLOPE_Y_EN) | (1<<DO_SLOPE_X_EN) | (1<<DO_FLAT_EN)); //enable Slope and Flat Interrupt
  writeBMA2XX(BMAREG_SLOPE_THRESHOLD, (char)(15)); //configure slope detection: ds 4.8.5
  writeBMA2XX(BMAREG_SLOW_THRESHOLD, (char)(10)); //configure slow motion detection
  writeBMA2XX(BMAREG_SLOPE_DURATION, 0b00001011); //configure slope detection: ds 4.8.5
  
  writeBMA2XX(BMAREG_DETECT_OPTS3, (1<<DO_SLO_NO_MOT_SEL) | (1<<DO_SLO_NO_MOT_Z_EN)|(1<<DO_SLO_NO_MOT_X_EN)|(1<<DO_SLO_NO_MOT_Y_EN)); //no motion on all axis
  
  writeBMA2XX(BMAREG_INT_MAP1, 1<<MAP_INT1_FLAT | 1<<MAP_INT1_NO_MOTION | 1<<MAP_INT1_SLOPE);  // ap FLAT and No motion interrupt to INT1 pin
  writeBMA2XX(BMAREG_INTPIN_OPTS, (INTPIN_ACTIVE_LO<<INTPIN_INT1_ACTLVL) | (INTPIN_PUSHPULL<<INTPIN_INT1_DRV) |
                     (INTPIN_ACTIVE_LO<<INTPIN_INT2_ACTLVL) | (INTPIN_PUSHPULL<<INTPIN_INT2_DRV) ); //pins are active low and push-pull mode      

  writeBMA2XX(BMAREG_SLEEP_DURATION, 1 << BMA_LOWPOWER_ENA); //go to low power Mode
}

char readBMA2XX(uint8_t address)
{
  //returns the contents of any 1 byte register from any address
  char buf;

  START;
  SPI.transfer(address|READ);
  buf = SPI.transfer(0xFF);
  STOP;
  return buf;
}



void writeBMA2XX(uint8_t address, char data)
{
  //write any data byte to any single address
  START;
  SPI.transfer(address);
  SPI.transfer(data);
  STOP;

  delayMicroseconds(2);
}

