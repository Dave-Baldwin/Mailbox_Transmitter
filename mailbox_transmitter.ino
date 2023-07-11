// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <RHDatagram.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

//RH communication
#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

#define VBATPIN A6
float measuredvbat = 0.0;
   
// Set the wdt 8s cycle counter increment limit before transmit
const int wdtCycleLimit = 24;   //24*8 seconds = 192s == 3+ minutes between timed sends
int wdtCycleCount = 0;

int doorClosedPin = 2;

bool doorClosedState = false;
bool doorClosedStateLast = false;
bool intDrivenWrite = false;
bool trigSendVals = false;

//RH_ASK driver;
// default transmit pin is pin 12
 RH_ASK driver(2000, 4, 7, 10);
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
//RH_ASK driver(2000, 4, 6, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
// RH_ASK driver(2000, PD14, PD13, 0); STM32F4 Discovery: see tx and rx on Orange and Red LEDS
// RH_ASK (uint16_t speed=2000, uint8_t rxPin=11, uint8_t txPin=12, uint8_t pttPin=10, bool pttInverted=false)

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram manager(driver, CLIENT_ADDRESS);

struct dataStruct{
  float battVoltage;
  bool boxOpen;
} SensorReadings;

// RF communication, Dont put this on the stack:
byte buf[sizeof(SensorReadings)] = {0};

// Define timeout time in milliseconds (example: 2000ms = 2s)
const long wdUpdateFreq = 60000;
long lastWDUpdateMillis = -61000;
int wdUpdateVal = 0;

void setup() {
   
  Serial.begin(9600);	  // Debugging

  // initialize led pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // turn LED on to show starting setup routine
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  
  // give time before low power functions take effect and possibly affect ability to upload sketches?
  delay(3000);

  // sleep wakeup watchdog timer setup
  watchdogSetup();

  // template code for LED light-up, not needed here
  //PORTD |= (1<<PD7);  // equals (roughly) digitalWrite(7, LOW);
  //delay(3500);
  //PORTD &= ~(1<<PD7); // equals (roughly) digitalWrite(7, LOW);
  //delay(3500);

  //pinMode(battChargingPin, INPUT);    // sets the digital pins as input
  //pinMode(battDonePin, INPUT);
  pinMode(doorClosedPin, INPUT_PULLUP);

  //RF communication
  if (!manager.init()) {
    Serial.println("init failed");
  }

  SensorReadings.boxOpen = false;
  SensorReadings.battVoltage = -1.0;

  Serial.println("Setup done");
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (HIGH is the off voltage level)
  
} // end setup

void loop()
{

  // reset watchdog timer
  wdt_reset();

  // attach interrupt at beginning of loop, it will be detached in ISR function
  attachInterrupt(digitalPinToInterrupt(doorClosedPin), triggerXmit, RISING);
  
  // after sleep finishes, the rest of the loop code will execute:
  measuredvbat = analogRead(VBATPIN);
  measuredvbat = analogRead(VBATPIN); // read a second time because apparently first measurement after wake is typically not good??
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " );
  //Serial.println(measuredvbat);

  SensorReadings.battVoltage = measuredvbat;

  // read digital inputs
  //if(digitalRead(battChargingPin)) {
  //  SensorReadings.chargingState = 1;
  //} else if(digitalRead(battDonePin)) {
  //  SensorReadings.chargingState = 2;
  //} else {
//    SensorReadings.chargingState = 0;
  //}

  // determine door closed/open state
  if(digitalRead(doorClosedPin)) {    // if high, then pullup resistor is dominating and the door is open
    doorClosedState = false;
  } else if(!digitalRead(doorClosedPin)) {
    doorClosedState = true;
  } else {
    Serial.println("Unexpected result on door closed pin read!");
  }
  if(doorClosedState != doorClosedStateLast) {
    Serial.print("Door state changed; pin read value: "); 
    Serial.println(digitalRead(doorClosedPin));
  }
  // update last value variable
  doorClosedStateLast = doorClosedState;
  // update data structure value for radio sending
  SensorReadings.boxOpen = !doorClosedState;

  // on timer expiry, send watchdog update
  //if(millis() - lastWDUpdateMillis > wdUpdateFreq) {
  //  trigSendVals = true;
  //} // end update

  // if the trigger has been set true, then send transmission
  if(trigSendVals || wdtCycleLimit == wdtCycleCount) {
    sendValues();

    Serial.print("batt volts sent: ");
    Serial.println(SensorReadings.battVoltage);
    Serial.print("door opn sent: ");
    Serial.println(SensorReadings.boxOpen);

    // reset send trigger
    trigSendVals = false;

    // reset watchdog cycle count
    wdtCycleCount = 0;

    // reset Transmit timer
    lastWDUpdateMillis = millis();
    
  }

  // trigger sleep at end of loop, with power down functions
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // choose power down mode
  cli();
  sleep_enable();
  power_adc_disable(); 
  //power_usart0_disable();
  power_spi_disable(); 
  power_timer0_disable();
  power_timer1_disable(); 
  power_timer2_disable(); 
  power_twi_disable();
  sleep_bod_disable(); // disable brown-out detector
  sei();
  sleep_cpu();
  sleep_disable();
  power_all_enable();

  //Serial.println("end loop");
  
} // end loop

void triggerXmit() {
  detachInterrupt(doorClosedPin); // external interrupt disable (INT0)
  trigSendVals = true;
  intDrivenWrite = true;
}

void sendValues() {

  byte zize=sizeof(SensorReadings);
  memcpy (buf, &SensorReadings, zize);
  
  Serial.println("Send to ask_datagram_srv");
  int i = 0;
  int repeat = 4; //Times to repeat same message - recommended by online sources to repeat since we are transmit-only and receiver cannot confirm receipt.
    
  // Send a message to manager_server
  while (i<repeat) {
    if (manager.sendto(buf, zize, SERVER_ADDRESS)) {
      Serial.println("Msg sent");
    } else {
      Serial.println("send fail");
    }
    delay(100);
    i = i+1;
  }

  if(intDrivenWrite) {
    Serial.println("Intrpt xmt"); 
    intDrivenWrite = false;
  } else {
    Serial.println("Timer xmt");
  }
      
  // Send a message to manager_server
  //if (manager.sendto(buf, zize, SERVER_ADDRESS)) {
  //  Serial.println("Message sent");   
  //} else {
  //  Serial.println("sendto failed");
  //}
}

void watchdogSetup(void){
  cli();
  wdt_reset();
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDIE) | (0<<WDE) | (1<<WDP3) | (1<<WDP0);  // 8s / interrupt, no system reset
  sei();
}

ISR(WDT_vect) {   // code to execute after 8s wakeup goes here.
  // increment watchdog cycle counter
  wdtCycleCount++;
}
