/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// Stepper Setup

const uint8_t enable_pin = 6;
const uint8_t step_pin = 5;
const uint8_t direction_pin = 4;

const uint8_t up_button_pin = 2;
const uint8_t down_button_pin = 3;
bool UP_FLAG = false;
bool DOWN_FLAG = false;
const uint8_t num_steps_on_button = 50;
const uint8_t step_frequency_hz = 100; //Hz

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0;
//int payload = 0;

void setup() {

  Serial.begin(115200);

  digitalWrite(enable_pin, HIGH); //Active low
  pinMode(enable_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);

  pinMode(up_button_pin, OUTPUT);
  pinMode(down_button_pin, OUTPUT);
  attachInterrupt( digitalPinToInterrupt(up_button_pin), ISR_UP, RISING);
  attachInterrupt( digitalPinToInterrupt(down_button_pin), ISR_DOWN, RISING);

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // To set the radioNumber via the Serial monitor on startup
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  char input = Serial.parseInt();
  radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

}  // setup

void ISR_UP(){
  UP_FLAG = true;
  return;
}

void ISR_DOWN(){
  DOWN_FLAG = true;
  return;
}

void handle_button(){

  Serial.println("handling button");

  float up = 10.0;
  float down = -10.0;
  
  if(UP_FLAG == true){
    bool report = radio.write(&up, sizeof(float));  // transmit & save the report
  }else if(DOWN_FLAG == true){
    bool report = radio.write(&down, sizeof(float));  // transmit & save the report
  }

  UP_FLAG = false;
  DOWN_FLAG = false;

  // if(UP_FLAG == true) digitalWrite(direction_pin, HIGH);
  // if(DOWN_FLAG == true) digitalWrite(direction_pin, LOW);
  // digitalWrite(enable_pin, LOW);
  // // int i;

  // for(i=0; i<num_steps_on_button; i++){
    // digitalWrite(step_pin, HIGH);
    // delay( 1.0/(float)step_frequency_hz*1000.0*0.1 );
    // digitalWrite(step_pin, LOW);
    // delay( 1.0/(float)step_frequency_hz*1000.0*0.1 );
  // }

  // digitalWrite(enable_pin, HIGH);
}

void loop() {

  if (role) {
    // This device is a TX node

    Serial.println("What number to send?");
    while (!Serial.available()) {
      if( UP_FLAG == true || DOWN_FLAG == true) handle_button();
     // wait for user input
    }
    float input = Serial.parseFloat();
    Serial.print("Printing: ");
    Serial.println(input);

    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&input, sizeof(float));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer
    
    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      //Serial.println(input);  // print payload sent
    }else{
      Serial.println("Failure");
    }

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);  // print the payload's value
    

    
    if(payload > 0){
      digitalWrite(direction_pin, HIGH);
    }else{
      digitalWrite(direction_pin, LOW);
      payload *= -1;
    }

    digitalWrite(enable_pin, LOW);
    delay(10);
    int steps = (int)payload;
    int i;

    Serial.print("Stepper Enabled: ");
    Serial.println(steps);

    for(i=0; i<steps; i++)
    {
      digitalWrite(step_pin, HIGH);
      delay(1);
      digitalWrite(step_pin, LOW);
      delay(10);
    }

    digitalWrite(enable_pin, HIGH);
    Serial.println("Stepper Disabled");
    }
    
  }  // role

  if (Serial.available()) {
    // change the role via the serial monitor

    char c = toupper(Serial.read());
    if (c == 'T' && !role) {
      // Become the TX node

      role = true;
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      radio.stopListening();

    } else if (c == 'R' && role) {
      // Become the RX node

      role = false;
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      radio.startListening();
    }
  }

}  // loop
