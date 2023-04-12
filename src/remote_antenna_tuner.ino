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


//#define stepper
//#define remote

#define IS_STEPPER  true

// Stepper Setup
#if IS_STEPPER
const uint8_t enable_pin = 6;
const uint8_t step_pin = 5;
const uint8_t direction_pin = 4;
#else
const uint8_t mode_button_pin = 2;
const uint8_t up_button_pin = 3;
const uint8_t down_button_pin = 4;
bool UP_FLAG = false;
bool DOWN_FLAG = false;
//const uint8_t num_steps_on_button = 50;
//const uint8_t step_frequency_hz = 100; //Hz
#endif

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

float payload = 0;

void setup() {

  Serial.begin(115200);

#if IS_STEPPER
  digitalWrite(enable_pin, HIGH); //Active low
  pinMode(enable_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  radioNumber = 0;
  Serial.println("I am the Stepper Driver and I love my job!");
#else
  pinMode(mode_button_pin, INPUT);
  pinMode(up_button_pin, INPUT);
  pinMode(down_button_pin, INPUT);
  radioNumber = 1;
  Serial.println("I am the remote and I work to live.");
  //attachInterrupt( digitalPinToInterrupt(mode_button_pin), ISR_MODE, RISING);
  //attachInterrupt( digitalPinToInterrupt(up_button_pin), ISR_UP, RISING);
#endif

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

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
  //int role = true;
  #if IS_STEPPER
    radio.startListening();  // put radio in RX mode
  #else
    radio.stopListening();  // put radio in TX mode
  #endif

}  // setup

#if !IS_STEPPER
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
#endif

void loop() {

#if IS_STEPPER

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
    
#else

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
  #endif

}  // loop
