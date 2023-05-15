/* Remote Antenna Tuner
 * For Stuart's dope Magloop tuner
 * Created: 2023-04-11
 */

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
//#include <millis.h>


#define IS_STEPPER  false

// Stepper Setup
const uint8_t enable_pin    = 9;
const uint8_t sleep_pin     = 6;
const uint8_t step_pin      = 5;
const uint8_t direction_pin = 4;

// Remote Setup
const uint8_t mode_button_pin = 2;
const uint8_t up_button_pin = 4;
const uint8_t down_button_pin = 3;
const uint8_t LED_ONE = A1;
const uint8_t LED_TWO = A0;
bool UP_FLAG = false;
bool DOWN_FLAG = false;

const int SPEED_ZERO_STEPS  = 1;
const int SPEED_ONE_STEPS   = 10;
const int SPEED_TWO_STEPS   = 100;
const int SPEED_THREE_STEPS = 500;

unsigned long NEXT_BUTTON_CHECK = 0;
unsigned long BUTTON_COOL_DOWN_MS = 200;

uint8_t DEBOUNCE_DELAY_MS = 100;
uint8_t UP_BUTTON_FLAG = false;
uint8_t DOWN_BUTTON_FLAG = false;
uint8_t MODE_BUTTON_FLAG = false;
uint8_t SPEED_MODE = 1; // 0, 1, 2, or 3 indicate the number of steps per button press.

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

int payload = 0;

void setup() {

  Serial.begin(115200);

#if IS_STEPPER
  pinMode(enable_pin, OUTPUT);
  pinMode(sleep_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH); //Active low
  digitalWrite(sleep_pin, HIGH); //Active low
  radioNumber = 0;
  Serial.println("I am the Stepper Driver and I love my job!");
#else
  pinMode(mode_button_pin, INPUT);
  pinMode(up_button_pin, INPUT);
  pinMode(down_button_pin, INPUT);
  pinMode(LED_ONE, OUTPUT);
  pinMode(LED_TWO, OUTPUT);
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

/*
void transmit_button(){

  Serial.println("handling button");

  float up = 10.0;
  float down = -10.0;
  
  if(UP_FLAG == true){
    bool report = radio.write(&up, sizeof(float));  // transmit & save the report
  }else if(DOWN_FLAG == true){
    bool report = radio.write(&down, sizeof(float));  // transmit & save the report
  }

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
*/

void stepper_main(){
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
    

}

void handle_buttons(){
  
  if(digitalRead(up_button_pin) == HIGH){
    if(UP_BUTTON_FLAG == false){
      UP_BUTTON_FLAG = true;
      NEXT_BUTTON_CHECK = millis() + DEBOUNCE_DELAY_MS;
    }else if(millis() < NEXT_BUTTON_CHECK){
      upButtonPressed();
      UP_BUTTON_FLAG = false;
      delay(BUTTON_COOL_DOWN_MS);
    }
  }

  if(digitalRead(down_button_pin) == HIGH){
    if(DOWN_BUTTON_FLAG == false){
      DOWN_BUTTON_FLAG = true;
      NEXT_BUTTON_CHECK = millis() + DEBOUNCE_DELAY_MS;
    }else if(millis() < NEXT_BUTTON_CHECK){
      downButtonPressed();
      DOWN_BUTTON_FLAG = false;
      delay(BUTTON_COOL_DOWN_MS);
    }
  }

  if(digitalRead(mode_button_pin) == HIGH){
    if(MODE_BUTTON_FLAG == false){
      MODE_BUTTON_FLAG = true;
      NEXT_BUTTON_CHECK = millis() + DEBOUNCE_DELAY_MS;
    }else if(millis() < NEXT_BUTTON_CHECK){
      modeButtonPressed();
      MODE_BUTTON_FLAG = false;
      delay(BUTTON_COOL_DOWN_MS);
    }
  }

  return;
}

void modeButtonPressed(){

  Serial.println("Mode Button!");

  SPEED_MODE++;
  if(SPEED_MODE > 3) SPEED_MODE = 0;

  digitalWrite(LED_ONE, LOW);
  digitalWrite(LED_TWO, LOW);

  if(SPEED_MODE == 1 || SPEED_MODE == 3) digitalWrite(LED_ONE, HIGH);
  if(SPEED_MODE == 2 || SPEED_MODE == 3) digitalWrite(LED_TWO, HIGH);

}

void downButtonPressed(){

  if(SPEED_MODE == 0) sendSteps((-1)*SPEED_ZERO_STEPS);
  if(SPEED_MODE == 1) sendSteps((-1)*SPEED_ONE_STEPS);
  if(SPEED_MODE == 2) sendSteps((-1)*SPEED_TWO_STEPS);
  if(SPEED_MODE == 3) sendSteps((-1)*SPEED_THREE_STEPS);

}

void upButtonPressed(){

  if(SPEED_MODE == 0) sendSteps(SPEED_ZERO_STEPS);
  if(SPEED_MODE == 1) sendSteps(SPEED_ONE_STEPS);
  if(SPEED_MODE == 2) sendSteps(SPEED_TWO_STEPS);
  if(SPEED_MODE == 3) sendSteps(SPEED_THREE_STEPS);

}

bool sendSteps(int input){
  
    Serial.print("Printing: ");
    Serial.println(input);

    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&input, sizeof(int));  // transmit & save the report
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

  return report;
}


void remote_main(){
  while(1){

    handle_buttons();

      //Serial.println("What number to send?");
      if (Serial.available()){
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
        Serial.println("What number to send?");
      }

  }

}


void loop() {

  if(IS_STEPPER){
    stepper_main();
  }else{
    Serial.println("What number to send?");
    remote_main();
  }

}  // loop
