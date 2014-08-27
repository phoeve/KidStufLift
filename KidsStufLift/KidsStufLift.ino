//
//   KIDSTUF DMX Decoder and Stepper Motor Controller
//          This program is intended to run on a Arduino microcontroller.  It receives DMX packets, decodes
//          them, and spins the stepper motor corresponding to the DMX value. 
//          The program replaces the Arduino interrupt handler with its own ISR(USART3_RX_vect) to 
//          process incoming packets via a state machine.
//
//
//   Supported Processors (tested)
//          - Arduino Mega
//
//   Supported Stepper Motor (tested)
//          - NEMA 34
//
//   Author(s) Peter Hoeve (peter@hoeve.us)  NPCC Production Team
//
//   Based on example code from Akiba@Freaklabs.com and Arduino
//
//   History:
//         8/ 6/14 - 0.1 (PH) Original
//         8/20/14 - 0.2 (PH) Tested and Debugged w/light panel
//         8/22/14 - 0.3 (PH) Tested installed rig.  Streamlined code and wrote better/faster mapDmx()
//

//#define UNIT_TESTING        


#include <AccelStepper.h>

#define NUM_CHANNELS 2 // Number of DMX channels to listen to

                  // DMX channel assignments (relative to base address)
#define DMX_POSITION_CHANNEL 0
#define DMX_SPEED_CHANNEL 1

#define FULL_UP       0
#define FULL_DOWN     15000            // Needs to be calibrated
#define HOME_OFFSET   FULL_DOWN/20     // Danger zone.  Slow home near home !!
#define MIN_SPEED     1                // 0 causes divide by zero in AccelStepper library
#define PANIC_SPEED   MIN_SPEED
#define MAX_SPEED     1500
#define HOME_SPEED    200
#define ACCELERATION  1000

      // DMX info/ranges
#define DMX_MIN_VALUE 0
#define DMX_MAX_VALUE 255
      // DMX packet states
enum {DMX_IDLE,  DMX_BREAK,  DMX_START,  DMX_RUN};
      // current state of DMX state machine
volatile unsigned char dmx_state;                // http://arduino.cc/en/Reference/Volatile
      // this is the start address for the dmx frame
unsigned int dmx_start_addr = 1;
      // this is the current address of the dmx frame
unsigned int dmx_addr;
      // this is used to keep track of the channels
unsigned int chan_cnt;
      // this holds the dmx data
unsigned char dmx_data[NUM_CHANNELS];

      // tell us when to update the pins
volatile unsigned char update;    


     
#define DIP_ON LOW             // If voltage is LOW (grounded), the dip switch is in the ON position.  
#define NUM_ADDRESS_BITS 9     // 2**9 gives 0-511
unsigned int address_pins[NUM_ADDRESS_BITS] = {50,48,46,44,42,40,38,36,34};
unsigned int myBaseAddress = 0;


 // initialize the stepper library on pins 8 and 9 (use a driver box):
AccelStepper stepper(AccelStepper::DRIVER, 9,8); // 9-PUL,8-DIR

#define HOME_SWITCH_PIN 11        // Arduino pins


boolean calibrated = false;      // until we hit the limit switch, we are NOT calibrated.  PANIC also un-calibrates !

boolean homeSwitchEngaged()
{
  if(digitalRead(HOME_SWITCH_PIN) != DIP_ON)
    return true;
    
  return false;
}

//===============================================================================================================


void setup()
{
  int i;
  unsigned char val=0;

  // Console port
  Serial.begin(57600);
  Serial.write("setup() ...\n");  

               // Initialize the origin / HOME switch
  pinMode(HOME_SWITCH_PIN, INPUT);           // set pin to input
  digitalWrite(HOME_SWITCH_PIN, HIGH);       // turn on pullup resistor
               
               // On power up or reset read base address from dip switch 
  for (i=0; i<NUM_ADDRESS_BITS; i++)
  {
    pinMode(address_pins[i], INPUT);           // set pin to input
    digitalWrite(address_pins[i], HIGH);       // turn on pullup resistor
    
    val = digitalRead(address_pins[i]);
    Serial.print(val);
    if (val==DIP_ON)
      bitSet(myBaseAddress, NUM_ADDRESS_BITS -i -1);   
  }
  
  dmx_start_addr = myBaseAddress;    // convert 0-511 -> 1-512
  //dmx_start_addr = myBaseAddress +1;    // convert 0-511 -> 1-512
  
  Serial.write("myBaseAddress is ");
  Serial.print(myBaseAddress);
  Serial.write("\n");
  Serial.write("dmx_start_addr is ");
  Serial.print(dmx_start_addr);
  Serial.write("\n");


    // set update flag idle
  update = 0;
  
  // set default DMX state
  dmx_state = DMX_IDLE;
   
  stepper.setAcceleration(ACCELERATION);   
  stepper.setCurrentPosition(0);                // assume we are at zero !
  stepper.moveTo (-FULL_DOWN);                  // Send us to the home switch
  calibrated = false; 
  
#ifndef UNIT_TESTING
  // initialize UART for DMX
  // 250 kbps, 8 bits, no parity, 2 stop bits
  UCSR3C |= (1<<USBS3);
  Serial3.begin(250000);
#endif

}

#ifdef UNIT_TESTING
long iteration = 0;
void simulateDmx(){
  iteration ++;
  
  update = true;
  
  if (iteration > 25){
    if (iteration > 50){
      dmx_data[DMX_SPEED_CHANNEL] = 128;
      dmx_data[DMX_POSITION_CHANNEL] = 128;
    }
    else{
      dmx_data[DMX_SPEED_CHANNEL] = 0;
      dmx_data[DMX_POSITION_CHANNEL] = 128;
    }
  }
  else{
    dmx_data[DMX_SPEED_CHANNEL] = 128;
    dmx_data[DMX_POSITION_CHANNEL] = 128;
  }
  
  delay(1000);

}
#endif

/**************************************************************************/
/*!
  This is where we translate the dmx data to the lift commands
*/
/**************************************************************************/

boolean paniced = false;

void loop()
{
  
  int newPosition, newSpeed;
  
#ifdef UNIT_TESTING
  simulateDmx();
#endif

#ifdef UNIT_TESTING
    Serial.write("update: ");
    Serial.print(update);
    Serial.write(" calibrated: ");
    Serial.print(calibrated);
    Serial.write(" paniced: ");
    Serial.print(paniced);
    Serial.write("\n"); 
#endif  

  if ((update && calibrated) || paniced) {      // only if DMX activity and we are calibrated (accepting DMX directives) OR PANICED
  
    update = 0;                    // our indication the ISR set some data.
  
    newPosition = map (dmx_data[DMX_POSITION_CHANNEL], DMX_MIN_VALUE, DMX_MAX_VALUE, FULL_UP, FULL_DOWN);
    newSpeed = map (dmx_data[DMX_SPEED_CHANNEL], DMX_MIN_VALUE, DMX_MAX_VALUE, MIN_SPEED, MAX_SPEED);    // 1 is lowest speed (MIN_SPEED).
    
    if (newPosition == 0)
      newPosition = -HOME_OFFSET;      // If DMX target position zero, let's go beyond and wait for HOME_SWITCH

    if (newSpeed == PANIC_SPEED) {    // MIN_SPEED means PANIC !!!!!!!!! 
      paniced = true; 
      stepper.setCurrentPosition(0);                // assume we are at zero !
      stepper.moveTo (-FULL_DOWN);                  // Send us to the home switch when un-paniced
      calibrated = false; 
    } else {
      if (paniced){
        paniced = false;          // un-paniced i.e. newSpeed != PANIC_SPEED .... but now we are !calibrated
      }
      else{
        stepper.setMaxSpeed(newSpeed);
        if (newPosition != stepper.targetPosition()) {
          stepper.moveTo (newPosition);  
        }
      }
    }
    
  }
    
  if (stepper.currentPosition() > stepper.targetPosition()) {  // Are we raising ??
    if (stepper.currentPosition() <= HOME_OFFSET)              // SLOW if we are raising and we are at or above HOME_OFFSET (setCurrentPosition(0) will do it)
      stepper.setMaxSpeed(HOME_SPEED);                         // Override DMX speed - danger zone
                                                               // Else allow DMX to drive speed

    if(homeSwitchEngaged()) {                  // Only if we are raising !
      calibrated = true;
      stepper.setCurrentPosition(0);           // We are HOME !!! 
      stepper.moveTo (0);                      // make targetPosition() match current position.
    }    
  }
  
  if (!paniced){
#ifdef UNIT_TESTING
    Serial.write("currentPosition: ");
    Serial.print(stepper.currentPosition());
    Serial.write(" targetPosition: ");
    Serial.print(stepper.targetPosition());
    Serial.write("\n"); 
#endif  
    stepper.run();   // Move 1 step towards targetPosition() unless PANICED
  }

}





#ifndef UNIT_TESTING
/**************************************************************************/
/*!
  This is the interrupt service handler for the DMX
*/
/**************************************************************************/
ISR(USART3_RX_vect)
{
  unsigned char status = UCSR3A;
  unsigned char data = UDR3;
  switch (dmx_state)
  {
    case DMX_IDLE:
      if (status & (1<<FE0))
      {
        dmx_addr = 0;
        dmx_state = DMX_BREAK;
        update = 1;
      }
    break;
    
    case DMX_BREAK:
      if (data == 0)
      {
        dmx_state = DMX_START;
      }
      else
      {
        dmx_state = DMX_IDLE;
      }
    break;
    
    case DMX_START:
      
      dmx_addr++;
      if (dmx_addr == dmx_start_addr)
      {
        chan_cnt = 0;
        dmx_data[chan_cnt++] = data;
        dmx_state = DMX_RUN;
      }
    break;
    
    case DMX_RUN:
      dmx_data[chan_cnt++] = data;
      if (chan_cnt >= NUM_CHANNELS)
      {
        dmx_state = DMX_IDLE;
      }
    break;
    
    default:
      dmx_state = DMX_IDLE;
    break;
  }
}
#endif
