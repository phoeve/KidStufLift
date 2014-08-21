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

//
#include <AccelStepper.h>

#define NUM_CHANNELS 2 // Number of DMX channels used

      // DMX channel assignments (relative to base address)
#define DMX_POSITION_CHANNEL 0
#define DMX_SPEED_CHANNEL 1

#define FULL_UP       0
#define FULL_DOWN     25000      // Needs to be calibrated
#define HOME_OFFSET   FULL_DOWN/10    // Danger zone.  Don't drive past the limit switch !!
#define MAX_SPEED     1500
#define HOME_SPEED    100
#define ACCELERATION  750
#define HOME_ACCEL    10000

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
unsigned char last_dmx_data[NUM_CHANNELS];

      // tell us when to update the pins
volatile unsigned char update;    


     
#define DIP_ON LOW             // If voltage is LOW (grounded), the dip switch is in the ON position.  
#define NUM_ADDRESS_BITS 9     // 2**9 gives 0-511
unsigned int address_pins[NUM_ADDRESS_BITS] = {50,48,46,44,42,40,38,36,34};
unsigned int myBaseAddress = 0;


 // initialize the stepper library on pins 8 and 9 (use a driver box):
AccelStepper stepper(AccelStepper::DRIVER, 9,8); // 9-PUL,8-DIR

#define HOME_SWITCH_PIN 11
#define ENABLE_PIN 12


void liftHome()
{
                            // Home stepper motor 
                            // move stepper until the micro-switch clicks - loop here until engaged.
  Serial.write("Backing up to HOME_OFFSET ...");
  
  stepper.setMaxSpeed(MAX_SPEED);       // depending on drive pulley size
  stepper.setAcceleration(ACCELERATION);    

  Serial.write("stepper.currentPosition() is ");
  Serial.print(stepper.currentPosition());
  Serial.write("\n");

  Serial.write("... Back up fast ... ");
  if (stepper.currentPosition() > HOME_OFFSET){        //Zoom to offset, maybe 250 steps (2-3 feet) from top
    stepper.moveTo(HOME_OFFSET);
    while (stepper.currentPosition() > HOME_OFFSET)
      stepper.run();                            // loop until current == target
  }
     
  Serial.write("... Back up slow ... ");
  stepper.setMaxSpeed(HOME_SPEED);            // Home slow .......
  stepper.setAcceleration(HOME_ACCEL);       
  
  stepper.move(-FULL_DOWN);            // Zero is where the switch clicks !
  while (true){
    stepper.run();
    if(digitalRead(HOME_SWITCH_PIN) != DIP_ON){
        stepper.setCurrentPosition(0);    
        break;                            // Stop !!!!!!!!!
    }
  }
  
  Serial.write("... At Home !\n");

          // Go back to full speed
  stepper.setMaxSpeed(MAX_SPEED);       // depending on drive pulley size
  stepper.setAcceleration(ACCELERATION);    

}

boolean stepperEnabled = true;
void stepperEnable()              // Enable/Disable stops motor no matter what state
{  
  stepperEnabled = true;
  digitalWrite(ENABLE_PIN, LOW);  // LOW enables driver/motor
}


void stepperDisable()
{
  stepperEnabled = false;
  digitalWrite(ENABLE_PIN, HIGH);        // HIGH disables driver/motor
}


boolean anyDmxChanges()
{                      // Fast check for DMX value changes - make sure we init both arrays to all zeros in setup()
  
  if (update){         // don't loop if no DMX packets received
    
    update = false;  
    
    for (int i=0; i<NUM_CHANNELS; i++){
      if (dmx_data[i] != last_dmx_data[i]){      // Any changes to DMX channels ?
        for (int j=0; j<NUM_CHANNELS; j++)
          last_dmx_data[j] = dmx_data[j];        // reset last to show change
        return true;
      }
    }   
  }
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

  pinMode(ENABLE_PIN, OUTPUT);           // set ENABLE pin to output

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

  for (int j=0; j<NUM_CHANNELS; j++)        // Initialize all DMX values
      last_dmx_data[j] = dmx_data[j] = 0;


    // set update flag idle
  update = 0;
  
  // set default DMX state
  dmx_state = DMX_IDLE;
  
  
  
 
  stepper.setCurrentPosition(HOME_OFFSET);      // assume we are near home
  
  stepperEnable();
  
  liftHome();       // If we have been disabled, we must home to reset position
  
  // initialize UART for DMX
  // 250 kbps, 8 bits, no parity, 2 stop bits
  UCSR3C |= (1<<USBS3);
  Serial3.begin(250000);

}



/**************************************************************************/
/*!
  This is where we translate the dmx data to the lift commands
*/
/**************************************************************************/


#define READ_DMX

void loop()
{
  unsigned int position, speed;
  
#if 0
  if(digitalRead(HOME_SWITCH_PIN) != DIP_ON){      // SAFETY CODE - This should not happen !!!  Switch engaged ?
      Serial.write("Limit Switch Engaged\n");
    stepper.setCurrentPosition(0);                // Reset "home" here
  }
#endif

  if (anyDmxChanges())      // DMX activity directed at our channels and different values from last check ?
  {
    update = false;
        
#ifdef READ_DMX
    for (int j=0; j<NUM_CHANNELS; j++)
    {
      Serial.print(j);
      Serial.write("=");
      Serial.print(dmx_data[j]);
      Serial.write(",");
    }
    Serial.write("\n");
    
            //
            //  NEW POSITION CHANNEL
    if (last_dmx_data[DMX_POSITION_CHANNEL] != dmx_data[DMX_POSITION_CHANNEL] || 1){      // new value ?
    
      position = map (dmx_data[DMX_POSITION_CHANNEL], DMX_MIN_VALUE, DMX_MAX_VALUE, 0, FULL_DOWN);
      Serial.write("DMX = ");
      Serial.print(dmx_data[DMX_POSITION_CHANNEL]);
      Serial.write("->New Position = ");
      Serial.print(position);
      Serial.write("\n");

      
      if (position > HOME_OFFSET)             
        stepper.moveTo (position);   // if at > HOME_OFFSET, ok, to go fast
      else if (position == 0)
            liftHome();              // Let's just go home - user has to wait until we reset.
          else
            stepper.moveTo (HOME_OFFSET);     // Let's just got to HOME_OFFSET, fast is ok
 
      last_dmx_data[DMX_POSITION_CHANNEL] = dmx_data[DMX_POSITION_CHANNEL];
    }
    
            //
            // NEW SPEED CHANNEL
    if (last_dmx_data[DMX_SPEED_CHANNEL] != dmx_data[DMX_SPEED_CHANNEL] || 1){      // new value ?
      
      speed = map (dmx_data[DMX_SPEED_CHANNEL], DMX_MIN_VALUE, DMX_MAX_VALUE, 1, MAX_SPEED);    // 1 is lowest speed.
      
      Serial.write("New Speed = ");
      Serial.print(speed);
      Serial.write("\n");
      
      if (speed == 1){
        Serial.write("Light panel ent us DMZ=0 for Speed - Disabling Stepper\n");
        stepper.setMaxSpeed (1);  // Whoooooahh - operator says STOP - PANIC  (no divide by zero in AccelStepper library !)
        stepperDisable();         // Whoooooahh - operator says STOP - PANIC
      }
      else{
        if (!stepperEnabled){
          stepperEnable();
          liftHome();
        }
        stepper.setMaxSpeed (speed);
      }       

      last_dmx_data[DMX_SPEED_CHANNEL] = dmx_data[DMX_SPEED_CHANNEL];
    }
#endif 




  } // if (andDmxChanges())
    
  
                      // SIMULATION
#ifndef READ_DMX   
  if (stepper.currentPosition() == stepper.targetPosition()){
    delay(2000);
    if(stepper.currentPosition()==FULL_DOWN){
      Serial.write("0");
      Serial.write("\n");
      stepper.moveTo(FULL_UP);
    }
    if(stepper.currentPosition()==0){
      Serial.write("24000");
      Serial.write("\n");
      stepper.moveTo(FULL_DOWN);
    }
  }
#endif

  stepper.run();   // Call this as often as possible !!!!

}






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