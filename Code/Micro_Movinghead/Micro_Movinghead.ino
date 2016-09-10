/* This code is highly influenced by the example codes of the Conceptinetics (https://www.tindie.com/products/Conceptinetics/dmx-shield-for-arduino-remote-device-management-capable/) and servo (https://www.arduino.cc/en/reference/servo) libaries. 
   Thanks for the example code. 
   
  Copyright (c) 2015 Micro movinghead project. All right reserved.

   This code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 3 of the License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// Import of libaries
#include <Conceptinetics.h>
#include <Servo.h> 

// Set number of DMX channels
#define DMX_SLAVE_CHANNELS 7

// Set RX enable
#define RXEN_PIN                2

// Strobo configuration
#define STROBO_start_level 10 // Value of the DMX strobo channel needs to exceed to start strobo
#define STROBO_flash_offset 51 // Offset of length of flash in ms
#define STROBO_flash_ratio -6 // Inverse ratio of DMX strobo level and length of flash in ms
#define STROBO_blackout_offset 4110 // Offset of length of blackout in ms
#define STROBO_blackout_ratio -16 // Inverse ratio of DMX strobo level and length of blackout in ms

// Servo delay in ms
#define Servodelay 15

// Configure a DMX slave controller
DMX_Slave dmx_slave ( DMX_SLAVE_CHANNELS , RXEN_PIN );

// Pin numbers of the led output
#define RED_out 6
#define GREEN_out 5
#define BLUE_out 3

// Decalaration and initiation of global variables

// Color channels
int RED = 0;
int GREEN = 0;
int BLUE = 0;

// Dimmer and strobo channel
int DIM = 0;
int STROBO = 0;

// Movement channels
int PAN = 127;
int TILT = 127;

// Internal timing varibles
unsigned long now;
unsigned long start_led = 0;
unsigned long start_pan = 0;
unsigned long start_tilt = 0;

// Timeout variables
unsigned long       lastFrameReceivedTime;
const unsigned long dmxTimeoutMillis = 2000UL;

// Decalaration of servos
Servo panservo; 
Servo tiltservo; 

// the setup routine runs once at startup
void setup() {   
             
  // Enable DMX slave interface and start recording
  // DMX data
  dmx_slave.enable ();  
  
  
  // DMX adress pins startup
  InitialiseIO();
  
  // Set startadress
  dmx_slave.setStartAddress (readDMXadress());

  // Set led pins as output pins
  pinMode ( RED_out, OUTPUT );
  pinMode ( GREEN_out, OUTPUT );
  pinMode ( BLUE_out, OUTPUT );

  // Attach servos to the correct pins
  panservo.attach(12);
  tiltservo.attach(13); 

  // Servo to mid position
  panservo.write(90);
  tiltservo.write(90);

  // DMX timeout setting
  dmx_slave.onReceiveComplete ( OnFrameReceiveComplete );
}

// the loop routine runs over and over again forever:
void loop() {
    // Setting startadress in case of a changed startadress
    dmx_slave.setStartAddress (readDMXadress());
    
    now=millis(); // Get new now time
    if ( now - lastFrameReceivedTime < dmxTimeoutMillis ) {
      // In case of no timeout the DMX data is stored in internal variables
      // NOTE:
      // getChannelValue is relative to the configured startaddress
      
      // Reading DMX data
      PAN = dmx_slave.getChannelValue (1);
      TILT = dmx_slave.getChannelValue (2);
      RED = dmx_slave.getChannelValue (3);   
      GREEN = dmx_slave.getChannelValue (4);
      BLUE = dmx_slave.getChannelValue (5);  
      DIM = dmx_slave.getChannelValue (6);
      STROBO = dmx_slave.getChannelValue (7);
      
      // Setting Led
      if (STROBO < STROBO_start_level) {
          // In case of no strobo (Strobo channel is below STROBO_start_level) the led is set by it's color channel and a dimmer channel. Also the strobo timing clock will be reset
          PWM_intensities(RED,GREEN,BLUE,DIM);
          start_led = millis();
      } else {
          now = millis();
          if ((now-start_led) < STROBO_flash_offset+STROBO/STROBO_flash_ratio) {
            // In case a strobo is active then there will first be a flash (controlled by the colorchannels and dimmer channel) of STROBO_flash_length
            PWM_intensities(RED,GREEN,BLUE,DIM);
          } else if (now-start_led < STROBO_blackout_offset+(STROBO_blackout_ratio*STROBO)) {
            // After which a blackout will happen. It's time is depended on the value of the strobo channel. A lower value will result in a longer blackout
            PWM_intensities(0,0,0,0);
          } else {
            // After the blackout happened the led will reset the strobo timing clock and start a new flash
            start_led = millis();
            PWM_intensities(RED,GREEN,BLUE,DIM);
          }
      }
    
      // Setting the servos. After a servo is set to a new position there is a Servodelay delay for the servo to get there
      now=millis();
       if (now-start_pan > Servodelay) {
           panservo.write(map(PAN,0,255,0,180));
           start_pan=millis();
       }
       if (now-start_tilt > Servodelay) {
           tiltservo.write(map(TILT,0,255,0,180));
           start_tilt=millis();
       }  
} else {
    // In case of DMX timeout the movingheads will go to there default position (leds off, and servo in mid position)
    analogWrite(RED_out,0);  
    analogWrite(GREEN_out,0);
    analogWrite(BLUE_out,0);

      
     now=millis();
     if (now-start_pan > Servodelay) {
     panservo.write(90);
     start_pan=millis();
     }
     if (now-start_tilt > Servodelay) {
     tiltservo.write(90);
     start_tilt=millis();
     }   
  }
}

void OnFrameReceiveComplete (unsigned short channelsReceived)
{
  
  if ( channelsReceived == DMX_SLAVE_CHANNELS)
  {
    // All slave channels have been received
  }
  else
  {
    // We have received a frame but not all channels we where 
    // waiting for, master might have transmitted less
    // channels
  }

  // Update receive time to determine signal timeout
  lastFrameReceivedTime = millis ();
}

void InitialiseIO(){
  // Setting pins for the startadress dipswitchs
  pinMode(14, INPUT);	   // Pin A0 is input to which a switch is connected
  digitalWrite(14, HIGH);   // Configure internal pull-up resistor
  
  pinMode(15, INPUT);	   // Pin A1 is input to which a switch is connected
  digitalWrite(15, HIGH);   // Configure internal pull-up resistor
  
  pinMode(16, INPUT);	   // Pin A2 is input to which a switch is connected
  digitalWrite(16, HIGH);   // Configure internal pull-up resistor
  
  pinMode(17, INPUT);	   // Pin A3 is input to which a switch is connected
  digitalWrite(17, HIGH);   // Configure internal pull-up resistor
  
  pinMode(18, INPUT);	   // Pin A4 is input to which a switch is connected
  digitalWrite(18, HIGH);   // Configure internal pull-up resistor
  
  pinMode(19, INPUT);	   // Pin A5 is input to which a switch is connected
  digitalWrite(19, HIGH);   // Configure internal pull-up resistor
  
  pinMode(4, INPUT);	   // Pin 4 is input to which a switch is connected
  digitalWrite(4, HIGH);   // Configure internal pull-up resistor
  
  pinMode(7, INPUT);	   // Pin 7 is input to which a switch is connected
  digitalWrite(7, HIGH);   // Configure internal pull-up resistor
  
  pinMode(8, INPUT);	   // Pin 8 is input to which a switch is connected
  digitalWrite(8, HIGH);   // Configure internal pull-up resistor
}

// Calculation of startadress
int readDMXadress(){
  int DMXadress =  511-(digitalRead(8)+digitalRead(7)*2+digitalRead(4)*4+digitalRead(14)*8+digitalRead(15)*16+digitalRead(16)*32+digitalRead(17)*64+digitalRead(18)*128+digitalRead(19)*256);
  return DMXadress;
}

// Calculation and setting of PWM intensities
void PWM_intensities(int r,int g, int b, int dimmer){
      analogWrite(RED_out,r*dimmer/255);  
      analogWrite(GREEN_out,g*dimmer/255);
      analogWrite(BLUE_out,b*dimmer/255);
}
