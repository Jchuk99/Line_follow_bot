// LineFollowFSMmain.c
// Runs on MSP432
// Simple line follower with 2 inputs and 2 outputs.
// Rather than real sensors and motors, it uses LaunchPad I/O
// Daniel and Jonathan Valvano
// January 24, 2018

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"

/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
 */

// Linked data structure
struct State {
  uint8_t out;                // 2-bit output
  uint16_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center &fsm[0]
#define Left   &fsm[1]
#define Right  &fsm[2]
State_t fsm[3]={
  {0x03, 500, { Right, Left,   Right,  Center }},  // Center
  {0x02, 500, { Left,  Center, Right,  Center }},  // Left
  {0x01, 500, { Right, Left,   Center, Center }}   // Right
};
State_t *Spt;  // pointer to the current state
uint8_t bump_sensor_activated;
uint8_t reflectance_input;
uint8_t motor_output;

/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void){ uint32_t heart=0;
  Clock_Init48MHz();
  LaunchPad_Init();
  TExaS_Init(LOGICANALYZER);  // optional
  bump_sensor_activated = 0;
  reflectance_input = 0;
  motor_output = 0;

  Spt = Center;
  while(1){
    motor_output = Spt->out;            // set output from FSM
    // first transform reflectance_input from 64 conditions to ~8 conditions?
    // combine reflectance input and bump_sensor_activated into one input for the FSM
    // which will stop if you activate any of the bump sensors
    Spt = Spt->next[reflectance_input]; // next depends on input and state
    }
  }
}

// Color    LED(s) Port2
// dark     ---    0
// red      R--    0x01
// blue     --B    0x04
// green    -G-    0x02
// yellow   RG-    0x03
// sky blue -GB    0x06
// white    RGB    0x07
// pink     R-B    0x05
