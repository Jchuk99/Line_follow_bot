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

#include "msp.h"
#include "../inc/Motor.h"
#include "../inc/Clock.h"
#include "..\inc\Reflectance.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"


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

uint32_t TIME;
uint8_t reflectance_start;

// Linked data structure
struct State {
  uint8_t out;                // 2-bit output
  uint8_t delay;              // time to delay in 1ms can only delay up to 255
  const struct State *next[6]; // Next if 2-bit input is 0-3
};

typedef const struct State State_t;
#define POS_CENTER 0
#define POS_LEFT 1
#define POS_SLIGHT_LEFT 2
#define POS_RIGHT 3
#define POS_SLIGHT_RIGHT 4
#define POS_LOST 5

#define Center &fsm[0]
#define SlightLeft   &fsm[1]
#define Left   &fsm[2]
#define SlightRight   &fsm[3]
#define Right  &fsm[4]
#define BufferCenter &fsm[5]
#define BufferCenter2 &fsm[6]
#define OffCenter &fsm[7]
#define OffLeft &fsm[8]
#define OffLeft2 &fsm[9]
#define OffLeft3 &fsm[10]
#define OffRight &fsm[11]
#define OffRight2 &fsm[12]
#define OffRight3 &fsm[13]
#define Stop  &fsm[14]
#define InitCenter &fsm[15]

State_t fsm[16]={
                //center, left, slightlight, right, slightright, lost
  //real output of center is 0x03(drive forward), 0x00 for testing
  {0x03, 50, { Center, Left,  SlightLeft, Right, SlightRight, BufferCenter}},  // Center
  {0x0B, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffLeft}},  // SlightLeft
  {0x02, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffLeft}},   // Left
  {0x13, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffRight}},   // SlightRight
  {0x01, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffRight}},   // Right
  {0x03, 50, { Center, Left,  SlightLeft, Right, SlightRight, BufferCenter2}},  // BufferCenter
  {0x03, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffCenter}},  // BufferCenter2
  {0x03, 50, { Center, Left,  SlightLeft, Right, SlightRight, Stop}},   // OffCenter
  {0x0A, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffLeft2}}, // OffLeft
  {0x0A, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffLeft3}}, // OffLeft2
  {0x0A, 50, { Center, Left,  SlightLeft, Right, SlightRight, BufferCenter2}}, // OffLeft3
  {0x09, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffRight2}},   // OffRight
  {0x09, 50, { Center, Left,  SlightLeft, Right, SlightRight, OffRight3}},   // OffRight2
  {0x09, 50, { Center, Left,  SlightLeft, Right, SlightRight, BufferCenter2}},   // OffRight3
  {0x00, 250, { Stop, Stop,  Stop, Stop, Stop, Stop}},   // Stop
  {0x1B, 250, { Center, Left,  SlightLeft, Right, SlightRight, BufferCenter}},  // initCenter

};

State_t *Spt;  // pointer to the current state
uint8_t bump_sensor_activated;
uint8_t reflect_in;
uint8_t fsm_in;

void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
    TIME = TIME + 1;

    if(reflectance_start){
        reflect_in = Reflectance_End();
        reflectance_start = 0;
    }
    else{
        if(TIME % 9 == 0){
            Reflectance_Start();
            reflectance_start = 1;
        }
    }
}



/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
void get_next_state(void){
    if(reflect_in == 0x18 ||reflect_in == 0xFF || reflect_in == 0x3C || reflect_in == 0x7E){
        fsm_in = POS_CENTER;
    }
    else if((reflect_in >= 0x08 && reflect_in < 0x0D)){
        fsm_in = POS_SLIGHT_LEFT;
        //0110 0000
    }
    else if((reflect_in >= 0x01 && reflect_in <= 0x07) || (reflect_in >= 0x0D && reflect_in <= 0x0F)){
        fsm_in = POS_LEFT;
    }
    else if(reflect_in >= 0x10 && reflect_in < 0x80){
        fsm_in = POS_SLIGHT_RIGHT;
    }
    else if (reflect_in >= 0x80 && reflect_in <= 0xF0){
        fsm_in = POS_RIGHT;
    }
    else if (reflect_in == 0x00){
        fsm_in = POS_LOST;
    }
    else{
        // default
        fsm_in = POS_CENTER;
    }
}
int main(void){
  Clock_Init48MHz();
  Motor_Init();
  Reflectance_Init();
  SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  EnableInterrupts();
  bump_sensor_activated = 0;
  Spt = Center;

  while(1){
    Read_Command(Spt->out);            // set output from FSM
    Clock_Delay1ms(Spt->delay);   // wait
    // first transform reflectance_input from 64 conditions to ~8 conditions?
    get_next_state();
    Spt = Spt->next[fsm_in]; // next depends on input and state
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
