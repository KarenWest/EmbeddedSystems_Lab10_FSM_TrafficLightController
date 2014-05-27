// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// November 7, 2013

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
// west facing red light connected to PB5
// west facing yellow light connected to PB4
// west facing red light connected to PB3
// south facing green light connected to PB2
// south facing yellow light connected to PB1
// south facing red light connected to PB0
// pedestrian / walker - WALK green light connected to PF3
// pedestrian / walker - DONT WALK - SOLID red light connected to PF1
// pedestrian / walker - HURRY WALK - FLASH red light ON and OFF TWICE connected to PF1
// pedestrian / walker present on either west or south road sensor detector connected to PE2 (1=walker present)
// west facing car detector connected to PE1 (1=car present)
// south facing car detector connected to PE0 (1=car present)


#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002401C)) // bits 2-0
#define SENSOR                  (*((volatile unsigned long *)0x4002401C))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control


// Linked data structure
struct State {
  unsigned long Out; 
  unsigned long Time;  
  unsigned long Next[8];}; 
typedef const struct State STyp;
#define goWest   0
#define waitWest 1
#define goSouth 2
#define waitSouth 3
#define goWalk   4
#define dontWalk1 5
#define walkOff1  6
#define dontWalk2  7
#define walkOff2  8
	
// Initialize SysTick with busy wait running at bus clock.
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))


STyp FSM[9]={
{0x31,50,{goWest, goWest, waitWest, waitWest, waitWest, waitWest, waitWest, waitWest }}, //0
{0x51,10,{goSouth, goSouth, goSouth, goSouth, goWalk, goWalk, goWalk, goSouth }}, //1
{0x85,50,{goSouth, waitSouth, goSouth, waitSouth, waitSouth, waitSouth, waitSouth, waitSouth }}, //2
{0x89,10,{goWest, goWest, goWest, goWest, goWalk, goWalk, goWalk, goWalk }}, //3
{0x92,50,{goWalk, dontWalk1, dontWalk1, dontWalk1, goWalk, dontWalk1, dontWalk1, dontWalk1 }}, //4
{0x91,10,{walkOff1, walkOff1, walkOff1, walkOff1, walkOff1, walkOff1, walkOff1, walkOff1 }}, //5
{0x90,10,{dontWalk2, dontWalk2, dontWalk2, dontWalk2, dontWalk2, dontWalk2, dontWalk2, dontWalk2 }}, //6
{0x91,10,{walkOff2, walkOff2, walkOff2, walkOff2, walkOff2, walkOff2, walkOff2, walkOff2 }}, //7
{0x90,10,{goWest, goWest, goSouth, goWest, goWest, goWest, goSouth, goWest }}}; //8 


unsigned long S;  // index to the current state 
unsigned long Input; 

int main(void){ 
	volatile unsigned long delay;
	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
  PLL_Init();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
  SYSCTL_RCGC2_R |= 0x32;      // 1) F B E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 8) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R &= ~0x0A; 			// 9) disable analog function on PF3 abd PF1
  GPIO_PORTF_PCTL_R = 0x00000000;   // 10) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x0A;          // 11) PF3 and PF1 outputs
  GPIO_PORTF_AFSEL_R &= ~0x0A; 			// 12) regular function on PF3 and PF1
  //GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R |= 0x0A;    			// 13) enable digital on PF3 and PF1

  S = goWest; //initial state 
  EnableInterrupts();
  while(1){
		//Moore machine - output based on current state
    LIGHT = FSM[S].Out >> 2;  // set west and south road traffic LED lights (PB5-0)
		GPIO_PORTF_DATA_R = ((FSM[S].Out & 0x2) << 2) | ((FSM[S].Out & 0x1) << 1); // set walk/don't walk leds (PF3 and PF1) 
	  //wait for time relevant to state
    SysTick_Wait10ms(FSM[S].Time);
		//get input sensors for cars (one for west rd, one for south rd) and one for pedestrian
    Input = SENSOR;     // read sensors (SENSOR defines to read bits PE2-0 ---no need to shift right 2 bits defined this way)
		//Moore machine - next state based on Input and current state
		S = FSM[S].Next[Input]; 
  } 
  
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 10000us equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

