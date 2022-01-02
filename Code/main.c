
#include <msp430g2553.h>
#include "clock.h"
#include "I2C.h"
#include "PCA9685.h"
#include "servo.h"

/**
 * main.c
 */
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#define UART_TXD   0x02                     // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD   0x04                     // RXD on P1.2 (Timer0_A.CCI1A)

//------------------------------------------------------------------------------
// Conditions for 9600 Baud SW UART, SMCLK = 16MHz
//------------------------------------------------------------------------------
#define UART_TBIT_DIV_2     833
#define UART_TBIT           1666

//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character
unsigned char a =0;
uint8_t i,j;
uint8_t Legs[] = { 0b000001, 0b100000 , 0b000010, 0b010000, 0b000100, 0b001000};
//uint8_t Legs[] = { 0b000001, 0b000010 , 0b000100, 0b001000, 0b010000, 0b100000};
//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);
void TimerA_UART(void);

uint16_t setServo(float pos, uint8_t leg)
{

    PCA9685_u16ChangeServoDegree_MG90(pos,leg);
    return 0;
}
void delay_ms(unsigned int ms)
{
    while (ms)
    {
        __delay_cycles(16000); //1000 for 1MHz and 16000 for 16MHz
        ms--;
    }
}

void setHipRaw(uint8_t leg, float pos) {
  setServo(pos, leg);
}

void setHip(uint8_t leg, float pos, uint8_t adj) {
    if (ISFRONTLEG(leg)) {
        pos -= adj;
      } else if (ISBACKLEG(leg)) {
        pos += adj;
      }
   if (leg >= LEFT_START) {
       pos =  - pos;

 }

  setHipRaw(leg, pos);
}
void setKnee(float pos, uint8_t leg) {
  // find the knee associated with leg if this is not already a knee
  if (leg < KNEE_OFFSET) {
    leg += KNEE_OFFSET;
  }
  setServo(pos, leg);
}
void setLeg(uint8_t legmask, float hip_pos, float knee_pos, uint8_t adj, uint8_t raw, float leanangle) {
  uint8_t i=0;
  for ( i = 0; i < NUM_LEGS; i++) {
      if (legmask & 0b1) {  // if the lowest bit is ON then we are moving this leg
        if (hip_pos != NOMOVE) {
          if (!raw) {
            setHip(i, hip_pos, adj);
          } else {
            setHipRaw(i, hip_pos);
          }
        }
        if (knee_pos != NOMOVE) {
          int pos = knee_pos;
          if (leanangle != 0) {
            switch (i) {
              case 0: case 6: case 5: case 11:
                if (leanangle < 0) pos -= leanangle;
                break;
              case 1: case 7: case 4: case 10:
                pos += abs(leanangle/2);
                break;
              case 2: case 8: case 3: case 9:
                if (leanangle > 0) pos += leanangle;
                break;
            }
          }

          setKnee(pos, i);
        }
      }
      legmask = (legmask>>1);  // shift down one bit position to check the next legmask bit
    }
}
void Servo_init(void)
{
    setLeg(ALL_LEGS, HIP_NEUTRAL , KNEE_NEUTRAL , 0, 1, 0);

}
void stand() {
    setLeg(ALL_LEGS, HIP_NEUTRAL,KNEE_STAND, 0, 1, 0);
}

void laydown() {

  setLeg(ALL_LEGS, HIP_NEUTRAL, 0, 0, 1, 0);
  delay_ms(10);
  setLeg(ALL_LEGS, HIP_NEUTRAL, 20, 0, 1, 0);
  delay_ms(10);
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_UP, 0, 1, 0);
}
void turn(uint8_t reverse, float hipforward, float hipbackward, float kneeup, float kneedown, float leanangle) {

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0, 0, leanangle);

      setLeg(TRIPOD2_LEGS, hipbackward - 10, NOMOVE, 40, 1, 0);
      setLeg(TRIPOD1_LEGS, hipforward + 10, NOMOVE, 40, 1, 0);

      delay_ms(200);

      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0, 0, leanangle);



      delay_ms(200);

      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0, 0, leanangle);

      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, 40, 1, 0);
      setLeg(TRIPOD2_LEGS, hipforward +10 , NOMOVE, 40, 1, 0);
      delay_ms(200);
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      delay_ms(100);
}
void gait_tripod(uint8_t reverse, float hipforward, float hipbackward, float kneeup, float kneedown, float leanangle) {

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }


      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0, 0, leanangle);

      setLeg(TRIPOD2_LEGS, hipbackward , NOMOVE, FBSHIFT, 0, 0);
      setLeg(TRIPOD1_LEGS, hipforward , NOMOVE, FBSHIFT, 0, 0);
      delay_ms(200);

      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0, 0, leanangle);

      delay_ms(200);

      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0, 0, leanangle);


      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT, 0, 0);
      setLeg(TRIPOD2_LEGS, hipforward , NOMOVE, FBSHIFT, 0, 0);

      delay_ms(200);

      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      delay_ms(100);
}
void typing()
{
    uint8_t i;
    setLeg(0b011110, 70, -90, 0, 0, 0);
    //delay_ms(10);

    for(i = 0; i < 8; i++ )
        {
        setLeg(FRONT_LEGS, 80, NOMOVE, FBSHIFT, 0, 0);
        delay_ms(50);
        setLeg(0b000001,NOMOVE, 20, FBSHIFT, 0, 0);
        setLeg(0b100000,NOMOVE, 70, FBSHIFT, 0, 0);
        delay_ms(50);
        setLeg(0b000001,NOMOVE, 70, FBSHIFT, 0, 0);
        setLeg(0b100000,NOMOVE, 20, FBSHIFT, 0, 0);
        delay_ms(50);
        if(i == 4)
        {
          setLeg(FRONT_LEGS, 10, NOMOVE, FBSHIFT, 0, 0);
          delay_ms(100);
        }
        }
        stand();
}
void dance()
{
      //setLeg(ALL_LEGS, NOMOVE, KNEE_STAND, 0, 0, 0);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, -60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 0);
      delay_ms(300);
}
void dance2()
{

      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, 90, 0, 0, 0);
      delay_ms(400);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, -60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 0);
      delay_ms(300);
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, -90, 0, 0, 0);
      delay_ms(600);

      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, 90, 0, 0, 0);
      delay_ms(400);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, -60);
      delay_ms(300);
      for ( i = 0; i < NUM_LEGS; i++)
      setHipRaw(i, 0);
      delay_ms(300);
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, -90, 0, 0, 0);
      delay_ms(600);
}
void foldup() {
  setLeg(ALL_LEGS, NOMOVE, KNEE_FOLD, 0, 1, 0);
  for (i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 60);
  dance();
}
void dance3()
{
    stand();
    for(i = 0; i < 6; i++)
    {
        setLeg(ALL_LEGS, NOMOVE, -40, 0, 0, 0);
        delay_ms(30);
        setLeg(ALL_LEGS, NOMOVE, KNEE_STAND, 0, 0, 0);
        delay_ms(40);
    }
}
void dance4()
{
    laydown();
    setLeg(LEFT_LEGS, NOMOVE, KNEE_UP, 0, 0, 0);
    delay_ms(30);
    setLeg(RIGHT_LEGS, NOMOVE, KNEE_UP, 0, 0, 0);
    delay_ms(30);
    stand();
}
void gait_ripple(uint8_t turn, uint8_t reverse, float hipforward, float hipbackward, float kneeup, float kneedown, uint8_t leanangle)
{


    if (reverse) {
      int tmp = hipforward;
      hipforward = hipbackward;
      hipbackward = tmp;
    }

    setLeg(ALL_LEGS, hipbackward, NOMOVE, FBSHIFT, turn,0);
    delay_ms(50);
    for(i = 0; i < 6; i++)
    {
        setLeg(Legs[i], NOMOVE, kneeup, 0, 0, 0);
        delay_ms(30);
        setLeg(Legs[i], hipforward, NOMOVE, FBSHIFT, turn, 0);
        delay_ms(30);
        setLeg(Legs[i], NOMOVE, kneedown, 0, 0, 0);
        delay_ms(20);
      }

}
void goForward_ripple(void)
{
    gait_ripple(0, 0, 40, -30, -30,-90, 0);
}
void goForward_tripod(void)
{
    gait_tripod( 0, 40, -30, -30, -90, 0);
}
void goBack_tripod(void)
{
    gait_tripod( 1, 30, -30, -40, -90, 0);
}
void goBack_ripple(void)
{
    gait_ripple(0, 1, 30, -30, -40,-90, 0);
}
void turnLeft_ripple(void)
{
    gait_ripple(1, 1, 30, -30, -40,-90,0);
}
void turnLeft_tripod(void)
{
    turn(1, 40, -20, -50, -90, 0);
}
void turnRight_ripple(void)
{
    gait_ripple(1, 0, 30, -30, -40,-90,0);
}
void turnRight_tripod(void)
{
    turn(0, 40, -20, -50, -90, 0);
}
void shrug()
{
    stand();
    delay_ms(50);
    setLeg(ALL_LEGS, NOMOVE, -60, 0, 0, 0);
    delay_ms(100);
    setLeg(ALL_LEGS, NOMOVE, -90, 0, 0, 0);
    delay_ms(100);
}
void seeSaw()
{

    stand();
    setLeg(MIDDLE_LEGS,30,NOMOVE, 0, 0, 0);
    delay_ms(10);
    for(i = 0; i < 5; i++)
    {
    setLeg(FRONT_LEGS, HIP_NEUTRAL, -90 , 0, 0, 0);
    setLeg(BACK_LEGS, HIP_NEUTRAL, 10 , 0, 0, 0);
    delay_ms(300);
    setLeg(FRONT_LEGS, HIP_NEUTRAL, 10 , 0, 0, 0);
    setLeg(BACK_LEGS, HIP_NEUTRAL, -90 , 0, 0, 0);
    delay_ms(300);
    }
    stand();


}

int main(void)

{
    WDTCTL = WDTPW | WDTHOLD; //stop watchdog timer
    Clock_vInit();
    TimerA_UART();
    I2C_vInitMaster(100000);
    _EINT();
    PCA9685_ChangeDevice(0x40);
    PCA9685_vInit();
    stand();

    while(1)
    {
        __bis_SR_register(LPM0_bits);
        while(rxBuffer == 0)
            {
            goForward_tripod();
            }
        while(rxBuffer == 10)
            {
            goForward_ripple();
            }
        while(rxBuffer == 1)
            {
            goBack_tripod();
            }
        while(rxBuffer == 11)
            {
            goBack_ripple();
            }
        while(rxBuffer == 2)
            {
            turnRight_tripod();
            }
        while(rxBuffer == 12)
            {
            turnRight_ripple();
            }
        while(rxBuffer == 3)
            {
            turnLeft_tripod();
            }
        while(rxBuffer == 13)
            {
            turnLeft_ripple();
            }
        while(rxBuffer == 4)
            {

            shrug();
            dance();
            }
        while(rxBuffer == 14)
            {
            seeSaw();
            }
        while(rxBuffer == 5)
            {
            dance2();
            }
        while(rxBuffer == 15)
            {
            typing();
            }
        while(rxBuffer == 6||rxBuffer == 16)
            {
               stand();
            }
        while(rxBuffer == 7)
            {
               dance3();
            }
        while(rxBuffer == 17)
            {
               //laydown();
               dance4();
            }

    }

}

void TimerA_UART(void)
{
    P1SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
            P1DIR = 0xFF & ~UART_RXD;               // Set all pins but RXD to output
            P2OUT = 0x00;
            P2SEL = 0x00;
            P2DIR = 0xFF;

            __enable_interrupt();

            TimerA_UART_init();                     // Start Timer_A UART
            TimerA_UART_print("G2xx2 TimerA UART\r\n");
            TimerA_UART_print("READY.\r\n");
}
void TimerA_UART_init(void)
{
    TACCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TACCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TACTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
    while (TACCTL0 & CCIE);                 // Ensure last char got TX'd
    TACCR0 = TAR;                           // Current state of TA counter
    TACCR0 += UART_TBIT;                    // One bit time till first bit
    TACCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
    txData = byte;                          // Load global variable
    txData |= 0x100;                        // Add mark stop bit to TXData
    txData <<= 1;                           // Add space start bit
}

//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char txBitCnt = 10;

    TACCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed?
        TACCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter
    }
    else {
        if (txData & 0x01) {
          TACCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TACCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;

    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TACCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TACCTL1 & CAP) {                 // Capture mode = start bit edge
                TACCTL1 &= ~CAP;                 // Switch capture to compare mode
                TACCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TACCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed?
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
                    TACCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}

