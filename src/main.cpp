// feel free to use it as what you want to
// no warranty of course
// (C) ivanr.spiridonov@gmail.com

#include <Arduino.h>
#include <PrintEx.h>
#include <FastGPIO.h>

#include "../standard/pins_arduino.h" //enables A6 & A7
#undef NUM_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS 8

/*
Pins for ATmega328P boards for FastGPIO use Alternative name
Number | AVR pin macro | Alternative name
0	IO_D0	
1	IO_D1	
2	IO_D2	
3	IO_D3	
4	IO_D4	
5	IO_D5	
6	IO_D6	
7	IO_D7	
8	IO_B0	
9	IO_B1	
10	IO_B2	SS
11	IO_B3	MOSI
12	IO_B4	MISO
13	IO_B5	SCK
14	IO_C0	A0      //  Up
15	IO_C1	A1      //  Un  
16	IO_C2	A2      //  Vp
17	IO_C3	A3      //  Vn
18	IO_C4	A4, SDA //  Wp
19	IO_C5	A5, SCL //  Wn
20	IO_C6	

*/
//StreamEx mySerial = Serial; // this one rises errors
PrintEx mySerial = Serial;

// PINOUTS
#define INTELNAL_LED    13 //PB5
#define BYPASS_RELAY    12 //PB4
#define SYNCHRO_OUT     2  //PD2

#define U_p_OUT         14  //A0
#define U_n_OUT         15  //A1
#define V_p_OUT         16  //A2
#define V_n_OUT         17  //A3
#define W_p_OUT         18  //A4
#define W_n_OUT         19  //A5

#define OVERTEMP_IN     11 //PB3
#define FREQ_ANALOG_IN  6  //ADC6

#define PORTC_INIT_DIR  0b00111111 
#define PORTC_SET_ZERO  0b00000000 
#define PORTC_SET_ONE   0b00111111 

#define U_p_mask  0b00000001
#define U_n_mask  0b00000010
#define V_p_mask  0b00000100
#define V_n_mask  0b00001000
#define W_p_mask  0b00010000
#define W_n_mask  0b00100000

#define IGBT_SET_OFF    PORTC_SET_ZERO  

#define Fmax  40  
#define Fmin  1  

#define OUTBUFF_SIZE  1440

#define pot_min         1
#define CHARGE_TIME     2000  //2sec

bool  outEnabled  = 0;
bool  blinkBit  = 0;

static uint8_t  FREQ;
static uint16_t outBuffIdx  = 0;
static uint8_t outBuff[OUTBUFF_SIZE] ;
static uint8_t oldFREQ = 0;

uint64_t microsCntr = 0;


//timer0 interrupt -----------------------------------------------------------------------  
ISR(TIMER0_COMPA_vect)
{
  //FastGPIO::Pin<SYNCHRO_OUT>::setOutput(HIGH);

  if(!outEnabled)
  {
    PORTC = IGBT_SET_OFF;
  }else
    {
      PORTC = outBuff[outBuffIdx];
    }

  outBuffIdx++;
  if(outBuffIdx >= OUTBUFF_SIZE)
  {
    outBuffIdx = 0;
    FastGPIO::Pin<SYNCHRO_OUT>::setOutput(HIGH);
    FastGPIO::Pin<SYNCHRO_OUT>::setOutput(LOW);
  }

  //FastGPIO::Pin<SYNCHRO_OUT>::setOutput(LOW);
}

//timer01 interrupt-------------------------------------------------------------------------

void  doTestBuff()
{
  int outBuffMaxIdx = 0;   
  for(int c=0; c < 1440; c++)
  {
    outBuff[outBuffMaxIdx] = 0;

    if((c > 0 ) && (c < 240))
    {
      outBuff[outBuffMaxIdx] |= U_p_mask; 
    }
    if((c > 240) && (c <480))
    {
      outBuff[outBuffMaxIdx] |= U_n_mask; 
    }
    if((c > 480) && (c < 720))
    {
      outBuff[outBuffMaxIdx] |= V_p_mask; 
    }
    if((c > 720) && (c < 960))
    {
      outBuff[outBuffMaxIdx] |= V_n_mask; 
    }
    if((c > 960) && (c < 1200))
    {
      outBuff[outBuffMaxIdx] |= W_p_mask; 
    }
    if(c > 1200)
    {
      outBuff[outBuffMaxIdx] |= W_n_mask; 
    }

    outBuffMaxIdx ++; 
  }  
}

void  doBuffer()
{
  int outBuffMaxIdx = 0;  
    for(int c=0; c < 360; c++)          // do one sin cycle
    {
      for(int p=0; p < 4; p++)          // do pwm in the sin cycle
      {
        outBuff[outBuffMaxIdx] = 0;
        if(p  < 2)                  //  minPWM
        {
          if((c > 0) && (c < 180))
          {
            outBuff[outBuffMaxIdx] |= U_p_mask;  
          }
          if((c > 180) && (c < 360))
          {
            outBuff[outBuffMaxIdx] |= U_n_mask;  
          }

          if((c > 120) && (c < 300))
          {
            outBuff[outBuffMaxIdx] |= V_p_mask;  
          }
          if((c < 120) || (c > 300))
          {
            outBuff[outBuffMaxIdx] |= V_n_mask;  
          }

          if((c > 60) && (c < 240))
          {
            outBuff[outBuffMaxIdx] |= W_p_mask;  
          }
          if((c < 60) || (c > 240))
          {
            outBuff[outBuffMaxIdx] |= W_n_mask;  
          }
        }
       
        if(p  < 2)                  //  middle PWM
        {
          if(((c > 30) && (c <= 60)) || ((c >= 120) && (c < 150)))
          {
            outBuff[outBuffMaxIdx] |= U_p_mask;  
          }
          if(((c > 210) && (c <= 240)) || ((c >= 300) && (c < 330)))
          {
            outBuff[outBuffMaxIdx] |= U_n_mask;  
          }

          if(((c > 150) && (c <= 180)) || ((c >= 240) && (c < 270)))
          {
            outBuff[outBuffMaxIdx] |= V_p_mask;  
          }
          if(((c <= 90) && (c > 60)) || ((c < 359) && (c >= 330)))
          {
            outBuff[outBuffMaxIdx] |= V_n_mask;  
          }

          if(((c > 90) && (c <= 120)) || ((c >= 180) && (c < 210)))
          {
            outBuff[outBuffMaxIdx] |= W_p_mask;  
          }
          if(((c <= 30) && (c > 0)) || ((c < 300) && (c >= 270)))
          {
            outBuff[outBuffMaxIdx] |= W_n_mask;  
          }
        }
        if(p < 3)                   // maxPWM
        {
          if((c > 60) && (c < 120))
          {
            outBuff[outBuffMaxIdx] |= U_p_mask;  
          }
          if((c > 240) && (c < 300))
          {
            outBuff[outBuffMaxIdx] |= U_n_mask;  
          }

          if((c > 180) && (c < 240))
          {
            outBuff[outBuffMaxIdx] |= V_p_mask;  
          }
          if((c < 60) && (c > 0))
          {
            outBuff[outBuffMaxIdx] |= V_n_mask;  
          }

          if((c > 120) && (c < 180))
          {
            outBuff[outBuffMaxIdx] |= W_p_mask;  
          }
          if((c < 360) && (c > 300))
          {
            outBuff[outBuffMaxIdx] |= W_n_mask;  
          }
        }

        outBuffMaxIdx++;
      }
    }
}

void  printBuff()
{
 //only for debug 
#define BYTE_TO_BINARY_PATTERN "%c %c%c %c %c%c %c %c%c %c"
#define BYTE_TO_BINARY(byte ) \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  ((byte & 0x30) == 0x30 ? '^' : ' '),\
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  ((byte & 0x0C) == 0x0C ? '^' : ' '),\
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0'), \
  ((byte & 0x03) == 0x03 ? '^' : ' '), \
  (byte == 0x00 ? '*' : ' ')
  //SERIAL_PORT.printf("\r\n PM "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(m5_power_mode) );
  //DEBUG_PORT.printf("\r\n PM "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(m5_power_mode) );

  uint16_t  vector = 0;
  mySerial.printf("\r\n%03d: ", vector);
  for(uint16_t c=0; c < 1440; c++)
  {
     mySerial.printf(" "BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(outBuff[c]) );
     mySerial.printf(" %04d  |  ", c);
     if( !((c+1) % 4) ) 
     {
       vector++;
       mySerial.printf("\r\n%03d: ", vector);
     }
     yield(); 
  }
  mySerial.printf("\r\n");
}

void  SetUpTimer0Intr(uint8_t _timerOCR0A){
  cli();//stop interrupts
  //set timer0 interrupt at 100kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  OCR0A = _timerOCR0A;
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for prescaler
  TCCR0B |= (1 << CS01) ; //| (1 << CS00);  
  // Set CS00 without prescaler
  TCCR0B |= (1<<CS00); 
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();//allow interrupts
}

void setup() {
	Serial.begin(115200);
	delay(100);
  mySerial.printf("\r\nSerial is ready...\r\n");
  mySerial.printf("The compile time & date is: %s, %s\r\n", __TIME__, __DATE__);

  PORTC = IGBT_SET_OFF;
  DDRC = PORTC_INIT_DIR;
  PORTC = IGBT_SET_OFF;

  FastGPIO::Pin<BYPASS_RELAY>::setOutput(LOW);
  delay(CHARGE_TIME);
  FastGPIO::Pin<BYPASS_RELAY>::setOutput(HIGH);

  pinMode(OVERTEMP_IN, INPUT_PULLUP);

  doBuffer();
  //doTestBuff(); // for test purpuse only
  //printBuff();  // test only, to check the buffer for errors

  SetUpTimer0Intr( Fmin );


  FastGPIO::Pin<SYNCHRO_OUT>::setOutput(LOW);
}

uint16_t pot = 0;
void loop() {
  if(microsCntr > 30000 ) // print each 1 sec
  {
    microsCntr = 0;
    blinkBit = !blinkBit;

    pot = analogRead (FREQ_ANALOG_IN);
    FREQ = (uint8_t)map(pot, 0,1023,Fmin,Fmax);
    //FREQ = (uint8_t)map(pot, 0,1023,1,80);

    if(oldFREQ != FREQ)
    {
      SetUpTimer0Intr( FREQ );
      mySerial.printf("ADC:%d FREQ:%d \r\n", pot , FREQ );
    }

    oldFREQ = FREQ;

    if(digitalRead(OVERTEMP_IN)){
      outEnabled = 0;
      PORTC = IGBT_SET_OFF;
    } else{
      outEnabled = 1;
    }

    digitalRead(OVERTEMP_IN) ? FastGPIO::Pin<INTELNAL_LED>::setOutput(blinkBit) : FastGPIO::Pin<INTELNAL_LED>::setOutput(outEnabled);

    //mySerial.printf("ADC:%d FREQ:%d outBuffMaxIdx:%d\r\n", pot , FREQ, outBuffMaxIdx);
    //mySerial.printf("timerOCR0A:%d out_enabled:%d\r\n",timerOCR0A, outEnabled);
  } 
  microsCntr++;
}