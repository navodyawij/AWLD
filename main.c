#define _XTAL_FREQ 16000000

#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7

#define BUZZER_PIN   RB5
#define SERVO_PIN RB6 // Define the pin connected to the servo
#define SERVO_POSITION_LOW 30 // Position for low water level
#define SERVO_POSITION_HIGH 120 // Position for high water level

#define PRESCALAR 16

#include <xc.h>
#include "lcd.h";
#include <pic16f877a.h>
#include <stdio.h>

// BEGIN CONFIG
#pragma config FOSC = HS 
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config BOREN = ON 
#pragma config LVP = OFF 
#pragma config CPD = OFF 
#pragma config WRT = OFF 
#pragma config CP = OFF
//END CONFIG

volatile int a;
volatile int w;

int ui = 0;

float PWM_FREQ;
float PWM_PERIOD;

long PWM_freq = 50;

void __init_pwm() {
    TRISCbits.TRISC2 = 0; // set RC2 as output
    T2CONbits.TMR2ON = 1; // set timer 2 on
    T2CONbits.T2CKPS = 0b10; // set pre scalar of 16
    CCP1CONbits.CCP1M = 0b1100; // set PWM mode of operation
}

void __set_pwm_freq(int f) {
    PWM_FREQ = f;
    PWM_PERIOD = 1 / PWM_FREQ;
    int PR2_value = ((PWM_PERIOD * _XTAL_FREQ) / (4 * PRESCALAR) - 1);
    PR2 = PR2_value;
}

void __set_duty_cycle(int duty_cycle) {
    float dc = (float) duty_cycle / 100;
    float dc_period = dc * PWM_PERIOD; 
    int reg_value = (int) ((dc_period * _XTAL_FREQ) / PRESCALAR);
    /*
     * PWM has 10 bit resolution
     * 8 bits of MSB is stored in CCPR1L
     * 2 bits of LSB is stored in CCP1CON(5:4)
     */
    CCPR1L = reg_value >> 2; 
    CCP1CONbits.CCP1X = (reg_value & 0b00000001);
    CCP1CONbits.CCP1Y = (reg_value & 0b00000010);
}


void __interrupt() echo()
{
  if(RBIF == 1)                       //Makes sure that it is PORTB On-Change Interrupt
  {
    RBIE = 0;                         //Disable On-Change Interrupt
    if(RB4 == 1)                      //If ECHO is HIGH
    TMR1ON = 1;                       //Start Timer
    if(RB4 == 0)                      //If ECHO is LOW
    {
      TMR1ON = 0;                     //Stop Timer
      a = (TMR1L | (TMR1H<<8))/117.64; //Calculate Distance
    }
  }
  RBIF = 0;                           //Clear PORTB On-Change Interrupt flag
  RBIE = 1;                           //Enable PORTB On-Change Interrupt
}

void sendDistance(int distance)
{
    char buffer[10];                   // Buffer to hold the distance string
    sprintf(buffer, "#UR %d cm\r\n", distance); // Format the distance as a string

    for (int i = 0; buffer[i] != '\0'; i++) // Send each character
    {
        while (!TXSTAbits.TRMT);       // Wait until the transmit buffer is empty
        TXREG = buffer[i];             // Transmit the character
    }
}


void showDistance()
{
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("     (1)");
    Lcd_Set_Cursor(2,1);
    //a = a / 2; // Adjust for the round trip
    
    sendDistance(a);
    Lcd_Write_String("Reading = ");
    Lcd_Set_Cursor(2,14);
    Lcd_Write_Char(a % 10 + 48);

    a = a / 10;
    Lcd_Set_Cursor(2,13);
    Lcd_Write_Char(a % 10 + 48);

    a = a / 10;
    Lcd_Set_Cursor(2,12);
    Lcd_Write_Char(a % 10 + 48);

    Lcd_Set_Cursor(2,15);
    Lcd_Write_String("cm");
}

void showWaterLevel()
{
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("     (2)");
    Lcd_Set_Cursor(2,1);
    //a = a / 2; // Adjust for the round trip
    w = 300 - a;
    sendDistance(a);
    Lcd_Write_String("Level = ");
    Lcd_Set_Cursor(2,14);
    Lcd_Write_Char(w % 10 + 48);

    w = w / 10;
    Lcd_Set_Cursor(2,13);
    Lcd_Write_Char(w % 10 + 48);

    w = w / 10;
    Lcd_Set_Cursor(2,12);
    Lcd_Write_Char(w % 10 + 48);

    Lcd_Set_Cursor(2,15);
    Lcd_Write_String("cm");
}

void moveServo(int position)
{
    if (position == 1) // Move to low position
    {
        __set_duty_cycle(0); // Set duty cycle for low position
    }
    else if (position == 2) // Move to high position
    {
        __set_duty_cycle(90);  // Set duty cycle for high position
    }
}

void main()
{
  TRISB = 0b01111000;                 //RB4 as Input PIN (ECHO)
  TRISD = 0x00;  // LCD Pins as Output
  OPTION_REGbits.nRBPU = 0;
  PORTBbits.RB1 = 1;
  PORTBbits.RB2 = 1;
  PORTBbits.RB3 = 1;
  PORTBbits.RB5 = 1;
  GIE = 1;                            //Global Interrupt Enable
  RBIF = 0;                           //Clear PORTB On-Change Interrupt Flag
  RBIE = 1;                           //Enable PORTB On-Change Interrupt
  
// servo
  __init_pwm();

  __set_pwm_freq(50);
  __set_duty_cycle(0);
    
  // Initialize USART
    SPBRG = 103;                         // Set baud rate to 9600 (Fosc = 4MHz)
    TXSTAbits.SYNC = 0;                // Asynchronous mode
    TXSTAbits.BRGH = 1;                // High speed baud rate
    RCSTAbits.SPEN = 1;                // Enable serial port
    TXSTAbits.TXEN = 1;                // Enable transmitter

    GIE = 1;                            // Global Interrupt Enable
    RBIF = 0;                           // Clear PORTB On-Change Interrupt Flag
    RBIE = 1;                           // Enable PORTB On-Change Interrupt
  
  Lcd_Init();

  Lcd_Set_Cursor(1,1);
  Lcd_Write_String("Water Level");
  Lcd_Set_Cursor(2,1);
  Lcd_Write_String("Monitor");

  __delay_ms(3000);
  Lcd_Clear();

  T1CON = 0x10;                       //Initialize Timer Module

  while(1)
  {
    TMR1H = 0;                        //Sets the Initial Value of Timer
    TMR1L = 0;                        //Sets the Initial Value of Timer
    
    RB0 = 1;                          //TRIGGER HIGH
    __delay_us(10);                   //10uS Delay
    RB0 = 0;                          //TRIGGER LOW 

    __delay_ms(100);                  //Waiting for ECHO
    a = a + 1; //Error Correction Constant
    
    if (RB3 == 0) {
            __delay_ms(10);
            if (RB3 == 0) {
                ui = 0;
            }
        }

    if (RB2 == 0) {
        __delay_ms(10);
        if (RB2 == 0) {
            ui = 1;
        }
    }
    
    if(a>=2 && a<=400)                //Check whether the result is valid or not
    {
        if (a <= 10) // Check if water level is too high
        {
          __set_duty_cycle(30); // Move servo to high position
        }
         else
        {
          __set_duty_cycle(80); // Move servo to low position
        }
        if(ui == 0)
        {
           showWaterLevel(); 
        }
        if(ui == 1)
        {
            showDistance();
        }
      //showDistance();
        //showWaterLevel();
    }
    else
    {
      Lcd_Clear();
      Lcd_Set_Cursor(1,1);
      Lcd_Write_String("Out of Range");
    }
    __delay_ms(400);
  }
}