/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/* WDT operating mode -> WDT Disabled */
#pragma config WDTE = OFF
/* Low voltage programming enabled, RE3 pin is MCLR */
#pragma config LVP = ON

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ                      31000UL

/* RE2 button pin */
#define Button_Read()                   (PORTEbits.RE2)
/* Button is active in low-logic level  */
#define BUTTON_PRESSED                  false
/* Long-press button is 1.5 sec (maximum 2.55 sec) */
#define BUTTON_LONG_PRESS_THRESHOLD     1500
/* Time is expressed in ms */
#define BUTTON_TIME_STEP                  10

#define TIMER_PRESCALER                   32
#define FREQUENCY_TO_PR_CONVERT(F)      (uint8_t)(((_XTAL_FREQ)/ \
                                        (4*(F))/(TIMER_PRESCALER))-1)
#define DUTYCYCLE_TO_CCPR_CONVERT(D,F)  (uint16_t)((float)(D)*(((_XTAL_FREQ)/ \
                                        (F)/(TIMER_PRESCALER))-1)/100.0)

/* Hz */
#define FREQUENCY_MAX                      4
#define FREQUENCY_MIN                      1
#define FREQUENCY_STEP                     1
/* percents */
#define DUTYCYCLE_MAX                     75
#define DUTYCYCLE_MIN                     25
#define DUTYCYCLE_STEP                    25


typedef enum{
    BT_NOCHANGE,
    BT_SHORT_PRESS,
    BT_LONG_PRESS
} button_t;

static button_t ButtonCheck(void);
static void     PORT_Initialize(void);
static void     PPS_Initialize(void);
static void     CLK_Initialize(void);
static void     PWM1_Initialize(void);
static void     TMR2_Initialize(void);
static void     PWM1_LoadDutyValue(uint16_t duty);
static void     TMR2_LoadPeriodRegister(uint8_t period);

static button_t ButtonCheck(void)
{
    button_t result = BT_NOCHANGE;
    uint8_t counter = 0;
    static bool old_button_state = !BUTTON_PRESSED;
    bool button_state = Button_Read();
    /* detecting only the button-press event */
    if( (button_state == BUTTON_PRESSED) && (old_button_state != BUTTON_PRESSED) )
    {
        /*  wait for debouncing time */
        __delay_ms(BUTTON_TIME_STEP);
        while( (Button_Read() == BUTTON_PRESSED) && \
                (counter < (BUTTON_LONG_PRESS_THRESHOLD/BUTTON_TIME_STEP)) )
        {
            /* then stay in the loop until either */
            /* is button released or long-press encountered*/
            counter++;
            __delay_ms(BUTTON_TIME_STEP);
        }
        if(counter)
        {
            result = BT_SHORT_PRESS;
            if(counter >= (BUTTON_LONG_PRESS_THRESHOLD/BUTTON_TIME_STEP))
                result = BT_LONG_PRESS;
        }
    }
    old_button_state = button_state;
    return result;
}

static void PORT_Initialize(void)
{
    /* RB4 is output for PWM1 */
    TRISBbits.TRISB4 = 0;
    /* RE2 is digital input with pull-up for push-button */
    TRISEbits.TRISE2 = 1;
    ANSELEbits.ANSELE2 = 0;
    WPUEbits.WPUE2 = 1;
}   
    
static void PPS_Initialize(void)
{
    /* Configure RB4 for PWM1 output */
    RB4PPS = 0x05;
}

static void CLK_Initialize(void)
{
    /* Configure NOSC LFINTOSC; NDIV 1 FOSC = 31kHz */
    OSCCON1bits.NOSC = 5;
    OSCCON1bits.NDIV = 0;
}

static void PWM1_Initialize(void)
{
	/* MODE PWM; EN enabled; FMT left_aligned */
    CCP1CONbits.MODE = 0x0C;
    CCP1CONbits.FMT = 1;
    CCP1CONbits.EN = 1;

	/* Selecting Timer 2 */
    CCPTMRSbits.C1TSEL = 1;

    /* Configure the default duty cycle */
    CCPR1 = (DUTYCYCLE_TO_CCPR_CONVERT(DUTYCYCLE_MIN, FREQUENCY_MIN)) << 6;
}

static void TMR2_Initialize(void)
{
    /* TIMER2 clock source is FOSC/4 */
    T2CLKCONbits.CS = 1;

    /* TIMER2 counter reset */
    T2TMR = 0x00;

    /* TIMER2 ON, prescaler 1:32, postscaler 1:1 */
    T2CONbits.OUTPS = 0;
    T2CONbits.CKPS = 5;
    T2CONbits.ON = 1;

    /* Configure the default period */
    PR2 = FREQUENCY_TO_PR_CONVERT(FREQUENCY_MIN);
}

static   void PWM1_LoadDutyValue(uint16_t dutyValue)
{
    /* Only the upper 10 bits are used in register CCPR1 */
    CCPR1 = dutyValue << 6;
}

static  void TMR2_LoadPeriodRegister(uint8_t periodVal)
{
    /* Configure the period register */
    PR2 = periodVal;
}

void main(void)
{
    uint8_t frequency  = FREQUENCY_MIN;  /* expressed in Hz */
    uint8_t duty_cycle = DUTYCYCLE_MIN;  /* expressed in percents */
    /* Initialize the device */
    PORT_Initialize();
    PPS_Initialize();
    CLK_Initialize();
    PWM1_Initialize();
    TMR2_Initialize();

    while (1)
    {
        button_t button_status = ButtonCheck();
        if(button_status == BT_SHORT_PRESS)
        {
            /* when short button press is detected */
            /* change the duty cycle (25%, 50% or 75%) */
            duty_cycle += DUTYCYCLE_STEP;
            if(duty_cycle > DUTYCYCLE_MAX)
                duty_cycle = DUTYCYCLE_MIN;

            /* update the duty cycle */
            PWM1_LoadDutyValue(DUTYCYCLE_TO_CCPR_CONVERT(duty_cycle, frequency));
        }
        else if(button_status == BT_LONG_PRESS)
        {
            /* when long button press is detected */
            /* change the frequency (1, 2, 3 or 4 Hz) */
            frequency += FREQUENCY_STEP;
            if(frequency > FREQUENCY_MAX)
                frequency = FREQUENCY_MIN;

            /* update the duty cycle and frequency */
            PWM1_LoadDutyValue(DUTYCYCLE_TO_CCPR_CONVERT(duty_cycle, frequency));
            TMR2_LoadPeriodRegister(FREQUENCY_TO_PR_CONVERT(frequency));
        }
    }
}

