/* 
 * Project  : RC circuit - PI compensator
 * Author   : O. DJADANE
 * Updated  : 2021/08/04
 * Processor: dsPIC33EV32GM102
 * Compiler : MPLAB XC16
 */

#define FOSC	(79227500ULL)	// 7.37 MHz * 43/(2*2) = 79.2 MHz
#define FCY		(39613750ULL)	// FOSC / 2 = 39.6 MHz

#include <xc.h>
#include <stdbool.h>            // bool
#include <stdint.h>             // uint8_t, uint16_t
#include <string.h>             // strtok, strcpy
#include <stdlib.h>             // atof, utoa

#pragma config POSCMD = NONE    // Primary Oscillator : disabled
#pragma config FNOSC = FRC      // Oscillator mode : FRC
#pragma config FCKSM = CSECMD	// Clock switch: enabled / FSCM: disabled
#pragma config IESO = OFF       // Two speed oscillator : disabled
#pragma config FWDTEN = OFF     // Watchdog Timer : disabled
#pragma config OSCIOFNC = ON    // RA3/OSC2 : general purpose I/O
#pragma config ICS = PGD3       // ICD Communication Channel: PGEC3/PGED3

// Variables/Defines - Sampling period
#define PR3VAL 6187             // 64*6188 / 39.6MHz = 10 ms

// Variables/Defines - PWM frequency
#define PR2VAL 39599            // 39.6MHz / (1 * (39599+1)) = 1 kHz

// Variables/Defines - UART
#define BRGVAL 256              // 39.6MHz / (4 * 38400) - 1
#define STR_IN_MAX 40
#define STR_OUT_MAX 20
char stringReceived[STR_IN_MAX];
char stringSent[STR_OUT_MAX];
char stringParseMode[3];
char stringParseDelim[1] = "/";
char *ptrStringSent;
volatile unsigned char charReceived;
volatile uint8_t indexParse = 0;
volatile bool isSerialReceiving = false;

// Variables/Defines - ADC
volatile uint16_t adcValue = 0;
volatile float adcVolt = 0;

// Variables/Defines - PID
#define SAMPLING_TIME 0.01
typedef struct{
    float kp, ki, kd, Ts, sp, pv, err, errSum, errDiff, errPrev, command;
} T_PID;
T_PID PID = {0, 0, 0, SAMPLING_TIME, 0, 0, 0, 0, 0, 0, 0};
float inputOpenLoop = 0;
bool isClosedLoop = false;
bool isTimeToRunCommand = false;

// Prototypes
void runCommand();
float bound(float, float, float);
void resetErrorsInPID();
void receiveDataWithMarkers();
void parseReceivedData();
void sendMeasurement();
void formatString();
void floatTwoDecimalsToString(char*, float);

int main() {
    /* Setup PLL for 40MIPS */
	CLKDIVbits.PLLPRE = 0;          // PLL pre-scaler  (N1)
    CLKDIVbits.PLLPOST = 0;         // PLL post-scaler (N2)
    PLLFBDbits.PLLDIV = 41;         // PLL multiplier  (M)
	__builtin_write_OSCCONH(0x01);	// Initiate clock switch to...
	__builtin_write_OSCCONL(0x01);	// ...FRC with PLL (NOSC = 1)
	while (OSCCONbits.COSC != 1);	// Wait for clock switching
	while (OSCCONbits.LOCK != 1);	// Wait for PLL to lock
    
    /* Setup UART for 38400 bps, using RB5 for TX / RB2 for RX */
    RPOR0bits.RP35R = 1;    // PPS output: RP35/RB5 -> U1TX
    TRISBbits.TRISB2 = 1;   // RB2: input
    ANSELBbits.ANSB2 = 0;   // RB2: digital
    RPINR18bits.U1RXR = 34; // PPS input: RP34/RB2 -> U1RX
    U1MODEbits.UARTEN = 0;  // Disable UART module
    U1MODEbits.STSEL = 0;   // Stop bit: 1
    U1MODEbits.PDSEL = 0;   // 8 bits, no Parity
    U1MODEbits.ABAUD = 0;   // Auto-Baud: disabled
    U1MODEbits.BRGH = 1;    // High Baud rate: enabled (4x baud clock)
    U1BRG = BRGVAL;         // Baud rate value
    U1MODEbits.UARTEN = 1;  // Enable UART
    U1STAbits.UTXEN = 1;    // U1TX is now controlled by UART1
    U1STAbits.URXISEL = 0;  // Interrupt after one character is received
    IFS0bits.U1RXIF = 0;    // Clear RX interrupt flag
    IEC0bits.U1RXIE = 1;    // Enable RX interrupt
    
    /* Setup Timer3 for 10 ms */
    T3CONbits.TON = 0;      // Disable Timer3
    T3CONbits.TCKPS = 2;    // Pre-scaler : 1:64
    T3CONbits.TCS = 0;      // Clock source: internal (FCY)
    T3CONbits.TGATE = 0;    // Gated timer mode: disabled
    TMR3 = 0;               // Clear timer register
    PR3 = PR3VAL;           // Period value
    IFS0bits.T3IF = 0;      // Clear Timer3 interrupt flag
    IEC0bits.T3IE = 1;      // Enable Timer3 interrupt
    T3CONbits.TON = 1;      // Enable Timer3
    
    /* Setup ADC for 10-bit resolution, using RB7/AN25 pin */
    TRISBbits.TRISB7 = 1;   // RB7: input
    ANSELBbits.ANSB7 = 1;   // RB7: analog
    AD1CON1bits.ADON = 0;   // Disable ADC module
    AD1CON2bits.VCFG = 0;   // Voltage reference: AVDD / AVSS
    AD1CON3bits.ADCS = 3;   // Conversion clock: TAD=(3+1)*TCY (4/39.6M > 75n)
    AD1CON1bits.SSRC = 2;   // Clock source to start conversion: Timer3
    AD1CON1bits.ASAM = 1;   // Sample auto-start: immediately after conversion
    AD1CON2bits.CHPS = 0;   // Channels utilized: CH0
    AD1CHS0bits.CH0SA = 25; // CH0 +ve input: AN25/RB7
    AD1CHS0bits.CH0NA = 0;  // CH0 -ve input: VREFL
    IFS0bits.AD1IF = 0;     // Clear ADC1 interrupt flag
    IEC0bits.AD1IE = 1;     // Enable ADC1 interrupt
    AD1CON1bits.ADON = 1;   // Enable ADC module

    /* Setup Timer2 for 1 kHz PWM */
    T2CONbits.TON = 0;      // Disable Timer
    T2CONbits.TCS = 0;      // Clock source: internal (FCY)
    T2CONbits.TGATE = 0;    // Gated timer mode: disabled
    T2CONbits.TCKPS = 0;    // Pre-scaler: 1:1
    TMR2 = 0;               // Clear timer register
    PR2 = PR2VAL;           // Load the period value
    T2CONbits.TON = 1;      // Enable Timer2
    
    /* Setup Output Compare for Timer2, using RB10 pin */
    RPOR4bits.RP42R = 16;   // PPS : OC1 -> RB10/RP42
    OC1CON1bits.OCM = 0;    // Disable Output Compare Module
    OC1R = 1;               // Duty cycle for the first PWM pulse
    OC1RS = 1;              // Duty cycle for the second PWM pulse
    OC1CON1bits.OCTSEL = 0; // Clock select: Timer2
    OC1CON1bits.OCM = 7;    // Mode: Edge-aligned PWM
    
    /* Main loop */
    while (true) {
        if (isTimeToRunCommand) {
            isTimeToRunCommand = false;
			// ADC conversion is automatically triggered by the timer
            runCommand();
            sendMeasurement();
        }
    }
    return 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    isTimeToRunCommand = true;
    IFS0bits.T3IF = 0;
}

void __attribute__ ((__interrupt__, no_auto_psv)) _AD1Interrupt(void) {
    adcValue = ADC1BUF0;
    adcVolt = adcValue * 5.0 / 1023.0;
    IFS0bits.AD1IF  = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){
    charReceived = U1RXREG;
    receiveDataWithMarkers();
    IFS0bits.U1RXIF = 0;
}

void runCommand() {
    /* Computes the duty cycle of the PWM output according to the control law */
    if (isClosedLoop) {
        PID.pv = adcVolt;
        PID.err = PID.sp - PID.pv;
        PID.errSum += PID.err * PID.Ts;
        PID.errDiff = (PID.err - PID.errPrev) / PID.Ts;
        PID.errPrev = PID.err;
        PID.command = PID.kp*PID.err + PID.ki*PID.errSum + PID.kd*PID.errDiff;
        OC1RS = (uint16_t) bound(PID.command * (PR2VAL/5.0), 2, PR2VAL);
    } else {
        OC1RS = (uint16_t) bound(inputOpenLoop * (PR2VAL/5.0), 2, PR2VAL);
    }
}

float bound(float value, float valueMin, float valueMax) {
    /* Constrains the value to be within a given range */
    if (value < valueMin) {
        value = valueMin;
    } else if (value > valueMax) {
        value = valueMax;
    }
    return value;
}

void resetErrorsInPID() {
    /* Avoids relying on old error values when tuning */
    PID.err = 0;
    PID.errSum = 0;
    PID.errDiff = 0;
    PID.errPrev = 0;
}

void receiveDataWithMarkers() {
    /* Stores data between parenthesis */
    if(!isSerialReceiving) {
        if (charReceived == '(') {
            isSerialReceiving = true;
        }
    } else {
        if (charReceived == ')') {
            stringReceived[indexParse] = '\0';
            isSerialReceiving = false;
            indexParse = 0;
            parseReceivedData();
        } else {
            stringReceived[indexParse] = charReceived;
            indexParse++;
        }
    }
}

void parseReceivedData() {
    /* Extracts parameters from the received string */
    char *ptr = strtok(stringReceived, stringParseDelim);
    strcpy(stringParseMode, ptr);
    if (!strcmp(stringParseMode, "CL")) {
        ptr = strtok(NULL, stringParseDelim);
        PID.sp = strtod(ptr, NULL);
        ptr = strtok(NULL, stringParseDelim);
        PID.kp = strtod(ptr, NULL);
        ptr = strtok(NULL, stringParseDelim);
        PID.ki = strtod(ptr, NULL);
        ptr = strtok(NULL, stringParseDelim);
        PID.kd = strtod(ptr, NULL);
        isClosedLoop = true;
        resetErrorsInPID();
    } else if (!strcmp(stringParseMode, "OL")) {
        ptr = strtok(NULL, stringParseDelim);
        inputOpenLoop = strtod(ptr, NULL);
        isClosedLoop = false;
    }
}

void sendMeasurement() {
    /* Sends measurements in a specific format through the serial port */
    formatString();
    for(ptrStringSent = stringSent; *ptrStringSent != '\0'; ptrStringSent++) {
        while(!U1STAbits.TRMT || isSerialReceiving);
        U1TXREG = *ptrStringSent;
    }
}

void formatString() {
    /* Format: <input/output> */
    /* Warning: not robust at all... just wanted to avoid including sprintf */
    ptrStringSent = stringSent;
    *ptrStringSent++ = '<';
    if (isClosedLoop) {
        floatTwoDecimalsToString(ptrStringSent, PID.sp); 
    } else {
        floatTwoDecimalsToString(ptrStringSent, inputOpenLoop);
    }
    ptrStringSent = stringSent + strlen(stringSent);
    *ptrStringSent++ = '/';
    floatTwoDecimalsToString(ptrStringSent, adcVolt);
    ptrStringSent = stringSent + strlen(stringSent);
    *ptrStringSent++ = '>';
    ptrStringSent = '\0';
}

void floatTwoDecimalsToString(char *ptr, float value) {
    /* Writes a positive float to the pointed string up to two decimal places */
    /* Warning: not robust at all... just wanted to avoid including sprintf */
    utoa(ptr, (unsigned int) value, 10);
    ptr += strlen(ptr);
    *ptr++ = '.';
    utoa(ptr++, (unsigned int)(value*10) % 10, 10);
    utoa(ptr, (unsigned int)(value*100) % 10, 10);
}