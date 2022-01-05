/* 
 * Project  : RC circuit - Digital Lead Compensator
 * Author   : O. DJADANE
 * Updated  : 2022/01/05
 * Processor: dsPIC33EV32GM102
 * Compiler : MPLAB XC16
 */

#define FOSC	(20000000ULL)	// 20 MHz
#define FCY		(10000000ULL)	// FOSC / 2 = 10 MHz

#include <xc.h>
#include <stdbool.h>            // bool
#include <stdint.h>             // uint8_t, uint16_t
#include <string.h>             // strtok, strcpy
#include <stdlib.h>             // atof, utoa

#pragma config POSCMD = HS      // Primary Oscillator : High-speed Crystal
#pragma config FNOSC = PRI      // Initial oscillator : Primary Oscillator
#pragma config FWDTEN = OFF     // Watchdog Timer : disabled
#pragma config ICS = PGD3       // ICD Communication Channel : PGEC3/PGED3

// Variables/Defines - Sampling period
#define PR3VAL 3906             // (256 * 3907) / 10MHz = 100 ms

// Variables/Defines - PWM frequency
#define PR2VAL 9999             // 10MHz / (1 * 10000) = 1 kHz

// Variables/Defines - UART
#define BRGVAL 64               // 10MHz / (4 * 65) = 38461 bps (38400)
#define STR_IN_MAX 40
#define STR_OUT_MAX 20
char stringReceived[STR_IN_MAX];
char stringSent[STR_OUT_MAX];
char stringParseDelim[1] = "/";
volatile unsigned char charReceived;
bool flagSerialReceiving = false;

// Variables/Defines - ADC
#define ADC_RESOLUTION 1024
#define ADC_VREF 5.0f
volatile uint16_t adcValue = 0;

// Variables/Defines - Compensator
typedef struct {
    float ek[3];
    float uk[3];
    float sp;
    float pv;
    bool isEnabled;
    float b[3];
    float a[3];
} CZ_T;
CZ_T Cz = { {0, 0, 0}, {0, 0, 0}, 0, 0, false,
            {0.476, 0.059, -0.417}, {1, -1.641, 0.641} };
bool flagRunCommand = false;

// Prototypes
void readMeasurement();
void runCommand();
float bound(float, float, float);
void resetCompensator();
void receiveDataWithMarkers();
void parseReceivedData();
void sendMeasurement();
void formatString(char *, float [], uint8_t);
void floatTwoDecimalsToString(char*, float);

int main() {
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
    
    /* Setup Timer3 for 100 ms */
    T3CONbits.TON = 0;      // Disable Timer3
    T3CONbits.TCKPS = 3;    // Pre-scaler: 1:256
    T3CONbits.TCS = 0;      // Clock source: internal (FCY)
    T3CONbits.TGATE = 0;    // Gated timer mode: disabled
    TMR3 = 0;               // Clear timer register
    PR3 = PR3VAL;           // Period value
    T3CONbits.TON = 1;      // Enable Timer3
    
    /* Setup ADC for 10-bit resolution, using RB7/AN25 pin */
    TRISBbits.TRISB7 = 1;   // RB7: input
    ANSELBbits.ANSB7 = 1;   // RB7: analog
    AD1CON1bits.ADON = 0;   // Disable ADC module
    AD1CON2bits.VCFG = 0;   // Voltage reference: AVDD / AVSS
    AD1CON3bits.ADCS = 0;   // Conversion clock: TAD=(0+1)*TCY (1/10M > 75n)
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
        if (flagRunCommand) {
            flagRunCommand = false;
            readMeasurement();
            runCommand();
            sendMeasurement();
        }
    }
    return 0;
}

void __attribute__ ((__interrupt__, no_auto_psv)) _AD1Interrupt(void) {
    adcValue = ADC1BUF0;
    flagRunCommand = true;
    IFS0bits.AD1IF  = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){
    charReceived = U1RXREG;
    receiveDataWithMarkers();
    IFS0bits.U1RXIF = 0;
}

void readMeasurement() {
    /* Scales the value in the ADC register for the control algorithm */
    Cz.pv = adcValue * ADC_VREF / ADC_RESOLUTION;
}

void runCommand() {
    /* Computes the duty cycle of the PWM according to the control algorithm */
    if (Cz.isEnabled) {
        Cz.ek[0] = Cz.sp - Cz.pv;
        Cz.uk[0] = Cz.b[0]*Cz.ek[0] + Cz.b[1]*Cz.ek[1] + Cz.b[2]*Cz.ek[2]
                - Cz.a[1]*Cz.uk[1] - Cz.a[2]*Cz.uk[2];
		Cz.ek[2] = Cz.ek[1];
        Cz.ek[1] = Cz.ek[0];
        Cz.uk[2] = Cz.uk[1];
        Cz.uk[1] = Cz.uk[0];
    }
    OC1RS = (uint16_t) bound(Cz.uk[0] * (PR2VAL/ADC_VREF), 2, PR2VAL);
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

void resetCompensator() {
    /* Avoids relying on old error values when switching loop mode */
    Cz.uk[0] = 0; Cz.uk[1] = 0; Cz.uk[2] = 0;
    Cz.ek[0] = 0; Cz.ek[1] = 0; Cz.ek[2] = 0;
}

void receiveDataWithMarkers() {
    /* Stores data between parenthesis */
    static uint8_t i = 0;
    if(!flagSerialReceiving) {
        if (charReceived == '(') {
            flagSerialReceiving = true;
        }
    } else {
        if (charReceived == ')') {
            stringReceived[i] = '\0';
            flagSerialReceiving = false;
            i = 0;
            parseReceivedData();
        } else {
            stringReceived[i] = charReceived;
            i++;
        }
    }
}

void parseReceivedData() {
    /* Extracts parameters from the received string */
    char stringParseMode[3];
    char *ptr = strtok(stringReceived, stringParseDelim);
    strcpy(stringParseMode, ptr);
    if (!strcmp(stringParseMode, "CL")) {
        ptr = strtok(NULL, stringParseDelim);
        Cz.sp = strtod(ptr, NULL);
        // no need to extract the rest of the parameters sent by qt-pid-scope...
        Cz.isEnabled = true;
    } else if (!strcmp(stringParseMode, "OL")) {
        resetCompensator();
        ptr = strtok(NULL, stringParseDelim);
        Cz.uk[0] = strtod(ptr, NULL);
        Cz.isEnabled = false;
    }
}

void sendMeasurement() {
    /* Sends measurements in a specific format through the serial port */
    float channels[] = {
        Cz.isEnabled ? Cz.sp : Cz.uk[0],
        Cz.pv };
    formatString(stringSent, channels, sizeof(channels)/sizeof(channels[0]));
    
    char *ptr;
    for(ptr = stringSent; *ptr != '\0'; ptr++) {
        while(!U1STAbits.TRMT || flagSerialReceiving);
        U1TXREG = *ptr;
    }
}

void formatString(char *ptr, float array[], uint8_t sizeArray) {
    /* Format: values are slash separated, the whole string is enclosed in <> */
    /* Warning: not robust at all... just wanted to avoid including sprintf */
    char *ptrInitial = ptr;
    uint8_t i;
    *ptr++ = '<';
    for(i = 0; i < sizeArray; i++) {
        floatTwoDecimalsToString(ptr, array[i]);
        ptr = ptrInitial + strlen(ptrInitial);
        if (i != sizeArray-1 ) {
            *ptr++ = '/';
        }
    }
    *ptr++ = '>';
    ptr = '\0';
}

void floatTwoDecimalsToString(char *ptr, float value) {
    /* Writes a positive float to the pointed string up to two decimal places */
    /* Warning: not robust at all... just wanted to avoid including sprintf */
    utoa(ptr, (uint16_t) value, 10);
    ptr += strlen(ptr);
    *ptr++ = '.';
    utoa(ptr, (uint16_t)(value*10) % 10, 10);
    ptr++;
    utoa(ptr, (uint16_t)(value*100) % 10, 10);
}