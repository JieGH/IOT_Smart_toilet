#define _XTAL_FREQ 4000000
#include <xc.h>
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)
// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)
// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)
// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)
// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <stdlib.h>            //standard library
#include <stdio.h>
#include <spi.h>            //serial peripheral interface functions
#include <delays.h>         //time delay functions
#include <usart.h>            //USART functions
#include <string.h>            //string functions
#include <adc.h>
#include <timers.h>
#include "MRF24J40.h"        //driver function definitions for MRF24J40 RF transceiver
#include <p18c452.h>

void Init_IO(void) {
    PORTA = 0x04;                            //PORTA initially all zeros except RA2 (TC77 chip select)
    TRISA = 0xF8;                            //RA0 and RA1 outputs (LEDs), RA2 Output (TC77 CS), rest inputs
    TRISB = 0x00;                            //JL To use somepin as output pin
    INTCON2bits.RBPU = 0;                    //enable pull up resistors on PORTB
    ADCON0 = 0x1C;                           //turn off analog input

    PORTCbits.RC0 = 1;                        //Chip select (/CS) initially set high (MRF24J40)
    TRISCbits.TRISC0 = 0;                    //Output: /CS
    PORTCbits.RC1 = 1;                        //WAKE initially set high (MRF24J40)
    TRISCbits.TRISC0 = 0;                    //Output: WAKE
    PORTCbits.RC2 = 1;                        //RESETn initially set high (MRF24J40)
    TRISCbits.TRISC2 = 0;                    //output: RESETn

    INTCONbits.INT0IF = 0;                //clear the interrupt flag (INT0 = RB0)
    INTCONbits.INT0IE = 1;                    //enable INT0
    RCONbits.IPEN = 1;                        //enable interrupt priorities
    INTCONbits.GIEH = 1;                    //global interrupt enable
    OSCCONbits.IDLEN = 1;                    //enable idle mode (when Sleep() is executed)

    OpenUSART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE &
              USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 25);    //setup USART @ 9600 Baud

    OpenSPI(SPI_FOSC_4, MODE_00, SMPMID);        //setup SPI bus (SPI mode 00, 1MHz SCLK) (MRF24J40)

    RB6 = 0;    // JL use RB6 as a output pin to indicate the Light ball
    RB7 = 0;    // JL use RB7 as a output pin to indicate the heating device in the toilet room
    RB5 = 0;    // JL output pin RB5 will be high when this board is transmitting data therefore to measure TDMA
}

void USARTOut(char *data, char bytes) {
    int i;
    for (i = 0; i < bytes; i++) {
        while (BusyUSART());
        WriteUSART(data[i]);
    }
}

typedef struct {
    unsigned char GID;
    unsigned char NID;            //unique identifier
    float Data1, Data2;            //data byte
    unsigned int crc;
}
        PacketType;

PacketType TxPacket, RxPacket;            //YL transmitted and received packets
char Text[128];
unsigned int ADCresult1, ADCresult2, Strength, CRCRight;//YL  variables of ADC results in digital,RF strength,correct of CRC
float Voltage1, Voltage2;//YL  convert digital ADC result into voltage
float Temperature;//YL  toilet temperature
float Lux;//YL  individual toilet room light strength
unsigned char FirstFlag = 1;//YL  guarantee the correctness of the accepted data by judging the firstflag

void ADC(void) {
    OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_8_TAD,   //YL 32MHZ crystal frequency,The result is aligned to the right,ADC sampling frequency of 8 Tad
            ADC_CH3 & ADC_INT_OFF & ADC_REF_VDD_VSS, 0);//YL choose channel 3 as digital output,choose 0 as reference voltage
    Delay10TCYx(5);                                     //YL delay time for 10us
    OpenADC(ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_8_TAD,
            ADC_CH4 & ADC_INT_OFF & ADC_REF_VDD_VSS, 0);
    Delay10TCYx(5);
    SetChanADC(ADC_CH3);                               //YL choose channel 3 as digital output
    ConvertADC();                                      //YL start converting
    while (BusyADC());
    ADCresult1 = ReadADC();                            //YL  read the ADC and write the result into the ADCresult1 variable

    SetChanADC(ADC_CH4);                               //YL choose channel 3 as digital output
    ConvertADC();                                      //YL start converting
    while (BusyADC());
    ADCresult2 = ReadADC();                            //YL  read the ADC and write the result into the ADCresult2 variable
    CloseADC();                                        //YL  close the ADC function
}
//DL speed up CRC implementation to use byte-by-byte table because comparing bit-by-bit is too slow.
const unsigned char code_Table[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/*****************function to caluculate CRC************************/
unsigned char CalculateCRC(unsigned char *message, unsigned char length) {
    unsigned char i, crc = 0;
//DL by calculation which is used bitwise XOR, if left crc is 0, this massage will be right data
    for (i = 0; i < length; i++) {
        crc = code_Table[crc ^ message[i]];
    }
    return crc;
}

// JL this function will turn on the light(pin RB6) or the heat(pin RB7)
// JL based on the room light intensity and room temperature
void too_dark_cold() {
// JL when the room is too dark, then turn on the light
    if (Lux < 50)
        RB6 = 1;
    else
        RB6 = 0;
// JL if the room is colder than 23 c degree, the heat will turn on
    if (Temperature < 23)
        RB7 = 1;
    else
        RB7 = 0;
}

void main(void) {
    Init_IO();
    MRF24J40Init();                //initialise IEEE 802.15.4 transceiver
    SetChannel(CHANNEL_16);        //set RF channel CHANNEL_11-CHANNEL_26 (EACH GROUP MUST HAVE UNIQUE CHANNEL)
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_16);        //setup timer 0 with prescaler x16
    FirstFlag = 1;

    // JL calibration coefficient for temperature  attached on node1
    float ta1 = -38.46;
    float tb1 = 21.92;
    float tc1 = 25.70;
    float toffset = -0.3;

    // JL calibration coefficient for light sensor board attached on node1
    float a3 = -17.60;
    float b3 = 250.06;
    float c3 = -123.83;

    while (1) {
        RB5 = 0;    // JL reset pin RB5 to digital low
        ADC();      // JL read ADC value
        // JL calculate the voltage of two analougue pin used in the board based on the ADCresult
        Voltage1 = (float) ADCresult1 * 3.3 / 1023;
        Voltage2 = (float) ADCresult2 * 3.3 / 1023;
        // JL calculate the temperature value and the light intensity
        Temperature = ta1 * (float) Voltage1 * Voltage1 + tb1 * Voltage1 + tc1 + toffset;
        Lux = a3 * Voltage2 * Voltage2 + b3 * Voltage2 + c3;
        // JL calling this function to decide whether we need turn on the heat or light
        // JL based on the room temperature and light intensity
        too_dark_cold();

        // JL to set the minimum of the light intensity in case 'LUX' equal to minus value which is incorrect value
        if (Lux < 2)
            Lux = 1;
        // JL put delay before sending all the data
        Delay10TCYx(20);

        // JL Group 6 this node is node number one
        TxPacket.GID = 6;
        TxPacket.NID = 1;

        //JL include the temperature of the room and light intensity as Data1 and Data2 in the packet
        TxPacket.Data1 = Temperature;
        TxPacket.Data2 = Lux;

        // JL to calculate the CRC before send data
        TxPacket.crc = CalculateCRC(&TxPacket, sizeof(TxPacket) - sizeof(TxPacket.crc));
        //--------------------send data wirelessly-----------------------
        if (INTCONbits.TMR0IF) {
            if (CRCRight == 1) {
                INTCONbits.TMR0IF = 0;
                PORTA = 0x05;
                PHYTransmit((char *) &TxPacket, sizeof(PacketType));    //Transmit RF data packet
                sprintf(Text, "Group ID=%u   Node ID=%u   Data1=%.2f   Data2=%.2f   RSSI =%u   CRCRight=%u \r\n",
                        TxPacket.GID, TxPacket.NID, TxPacket.Data1, TxPacket.Data2, Strength, CRCRight);
                // JL the data packet contain the group ID, NOde id, the two datas, RSSI and CRC status.
                USARTOut(Text, strlen(Text)); // JL print the results to the serial output
                PORTA = 0x04;
                RB5 = 1; // JL enable this pin to digital high after broadcast for TDMA measurenment
            } else {
                sprintf(Text, "Group ID=6   Node ID=5   Data1=0   Data2=0   RSSI =0   CRCRight=0\r\n");
                USARTOut(Text, strlen(Text));
            }
        }

        //----------------receive data-------------------------
        if (PHYReceive((char *) &RxPacket, &Strength) == sizeof(PacketType)) {
            PORTA = 0x06;
            // JL check the CRC every time received a packet, the reminder will be calculated,
            // if the reminder is not zero, means data received is incorrect else correct
            if (0 == CalculateCRC(&RxPacket, sizeof(PacketType)))
                CRCRight = 1;
            else
                CRCRight = 0;
            // JL if crc correct, the data received form slaves will be printed out in the serial output
            if (CRCRight == 1) {
                sprintf(Text, "Group ID=%u   Node ID=%u   Data1=%.2f   Data2=%.2f   RSSI =%u   CRCRight=%u \r\n",
                        RxPacket.GID, RxPacket.NID, RxPacket.Data1, RxPacket.Data2, Strength, CRCRight);
                USARTOut(Text, strlen(Text));
                PORTA = 0x04;
            } else {
                // JL if crc incorrect, will print this information, the node ID 5 is to inform python,
                // the data received is CRC incorrect, detail of this '5' see python code
                sprintf(Text, "Group ID=6   Node ID=5   Data1=0   Data2=0   RSSI =0   CRCRight=0\r\n");
                USARTOut(Text, strlen(Text));
            }
        }
    }
}
