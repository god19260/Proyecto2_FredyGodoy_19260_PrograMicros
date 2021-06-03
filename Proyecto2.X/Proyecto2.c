/*
 * File:   Proyecto2.c
 * Author: fredy
 *
 * Created on May 29, 2021, 4:59 PM
 */


#include <xc.h>
#define _XTAL_FREQ 8000000
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (RC oscillator: 
                // CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
                                // pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                // protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                // protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //(Internal/External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                // (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM 
                       // pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                // Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable 
                                // bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//------------------------------------------------------------------------------
//********************* Declaraciones de variables *****************************
char Valor_CCP1;
char Valor_CCP2;
char Boton;
char B_flag;
char Adelante;
char Atras;
//------------------------------------------------------------------------------
//***************************** Prototipos *************************************
void Botones(void);
void Motores_Adelante(void);
void Motores_Atras(void);
void Motores_Neutro(void);
void Motores(void);
//------------------------------------------------------------------------------
//*************************** Interrupciones ***********************************
void __interrupt() isr (void){    
    // Interrupción del Puerto B
    if (RBIF == 1){ 
        
        B_flag = 1;
        if (RB0 == 0){
            Boton = 0; //Adelante
        }else if(RB1 == 0){
            Boton = 1; // Atras
            RD6 = 1;
        }else if(RB2 == 0){
            Boton = 2; // Neutro
            
        }else if(RB3 == 0){ 
            Boton = 3; // Luces
        }else if(RB4 == 0){ //RB4 == 0
            Boton = 4; // 4WD
        }    
        RBIF = 0; 
    }// Fin de interrupción del PORTB
    
    
    // Interrupcion del ADC module
    if (ADIF == 1){
        ADIF = 0;
        if (ADCON0bits.CHS == 0){
            Valor_CCP1 = ADRESH/2;
            ADCON0bits.CHS = 0;
        } else if(ADCON0bits.CHS == 1){ // CCP2
            ADCON0bits.CHS = 2;
        }
        else{
            ADCON0bits.CHS = 0;
        }   
        __delay_us(50);
        ADCON0bits.GO = 1; 
    } // Fin de interrupción del ADC
}    

void main(void) {
    // Oscilador
    IRCF0 = 1;       // Configuración del reloj interno 
    IRCF1 = 1;
    IRCF2 = 1;       // 8 Mhz   
  
    INTCON = 0b11101000;
    
    // Configurar PWM
    PR2 = 249;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b00001100;
    CCP2CONbits.CCP2M = 0b00001100;
    CCPR1L = 0x00;
    CCPR2L = 0x00;
    CCP1CONbits.DC1B = 0;
    CCP2CONbits.DC2B0 = 0;
    CCP2CONbits.DC2B1 = 0;
    
    // Configurarcion del TMR2
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS0 = 1; //prescaler de 16
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.TMR2ON = 1;
   
    // Configuración del modulo ADC
    PIE1bits.ADIE = 1;
    ADIF = 0; // Bandera de interrupción
    ADCON1bits.ADFM = 0; // Justificado a la izquierda    
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG0 = 0; // Voltajes de referencia en VSS y VDD
    ADCON0bits.ADCS0 = 0;
    ADCON0bits.ADCS1 = 1; // FOSC/8
    ADCON0bits.ADON = 1;
    __delay_us(50);
    ADCON0bits.GO = 1; 

    // Configuración del puerto B
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0=1;
    WPUBbits.WPUB1=1;
    // Activación Interrup on change
    IOCB0 = 1;
    IOCB1 = 1;
    INTCONbits.RBIF = 0;        //Limpiar bandera para puerto B
    INTCONbits.RBIE = 1;        //Interrupción puerto B
    
    
    // Configurar puertos
    ANSEL  = 0b00000111;
    ANSELH = 0x0;
    TRISA  = 0xff; // Puerto para entradas 
    TRISC  = 0;  // Definir el puerto C como salida
    TRISD  = 0;  // Definir el puerto D como salida
    TRISE  = 0;  // Definir el puerto E como salida
    
    //Limpieza de puertos
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    //loop principal
    while(1){  
        Motores();
        Botones();
        if (Adelante == 1 && Atras == 0){
            Motores_Adelante();
        }else if(Adelante == 0 && Atras == 1){
            Motores_Atras();
        }else if(Adelante == 0 && Atras == 0){
            Motores_Neutro();
        } 
    } // fin loop principal while 
} // fin main
void Motores(void){
    CCPR1L = Valor_CCP1;
    CCPR2L = 0x00;
}

void Botones(void){
    if (B_flag == 1){
        if(Boton == 0){ // Adelante
            Adelante = 1;
            Atras = 0;
        }else if(Boton == 1){ // Atras
            Adelante = 0;
            Atras = 1;
        }else if(Boton == 2){ // Neutro
            RD5 = 1;
            Adelante = 0;
            Atras = 0;
        }else if(Boton == 3){ // Luces
            
        }else if(Boton == 4){ // 4WD
            
        }
        B_flag = 0;
    }
}  

void Motores_Adelante(void){
    RD0 = RC2;
    RD2 = RD0;
    RD1=0;
    RD3=RD1;
}
void Motores_Atras(void){
    RD1 = RC2;
    RD3 = RD1;
    RD0 = 0;
    RD2 = RD0;
}

void Motores_Neutro(void){
    RD1 = 0;
    RD3 = RD1;
    RD0 = 0;
    RD2 = RD0;
}