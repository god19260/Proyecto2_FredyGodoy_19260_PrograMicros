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
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: 
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
char Lock;
char WD = 0; //4WD
char Contador_Servo1=0;
char Contador_Servo2=0;
char Direccion = 0;
char M_Luces;
char t;
//------------------------------------------------------------------------------
//***************************** Prototipos *************************************
void Botones(void);
void Motores_Adelante(void);
void Motores_Atras(void);
void Motores_Neutro(void);
void Motores(void);
void Servo_1(void);
void Servo_2(void);
//------------------------------------------------------------------------------
//*************************** Interrupciones ***********************************
void __interrupt() isr (void){    
    // Interrupcion del timer0 - Cada 0.5ms
    if (T0IF == 1){ 
        T0IF = 0;
        TMR0 = 255; 
        // Servo 1
        if(Contador_Servo1 <= Direccion){
           RD6 = 1;
        }else {
           RD6 = 0;
        }
        if(Contador_Servo1 >= 20){
            Contador_Servo1 = 0;
        }
        
        //Servo 2
        if(Contador_Servo2 <= M_Luces){
           RD7 = 1;
        }else {
           RD7 = 0;
        }
        if(Contador_Servo2 >= 20){
            Contador_Servo2 = 0;
        }
        Contador_Servo1++;
        Contador_Servo2++;
    } // Fin de interrupción timer0
    
    // Interrupcion del ADC module
    if (ADIF == 1){
        ADIF = 0;
        if (ADCON0bits.CHS == 0){ // CCP1
            Valor_CCP1 = ADRESH/2;
            ADCON0bits.CHS = 1;
        } else if(ADCON0bits.CHS == 1){ // CCP2
            Valor_CCP2 = ADRESH/2;
            ADCON0bits.CHS = 2;            
        } else if(ADCON0bits.CHS == 2){
            Direccion = ADRESH;
            Direccion = Direccion*6/255;
            ADCON0bits.CHS = 3;        
        } else if(ADCON0bits.CHS == 3){
            M_Luces = ADRESH;
            M_Luces = M_Luces*6/255;
            ADCON0bits.CHS = 0;        
        } 
        __delay_us(50);
        ADCON0bits.GO = 1; 
    } // Fin de interrupción del ADC
    // Interrupción del Puerto B
    if (RBIF == 1){ 
        if (PORTB == 0b11111110){ // RB0 == 0
            Boton = 0; //Adelante
            B_flag = 1;
        }else if(PORTB == 0b11111101){ // RB1 == 0
            Boton = 1; // Atras
            B_flag = 1;
        }else if(PORTB == 0b11111011){ // RB2 == 0
            Boton = 2; // Neutro
            B_flag = 1;
        }else if(PORTB == 0b11110111){ // RB3 == 0 
            Boton = 3; // Luces
            B_flag = 1;
        }else if(PORTB == 0b11101111){ //RB4 == 0
            Boton = 4; // 4WD
            B_flag = 1;
        }else if(PORTB == 0b11011111){ //RB5 == 0
            Boton = 5; // Lock
            B_flag = 1;
        }      
        RBIF = 0; 
    }// Fin de interrupción del PORTB
    // Interrupcion Serial
    if (RCIF == 1){
        RCIF = 0;         
        //TXREG --> Transmitir datos
        if (RCREG == '1'){ // 1 en hexa
            TXREG = Direccion;
        } else if(RCREG == '2'){
            TXREG = M_Luces;
        }else if(RCREG == '3'){
            
        }     
    } // Fin interrupcion Serial
    
}    

void main(void) {
    // Oscilador
    IRCF0 = 1;       // Configuración del reloj interno 
    IRCF1 = 1;
    IRCF2 = 1;       // 8 Mhz   
    
    // Configurar Timer0
    PS0  = 0;
    PS1  = 1;
    PS2  = 1;         // Prescaler de 64
    T0CS = 0;
    PSA  = 0;
    INTCON = 0b11101000;
    TMR0 = 255;
    
    // Configuración de Asynchronous TRANSMITTER
    TXEN = 1;
    SYNC = 0;
    SPEN = 1;
    // Configuración de Asynchronous RECEIVER
    CREN = 1;
    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 0;
    SPBRG=12;  // baudrate 9600 para 8MHz
    
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
    WPUBbits.WPUB2=1;
    WPUBbits.WPUB3=1;
    WPUBbits.WPUB4=1;
    WPUBbits.WPUB5=1;
    WPUBbits.WPUB6=1;
    // Activación Interrup on change
    IOCB0 = 1;
    IOCB1 = 1;
    IOCB2 = 1;
    IOCB3 = 1;
    IOCB4 = 1;
    IOCB5 = 1;
    IOCB6 = 1;
    INTCONbits.RBIF = 0;        //Limpiar bandera para puerto B
    INTCONbits.RBIE = 1;        //Interrupción puerto B
    
    
    // Configurar puertos
    ANSEL  = 0b00001111;
    ANSELH = 0x0;
    TRISA  = 0xff; // Puerto para entradas 
    TRISC  = 0b10000000;  // Definir el puerto C como salida
    TRISD  = 0;  // Definir el puerto D como salida
    TRISE  = 0;  // Definir el puerto E como salida
    
    //Limpieza de puertos
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    Boton = 2;
    B_flag = 0;
    Lock = 1;
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
    if(WD == 0){
        CCPR1L = 0;
        CCPR2L = Valor_CCP2;
    }else if(Lock == 1 && WD == 1){
        CCPR1L = Valor_CCP2;
        CCPR2L = Valor_CCP2;
    }else if((Lock == 0 && WD == 1)){
        CCPR1L = Valor_CCP1;
        CCPR2L = Valor_CCP2;    
    }
}

void Botones(void){
    if (B_flag == 1){
        if(Boton == 0 && RB0 == 1){ // Adelante
            Adelante = 1;
            Atras = 0;
            B_flag = 0;
        }else if(Boton == 1){ // Atras
            Adelante = 0;
            Atras = 1;
            B_flag = 0;
        }else if(Boton == 2){ // Neutro
            Adelante = 0;
            Atras = 0;
            B_flag = 0;
        }else if(Boton == 3){ // Luces
            B_flag = 0;
        }else if(Boton == 4){ // 4WD
            WD = 1;
            B_flag = 0;
        }else if(Boton == 5){ // Lock
            if(Lock == 1){
                Lock = 0;
            }else{
                Lock = 1;
            }
            B_flag = 0;
        }
        
    }
}  

void Motores_Adelante(void){
    RD0 = RC2;
    RD2 = RC1;
    RD1 = 0;
    RD3 = RD1;
}
void Motores_Atras(void){
    RD1 = RC2;
    RD3 = RC1;
    RD0 = 0;
    RD2 = RD0;
}

void Motores_Neutro(void){
    RD1 = 0;
    RD3 = RD1;
    RD0 = 0;
    RD2 = RD0;
}


 