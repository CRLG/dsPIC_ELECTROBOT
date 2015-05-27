/**********************************************************************
* © 2010 Microchip Technology Inc.
*
* FileName:        main_led.c
* Dependencies:    p24HJ64GP502.h
* Processor:       PIC24H
* Compiler:        MPLAB® C30 v2.01 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (“Microchip”) licenses this software to you
* solely for use with Microchip dsPIC® digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:  
*
* Simple demo program showing a flashing LED on each port
*
**********************************************************************/

#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif
#include <libpic30.h>

#include "I2CSlaveDrv.h"	//Include file for I2C1 Driver
#include "General.h"
/******************************* 

Set device configuration values 

********************************/
#ifdef __PIC24HJ64GP502__
_FOSCSEL(FNOSC_FRC);								// set oscillator mode for FRC ~ 8 Mhz
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);	// use OSCIO pin for RA3
_FWDT(FWDTEN_OFF);									// turn off watchdog
#elif defined(__dsPIC33FJ128MC802__)
_FOSCSEL(FNOSC_FRCPLL);								
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);									
#endif


#define PERIOD  0x3FFF						// sets the default interval flash rate
#define FLASH_RATE_NORMAL 1						// smaller value yields faster rate (1-100)
#define FLASH_RATE_ERR_COM_MASTER 30        // flash rate if master is absent
#define FOREVER 1							// endless 

/* function prototypes */
void InitTimer1();							 

/* globals */
unsigned int Counter = 0;
volatile unsigned int timer_expired;

unsigned short AnalogInput[4];

void Init_Ports(void);
void Init_Analog(void);
void Init_QEI(void);
void Init_PWM(void);
void Init_Registers(void);

unsigned short getAnalog(unsigned char voie);

void ControleMoteur1(char speed);
void ControleMoteur2(char speed);

T_dsPIC_REGISTER dsPIC_reg[MAX_REGISTRES_NUMBER];
    
    
    
    
    
    


/********************************* 

	main entry point

*********************************/

int main ( void )
{
 char scval;
 unsigned short flash_speed;
 unsigned char perteComMaster=0;
 
 Init_Ports();
 Init_Registers();
 Init_QEI();
 Init_Analog();
 InitTimer1();
 Init_PWM();
 i2c1_init();
 
 // A l'init, tous les moteurs sont éteints
 ControleMoteur1(0);
 ControleMoteur2(0);

	/*  endless loop*/
	while (FOREVER)
	{

        // Diag de perte de comm avec le master
        if (timer_expired) {
ENTER_CRITICAL_SECTION_I2C()
            if (cptPerteComMaster!=0xFFFF) { cptPerteComMaster++; }
            perteComMaster = (cptPerteComMaster > 100);
LEAVE_CRITICAL_SECTION_I2C()
            flash_speed = (perteComMaster==0)?FLASH_RATE_NORMAL:FLASH_RATE_ERR_COM_MASTER;
        }
        
		if (timer_expired && (Counter >= flash_speed) )
		{
            LATAbits.LATA4 = !LATAbits.LATA4; 
		    Counter = 0;
			timer_expired = 0;
		}
		AnalogInput[0] = getAnalog(0)>>4;
		AnalogInput[1] = getAnalog(1)>>4;
		AnalogInput[2] = getAnalog(4)>>4;
		AnalogInput[3] = getAnalog(5)>>4;
ENTER_CRITICAL_SECTION_I2C()
        dsPIC_reg[REG_EANA_6_10].val = AnalogInput[0];
        dsPIC_reg[REG_EANA_7_11].val = AnalogInput[1];
        dsPIC_reg[REG_EANA_8_12].val = AnalogInput[2];
        dsPIC_reg[REG_EANA_9_13].val = AnalogInput[3];
LEAVE_CRITICAL_SECTION_I2C()

        // _____________________________________________________
        // Recherche une nouvelle demande de PWM sur le moteur 3 (5 sur dsPIC 2)
        if (dsPIC_reg[REG_COMMANDE_MOTEUR_3].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            scval = (signed char)dsPIC_reg[REG_COMMANDE_MOTEUR_3].val;
            dsPIC_reg[REG_COMMANDE_MOTEUR_3].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleMoteur1(scval);            
         }    
        // _____________________________________________________
        // Recherche une nouvelle demande de PWM sur le moteur 4 (6 sur dsPIC 2)
        if (dsPIC_reg[REG_COMMANDE_MOTEUR_4].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
            scval = (signed char)dsPIC_reg[REG_COMMANDE_MOTEUR_4].val;
            dsPIC_reg[REG_COMMANDE_MOTEUR_4].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
            ControleMoteur2(scval);            
         }    
	}
}

/*---------------------------------------------------------------------
  Function Name: InitTimer1
  Description:   Initialize Timer1 for 1 second intervals
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void InitTimer1( void )
{
	T1CON = 0;						/* ensure Timer 1 is in reset state */
 	IFS0bits.T1IF = 0;				/* reset Timer 1 interrupt flag */
	IPC0bits.T1IP = 4;				/* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE = 1;				/* enable Timer 1 interrupt */
	PR1 = PERIOD;					/* set Timer 1 period register */
	T1CONbits.TCKPS = 2;			/* select Timer1 Input Clock Prescale */
	T1CONbits.TCS = 0;			 	/* select external timer clock */
	T1CONbits.TON = 1;			 	/* enable Timer 1 and start the count */ 
	
}


/*---------------------------------------------------------------------
  Function Name: _T1Interrupt
  Description:   Timer1 Interrupt Handler
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
{


	timer_expired = 1;				/* flag */
	Counter++;						/* keep a running counter */
 	IFS0bits.T1IF = 0;				/* reset timer interrupt flag	*/

}	
	


// ___________________________________________________________
void Init_QEI(void)
{
     // Configure sur quelles pins les entrées QEI sont :
     // Codeur 1 : QEA sur RB2 (RP6) / QEB sur RB3 (RP7)
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    _QEA1R = 4;
    _QEB1R = 5;
    // Codeur 2 : QEA sur RB4 (RP4) / QEB sur RB5 (RP5)
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;
    _QEA2R = 6;
    _QEB2R = 7;

    // Active le module QEI
    QEI1CON = 0;
    QEI1CONbits.QEIM = 7; // Mode quadrature x4
    QEI2CON = 0;
    QEI2CONbits.QEIM = 7; // Mode quadrature x4
    // RAZ des compteurs
    POS1CNT = 0;
    POS2CNT = 0;


}

// ___________________________________________________________
void Init_Ports(void)
{
    /* 	Initialize ports */
    LATA  = 0x0000; 				// set latch levels
    TRISA = 0xFFFF; 				// set IO as inputs
    // Configuration de la LED sur la carte MICTOSTICK
    TRISAbits.TRISA4 = 0; 				// set IO as outputs


    LATBbits.LATB15  = 0;                         // set latch levels
    TRISBbits.TRISB15 = 0; 				// set IO as outputs

    LATBbits.LATB14  = 0;                         // set latch levels
    TRISBbits.TRISB14 = 0; 				// set IO as outputs

    LATBbits.LATB13  = 0;                         // set latch levels
    TRISBbits.TRISB13 = 0; 				// set IO as outputs

    LATBbits.LATB12  = 0;                         // set latch levels
    TRISBbits.TRISB12 = 0; 				// set IO as outputs

    LATBbits.LATB11  = 0;                         // set latch levels
    TRISBbits.TRISB11 = 0; 				// set IO as outputs

    LATBbits.LATB10  = 0;                         // set latch levels
    TRISBbits.TRISB10 = 0; 				// set IO as outputs

}

// ___________________________________________________________
void Init_PWM(void)
{
// Initialize Output Compare Module
OC1CONbits.OCM = 0b000; // Disable Output Compare Module
OC1R = 100; // Write the duty cycle for the first PWM pulse
OC1RS = 200; // Write the duty cycle for the second PWM pulse
OC1CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC1R = 100; // Load the Compare Register Value
OC1CONbits.OCM = 0b110; // Select the Output Compare mode
_RP15R = 0b10010; // OC1 sur in RP15

OC2CONbits.OCM = 0b000; // Disable Output Compare Module
OC2R = 100; // Write the duty cycle for the first PWM pulse
OC2RS = 200; // Write the duty cycle for the second PWM pulse
OC2CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC2R = 100; // Load the Compare Register Value
OC2CONbits.OCM = 0b110; // Select the Output Compare mode
_RP12R = 0b10011; // OC2 sur in RP12


/*
OC3CONbits.OCM = 0b000; // Disable Output Compare Module
OC3R = 100; // Write the duty cycle for the first PWM pulse
OC3RS = 200; // Write the duty cycle for the second PWM pulse
OC3CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC3R = 100; // Load the Compare Register Value
OC3CONbits.OCM = 0b110; // Select the Output Compare mode

OC4CONbits.OCM = 0b000; // Disable Output Compare Module
OC4R = 100; // Write the duty cycle for the first PWM pulse
OC4RS = 200; // Write the duty cycle for the second PWM pulse
OC4CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
OC4R = 100; // Load the Compare Register Value
OC4CONbits.OCM = 0b110; // Select the Output Compare mode
*/

// Initialize and enable Timer2
T2CONbits.TON = 0; // Disable Timer
T2CONbits.TCS = 0; // Select internal instruction cycle clock
T2CONbits.TGATE = 0; // Disable Gated Timer mode
T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
TMR2 = 0x00; // Clear timer register
PR2 = 500; // Load the period value
IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level
IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
IEC0bits.T2IE = 0; // Disable Timer 2 interrupt
T2CONbits.TON = 1; // Start Timer
}


// ___________________________________________________________
void Init_Analog(void)
{
  AD1CON1bits.AD12B = 0;        // Select 10-bit mode
  AD1CON1bits.SSRC = 0;         // Conversion manuelle
  AD1CON1bits.ASAM = 0;         // Sample manuel
   
  AD1CON3bits.ADCS = 10;
  
  AD1PCFGL = 0xCC;              // AN0/AN1/AN4/AN5 sont des entrées analogiques
  
  AD1CON1bits.ADON = 1;         // Active le module ADC   
}


// ___________________________________________________________
unsigned short getAnalog(unsigned char voie)
{
  unsigned short ADCValue;
 // TODO Sélectionne l'entrée à convertir
 AD1CHS0bits.CH0SA = voie;
 AD1CON1bits.SAMP = 1; // Start sampling
 // for (i=0; i<3000; i++);
 __delay32(3000);
 //DelayUs(10); // Wait for sampling time (10us)
 AD1CON1bits.SAMP = 0; // Start the conversion
 while (!AD1CON1bits.DONE); // Wait for the conversion to complete
 ADCValue = ADC1BUF0; // Read the conversion result
 return(ADCValue);  
}      


// ___________________________________________________________
void Init_Registers(void)
{
  unsigned short i=0;
  
  // Initialise de manière massive tous les registres
  for (i=0; i<MAX_REGISTRES_NUMBER; i++) {
     dsPIC_reg[i].val               = 0;
     dsPIC_reg[i].new_data          = 0;
     dsPIC_reg[i].type_read_write   = READ_ONLY; 
  }
  // Les registres Read/write
  for (i=REG_COMMANDE_MOTEUR_3; i<=REG_PINCFG_PGEC; i++) {
    dsPIC_reg[i].type_read_write = READ_WRITE;     
  }
  
  // Initialise les valeurs par défaut
  dsPIC_reg[REG_VERSION_SOFT_MAJ].val               = VERSION_SOFT_MAJ;
  dsPIC_reg[REG_VERSION_SOFT_MIN].val               = VERSION_SOFT_MIN;
  dsPIC_reg[REG_PTR_REG_LECTURE_I2C].val            = REG_CODEUR_1_MSB;
  dsPIC_reg[REG_NBRE_REGISTRES_LECTURE_I2C].val     = 8; // Nombre de registres lus par le MBED lors d'une opération de lecture
}   



// _______________________________________________________
void ControleMoteur1(char speed)
{
  // Saturation
  if (speed > 100)          { speed = 100; }
  else if (speed < -100)    { speed = -100; }
  
  if (speed < 0) {
      LATBbits.LATB13 = 1; 
      LATBbits.LATB14 = 0;  
      speed = -1 * speed;
  }
  else if (speed >  0) {
      LATBbits.LATB13 = 0; 
      LATBbits.LATB14 = 1;  
  }
  else { // speed = 0
     LATBbits.LATB13 = 1;
     LATBbits.LATB14 = 1; 
  }
  
  // Applique le rapport cyclique 
  OC1RS = (speed * PR2)/100;           
}



// _______________________________________________________
void ControleMoteur2(char speed)
{
  // Saturation
  if (speed > 100)          { speed = 100; }
  else if (speed < -100)    { speed = -100; }
  
  if (speed < 0) {
      LATBbits.LATB10 = 1; 
      LATBbits.LATB11 = 0;  
      speed = -1 * speed;
  }
  else if (speed >  0) {
      LATBbits.LATB10 = 0; 
      LATBbits.LATB11 = 1;  
  }
  else { // speed = 0
     LATBbits.LATB10 = 1;
     LATBbits.LATB11 = 1; 
  }
  
  // Applique le rapport cyclique 
  OC2RS = (speed * PR2)/100;           
}
