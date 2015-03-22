/*
===================================================================================================================
|  MSP430F5172 code for PV panel Maximum Power Point Tracking (MPPT). Inputs and outputs are as follows:          |
|  A1 (P1.1): Output voltage sensing. 1.5V reference.                                                             |
|  A2 (P1.2): Output current sensing. 1.5V reference.                                                             |
|  P2.0: Combined gate drive output. Fsw = 20KHz.                                                                 |
|                                                                                                                 |
|  Code overview: This MPPT is a hybrid between two common methods of MPPT tracking:                              |
|  Preturb and observe (P&O), and Sweep and Settle (S&S). Upon initialization, the MPPT performs                  |
|  and output power sweep through a subset of all duty cycles (in this case, 55%-->95%). In this sweep,           |
|  it logs what it believes to be the maximum power point, and begins the P&O operation at this starting point.   |
|  This has two benefits: quicker initial MPPT operation, and slightly better global maxima identification.       |
|                                                                                                                 |
|  Currently, this MPPT is being used with a Shell Solar Sq-85 85W PV charging a 12V deep cycle marine battery.   |
|  However, it is robust enough to be used with panels of varying power, as long as they are being used in an     |
|  environment where there is a large enough power fluctuation to detect a change in power.                       |
|  Output voltage and current sensing are also required.                                                          |
===================================================================================================================
*/


#include "msp430f5172.h"
#include <math.h>

// 10-bit ADC conversion result array
unsigned int ADC_Result[3];               //[2]:A0 = UNSUSED, [1]:A1 = V, [0]:A2 = I
            
//Create global variables 
float bestDuty = 0;
float bestPower = 0;
float currentPower = 0;
float lastPower = 0;
volatile unsigned long i = 0;	             // Declare counter variable
long dutyCycle = 5000;                     //start the duty cycle at an initial value

void readADC() {
	 while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
	 ADC10CTL0 |= ADC10ENC + ADC10SC;        // Sampling and conversion start
	 __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
	 __delay_cycles(5000);                   // Delay between sequence convs

	 //At this point, the results are contained in the ADC_result array, so check there
}

//Function to raise the MSP430 core voltage to levels required by the local osc.
void SetVcoreUp (unsigned int level)
 {
 	// Subroutine to change core voltage
   // Open PMM registers for write
   PMMCTL0_H = PMMPW_H;
   // Set SVS/SVM high side new level
   SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
   // Set SVM low side to new level
   SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
   // Wait till SVM is settled
   while ((PMMIFG & SVSMLDLYIFG) == 0);
   // Clear already set flags
   PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
   // Set VCore to new level
   PMMCTL0_L = PMMCOREV0 * level;
   // Wait till new level reached
   if ((PMMIFG & SVMLIFG))
     while ((PMMIFG & SVMLVLRIFG) == 0);
   // Set SVS/SVM low side to new level
   SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
   // Lock PMM registers for write access
   PMMCTL0_H = 0x00;
}

//This function uses the ADC read to calculate the output voltage
//10X voltage divider from the output & 10bit ADC result
float calcVoltage(unsigned int voltADC) { 
	  float temp = (float)voltADC * 0.01564; //cast the ADC read to float just to be safe
	  return temp;
  }
//This function uses the ADC read to calculate the output current
//0.125 voltage divider, INA194 current sense gain = 50, and 30mOhm sense resistor
float calcCurrent(unsigned int ampADC) { 
	  float temp = (float)ampADC * 0.01564; //cast the ADC read to float just to be safe
	  return temp;
  }


//This function does the actual MPPT tracking using preturb and observe.
//Duty cycle increments of 0.5%, wait time of ~10ms between samples.
void doMPPTTrack() {
	 dutyCycle = bestDuty;
	 int increment = 50;
	 for(;;) {

	    //measure the output current
	    readADC();
	    float currentHomie = calcCurrent(ADC_Result[0]);

	    //measure the output voltage
	    float voltageDawg = calcVoltage(ADC_Result[1]);

	    //calculate the current output power
	    currentPower = voltageDawg*currentHomie;

		//set a lower bound on the output power resolution (better noise immunity)
		if(currentPower < 0.5) {
			currentPower = 0;
		}

    //Track the actual MPPT. If the power increases, keep going the same direction.
		if (currentPower > lastPower) {     //move the duty cycle up
			dutyCycle += increment;           //move the DC by 0.5%
		}
    //Otherwise, turn around and go back. Later, rinse, repeat.
		else {                              //move the duty cycle down
			increment *= (-1);
			dutyCycle += increment;           //move the DC by 0.5%
		}

		//include some limits
		if (dutyCycle > 8700) dutyCycle = 8700; //88%
		if (dutyCycle < 5000) dutyCycle = 5000; //50%

		TD0CCR2 = dutyCycle;                //write the new duty cycle to the TD0CCR2 register

		lastPower = currentPower;           //keep track of the "last" power

    //very crude delay loop
		i = 250000;
		do(i--);
		while(i != 0);		                	// Wait ~0.01s before sampling and moving again
	 }
 }

//function to sweep through a reasonable range of duty cycles so that the MPPT algorithm can start in the
//right ballpark. This works pretty well and keeps the MPPT from "blindly wandering" through
//the entire power curve before finding the MPPT. It also helps keep it away from local maximums to some degree.
void doSweep() {
	  unsigned int D_C;
	  bestPower = 0; //reset the "best" power at the beginning of each sweep
	  for (D_C = 5500; D_C < 9500; D_C += 100) { //do a power sweep from 55% to 95% duty cycle
		  TD0CCR2 = D_C; //write the new duty cycle to the TD0CCR2 register

      //crude delay to allow the transients to settle and the converter to reach steady state before sampling.
		  i = 250000;
		  do(i--);
		  while(i != 0);

		  //measure the output current
		  readADC();
		  float currentHomie = calcCurrent(ADC_Result[0]);

		  //measure the output voltage
		  float voltageDawg = calcVoltage(ADC_Result[1]);

		  //calculate the current output power
		  currentPower = voltageDawg*currentHomie;

		  //check if there is a new high score! (LOL)
		  if(currentPower > bestPower) {
			  bestPower = currentPower;
			  bestDuty = D_C;
		  }

      //yet another crude delay. Timing doesn't actually really matter, so this is fine.
		  i = 2500000;
		  do(i--);
		  while(i != 0);			                  // Wait ~0.01s before sampling and moving again
	  }
  }

//Main function. Here's where the magic happens!
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop the watchdog timer.

  P1SEL |= BIT6;				                    // Set P1.6 to output direction (Timer D0.0 output)
  P1DIR |= BIT6;
  P1SEL |= BIT7;				                    // Set P1.7 to output direction (Timer D0.1 output)
  P1DIR |= BIT7;
  P2SEL |= BIT0;				                    // Set P2.0 to output direction (Timer D0.2 output)
  P2DIR |= BIT0;
  P1DIR |= 0x01;				                    // Set P1.0 to output direction (to drive LED)
  P1OUT |= 0x01;				                    // Set P1.0  - turn LED on
  __delay_cycles(500000);
  P1OUT ^= 0x01;				                     // Toggle P1.0 using exclusive-or function  - turn LED off

  // Increase Vcore setting to level3 to support fsystem=25MHz
  // NOTE: Change core voltage one level at a time.
  SetVcoreUp (0x01);
  SetVcoreUp (0x02);
  SetVcoreUp (0x03);

  // Initialize DCO to 25MHz
  __bis_SR_register(SCG0);                  // Disable the FLL control loop
  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
  UCSCTL1 = DCORSEL_6;                      // Select DCO range 4.6MHz-88MHz operation
  UCSCTL2 = FLLD_1 + 763;                   // Set DCO Multiplier for 25MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (762 + 1) * 32768 = 25MHz
                                            // Set FLL Div = fDCOCLK/2
  __bic_SR_register(SCG0);                  // Enable the FLL control loop

  // Worst-case settling time for the DCO when the DCO range bits have been
  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
  // User Guide for optimization.
  // 32 x 32 x 25 MHz / 32,768 Hz = 782000 = MCLK cycles for DCO to settle
  __delay_cycles(782000);

   // Configure TimerD in Hi-Res Regulated Mode
  TD0CTL0 = TDSSEL_2;                        // TDCLK=SMCLK=25MHz=Hi-Res input clk select
  TD0CTL1 |= TDCLKM_1;                       // Select Hi-res local clock
  TD0HCTL1 |= TDHCLKCR;				               // High-res clock input >15MHz
  TD0HCTL0 = TDHM_0 + 					             // Hi-res clock 8x TDCLK = 200MHz
  		   TDHREGEN + 					               // Regulated mode, locked to input clock
  		   TDHEN;     					               // Hi-res enable

  // Wait some, allow hi-res clock to lock
  P1OUT ^= 0x01;							               // Toggle P1.0 using exclusive-OR, turn LED on
  while(!TDHLKIFG);					                 // Wait until hi-res clock is locked
  P1OUT |= 0x01;							               // Toggle P1.0 using exclusive-OR, turn LED off

  // Configure the CCRx blocks
  // These actually set the PWM frequency and duty cycle for the gate drives.
  TD0CCR0 = 10000;                           // PWM Period. So sw freq = 200MHz/10000 = 20 kHz
  TD0CCTL1 = OUTMOD_7 + CLLD_1;              // CCR1 reset/set
  TD0CCR1 = 5000;                            // CCR1 PWM duty cycle of 5000/10000 = 50%
  TD0CCTL2 = OUTMOD_7 + CLLD_1;              // CCR2 reset/set
  TD0CCR2 = 1000;                            // CCR2 PWM duty cycle of 1000/10000 = 10%
  TD0CTL0 |= MC_1 + TDCLR;

  // Configure ADC10 (output power measurement)
  ADC10CTL0 = ADC10SHT_2 + ADC10MSC + ADC10ON;// 16ADCclks, MSC, ADC ON
  ADC10CTL1 = ADC10SHP + ADC10CONSEQ_1;     // sampling timer, s/w trig.,single sequence
  ADC10CTL2 = ADC10RES;                     //10 bit res? //
  ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_2;   // A0,A1,A2(EoS)

  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
  REFCTL0 |= REFVSEL_0+REFON;               // Select internal ref = 1.5V
                                            // Internal Reference ON
  __delay_cycles(75);                       // Delay (~75us) for Ref to settle

  // Configure DMA0 (ADC10IFG trigger)
  DMACTL0 = DMA0TSEL_24;                    // ADC10IFG trigger
  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0);
                                            // Source single address
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &ADC_Result[0]);
                                            // Destination array address
  DMA0SZ = 0x03;                            // 3 conversions
  DMA0CTL = DMADT_4 + DMADSTINCR_3 + DMAEN + DMAIE;
  //This is the real "main" loop. It basically does the startup sweep, goes to the best duty cycle, and then
  //Preturbs and observes until the end of time.
  for (;;) {
	doSweep();
	TD0CCR2 = bestDuty;
	doMPPTTrack();
  }
}


//DMA ISR vector. Super basic. It only kicks the MSP430 out of low power mode so that we know that the conversion is complete.
#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case  2:
                                              // sequence of conversions complete
      __bic_SR_register_on_exit(CPUOFF);      // exit LPM
      break;                                  // DMA0IFG
    default: break;
  }
}
