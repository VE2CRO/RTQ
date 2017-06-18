//***************************************************************************
//* Title                : DTMF Generator
//* DESCRIPTION
//*
//***************************************************************************
#include "DTMF_Generator.h"
#include "SITE_ID.h"
//************************** SIN TABLE *************************************
// Samples table : one period sampled on 128 samples and
// quantized on 7 bit
//f(x)=63+63sin(2*pi*(x/128)) xE[0,127]
//**************************************************************************
// Variables {{{
const unsigned char auc_SinParam[128] = {
63,66,69,72,75,78,81,84,87,90,93,95,98,101,103,105,108,110,112,114,115,117,119,120,121,122,123,124,125,125,126,126,126,126,126,125,125,124,123,122,121,120,119,117,115,114,112,110,108,105,103,101,98,95,93,90,87,84,81,78,75,72,69,66,63,60,57,54,51,48,45,42,39,36,33,31,28,25,23,21,18,16,14,12,11,9,7,6,5,4,3,2,1,1,0,0,0,0,0,1,1,2,3,4,5,6,7,9,11,12,14,16,18,21,23,25,28,31,33,36,39,42,45,48,51,54,57,60
};

//***************************  x_SW  ***************************************
//Table of x_SW (excess 8): x_SW = ROUND(8*fa,b/f_sine_base)
//**************************************************************************

#define TIMER2_PERIOD   255
#define FOSC      8000000/4 // 4 clock cycles per instruction (2MHz effective)
#define SINE_PTS    128
#define PWM_FREQ    (FOSC/(TIMER2_PERIOD+1))
#define BASE_FREQ    (PWM_FREQ / SINE_PTS)  // Slowest synthesizable period (approx 60Hz)

//low frequency (row)
//697hz  ---> x_SW = 23
//770hz  ---> x_SW = 25
//852hz  ---> x_SW = 27
//941hz  ---> x_SW = 31

// Corrected each frequency. Actual frequency was measured on an oscilloscope and
// the correction factor was calculated based on the difference between the actual
// frequency and the target frequency.
const unsigned char auc_frequencyL[4] = {
  (697/BASE_FREQ)*8*1.03,
  (770/BASE_FREQ)*8*1.05,
  (852/BASE_FREQ)*8*1.07,
  (941/BASE_FREQ)*8*1.02};

#define MORSE_FREQ (440/BASE_FREQ)*8;

//high frequency (coloun)
//1209hz  ---> x_SW = 40
//1336hz  ---> x_SW = 44
//1477hz  ---> x_SW = 48
//1633hz  ---> x_SW = 54

const unsigned char auc_frequencyH[4] = {
  (1209/BASE_FREQ)*8*1.03,
  (1336/BASE_FREQ)*8*1.04,
  (1477/BASE_FREQ)*8*1.01,
  (1633/BASE_FREQ)*8*1.04};

//**************************  global variables  ****************************
unsigned char x_SWa = 0x00;               // step width of high frequency
unsigned char x_SWb = 0x00;               // step width of low frequency
unsigned int16  i_CurSinValA = 0;           // position freq. A in LUT (extended format)
unsigned int16  i_CurSinValB = 0;           // position freq. B in LUT (extended format)
unsigned int  i_TmpSinValA;               // position freq. A in LUT (actual position)
unsigned int  i_TmpSinValB;               // position freq. B in LUT (actual position)
// variables }}}
#INT_RTCC
void timer0interrupt(void) { // {{{
  int in_b;
  if (sec) {
    sec--;
  } else {
    in_b = (input_b() >> 5) & 0x07;
    COR_IN = (sCOR)(in_b^Polarity);  // Refresh COR status each second.
    COR_FLAG=1;
    sec = 31;
  }
  if ( min ) {
    min--;
  } else {
    PTTTimeOutFlag=1;
    min=1831;
  }
//  min = 1831; // 915.527 TMR0 interrupts per minute @4MHz.
    
} // }}}
// INT_RB {{{
#INT_RB
void cor_interrupt(void) {
  int in_b;
  if ( input(PIN_A4) && !DTMF.Strobe) {
    DTMF.Key = input_a() & 0xF; // Keep only 4 bits.
    DTMF.Strobe = 1;
    if ( DTMF.Key == 0x0A ) { // Digit 0 --> Swap with "D"
      DTMF.Key = 0x00;
    } else if (DTMF.Key == 0x00) { // Digit "D" --> Swap with "0"
      DTMF.Key = 0x0A;
    }
  }
  in_b = (input_b() >> 5) & 0x07;
  COR_IN = (sCOR)(in_b^Polarity);  
  COR_FLAG = 1;
} // }}}
//**************************************************************************
// Timer overflow interrupt service routine
//**************************************************************************
// INT_TIMER2 {{{
#INT_TIMER2
void timer2interrupt(void) {
int DUTY_CYCLE;
  if (x_SWa == 0 && x_SWb == 0){
    set_pwm1_duty(255);
  }else{
  if ( x_SWa ) {
    i_CurSinValA += x_SWa;       
  } else {
    i_CurSinValA = 0;
  }
  if (x_SWb ) {
      i_CurSinValB += x_SWb;
  } else {
      i_CurSinValB = 0;
  }
  i_TmpSinValA  =  (char)(((i_CurSinValA)>>3)&0x7F); 
  i_TmpSinValB  =  (char)(((i_CurSinValB)>>3)&0x7F);
  //DUTY_CYCLE = (auc_SinParam[i_TmpSinValA] + (auc_SinParam[i_TmpSinValB]-(auc_SinParam[i_TmpSinValB]>>2)));
  DUTY_CYCLE = auc_SinParam[i_TmpSinValA];
  DUTY_CYCLE += auc_SinParam[i_TmpSinValB];
  set_pwm1_duty(DUTY_CYCLE); // Half-way
  }  
} // }}}
//**************************************************************************
// MORSE functions 
//**************************************************************************
void dit (void) { // {{{
    unsigned int16 delay;
    delay = DitDelay * MORSE_MULTIPLIER_ISR_ON;
    //  Generate tone
    x_SWa = 0;
    x_SWb = MORSE_FREQ;
    init_pwm();
    delay_ms(delay);
    stop_pwm();
    delay_ms(DitDelay * MORSE_MULTIPLIER_ISR_OFF);
}  // }}}
void dah (void) { // {{{
    unsigned int16 delay;
    delay = DAH_DURATION_RATIO * DitDelay * MORSE_MULTIPLIER_ISR_ON;
    //  Generate tone
    x_SWa = 0;
    x_SWb = MORSE_FREQ;
    init_pwm();
    delay_ms(delay);
    stop_pwm();
    delay_ms(DitDelay * MORSE_MULTIPLIER_ISR_OFF);
} // }}}
void morse(int c) { // {{{
  int mc;
  int x;

  ActivateRXRelays(0);
  mc = cMorseChar[c]; 
  
  for(x=0;x<4;x++) {
    switch(mc & 0xc0) { // Check two MSB's
      case(0x40):
        dit();
        break;
      case(0x80):
        dah();
        break;
      default:
        break;
    }
    mc = mc << 2; // Shift two MSB's out and continue with next ones
  }
  if ( c < 10 ) { // Digits --> add the 5th dit or dah.
    if ( c < 5 ) {
      dah();
    }
    else {
      dit();
    }
  }
  delay_ms(DAH_DURATION_RATIO * DitDelay * MORSE_MULTIPLIER_ISR_OFF);
  return;
} // }}}

//**************************************************************************
// Initialization
//**************************************************************************
// init_pins {{{
void init_pins(void) {
  set_tris_a(0x1F);
  set_tris_b(0xE0);  
  set_tris_c(0b010001000);
  output_bit(PTT0_PIN,0);
  output_bit(PTT1_PIN,0);
  output_bit(PTT2_PIN,0);
      UpdatePTT=0;
  output_bit(AUX0_PIN,0);
  output_bit(AUX1_PIN,0);
  enable_interrupts(INT_RB5|INT_RB6|INT_RB7);
  ActivateRXRelays(0);
  // debug pins
} // }}}
// init {{{
void init ( void ) 
{
  // Setup TIMER0 as the main delay counter
  //OPTION = 0b0111;  // prescale by 256
  sec=0;
  min=0;
  PgmMode=0;
  setup_timer_0 (RTCC_DIV_256|RTCC_INTERNAL);
  set_timer0(0);
  CALIBRATE_DTMF_FLAG = 0;
  enable_interrupts(INT_RTCC);
  
  // Set up DTMF and functions tables;
        DTMF = (sDTMF)0;
  // Program PgmMode function:
  // * 9 9 
        Function[FUNCTION_NUM].Digit[0].Key     = 0x0D;
        Function[FUNCTION_NUM].Digit[0].Strobe  = 1;
        Function[FUNCTION_NUM].Digit[0].LastKey = 0;
        Function[FUNCTION_NUM].Digit[1].Key     = 9;
        Function[FUNCTION_NUM].Digit[1].Strobe  = 1;
        Function[FUNCTION_NUM].Digit[1].LastKey = 0;
        Function[FUNCTION_NUM].Digit[2].Key     = 9;
        Function[FUNCTION_NUM].Digit[2].Strobe  = 1;
        Function[FUNCTION_NUM].Digit[2].LastKey = 1;
        Function[FUNCTION_NUM].Digit[3]         = (sDTMF)0;
        Function[FUNCTION_NUM].PgmPtr           = 8;
} // }}}
// init_pwm {{{
void init_pwm (void)
{
  disable_interrupts(INT_RTCC);
  //setup_ccp1(CCP_PWM | CCP_PULSE_STEERING_C,CCP_PWM_H_H);
  //setup_ccp1(CCP_PWM | CCP_PULSE_STEERING_C | CCP_PULSE_STEERING_SYNC);
  CCP1CON = 0x0C;
  PSTRCON = 0x04;
  setup_timer_2 (T2_DIV_BY_1,TIMER2_PERIOD,1);
  enable_interrupts(INT_TIMER2);
  output_drive(PIN_C3);
} // }}}
// stop_pwm {{{
void stop_pwm (void) {
  setup_ccp1(CCP_OFF);
  disable_interrupts(INT_TIMER2);
  enable_interrupts(INT_RTCC);
  output_float(PIN_C3);
} // }}}
// ProcessCorFlag {{{
void ProcessCorFlag(void) {
  //COR_IN = (sCOR)(((input_b() >> 4)&0x07)^Polarity);
  ActivateRXRelays((int)COR_IN);
}
// ProcessCorFlag }}}
// ActivateRXRelays(int) {{{
void ActivateRXRelays(int cor_in) {
  int ActiveRelay;
    int ActiveCOR;
    int1 NewLocalCOR; // High when a new cor arrives on a local radio
    int1 NewCOR,NoCOR;
    // Only RX on radios 2 and 3 can override a relay.
    // If cor_in==0 then also do processing (disable relays)
    ActiveCOR = cor_in & Enable & 0x02; // High for COR's that are enabled
    NewCOR=!(ActiveRelay & ActiveCOR);
    NewLocalCOR=!(ActiveRelay & ActiveCOR & 0x06);
    NoCOR=!(ActiveCOR);

//    if ( (ActiveRelay < 0x02) || NoCOR || NewCOR) { 
// JL  2016-08-21 Change for 4 for Priority on Radio 2
    if ( (ActiveRelay < 0x04) || NoCOR || NewCOR) {  
      if((ActiveCOR & 0x04) || (NoCOR && (cor_in&0x04))) {
      output_bit(RX_EN1,0);
      output_bit(RX_EN0,0);
      output_bit(RX_EN2,1);
      ActiveRelay=4;
      if ( Enable & 0x04) {
        if( NewLocalCOR ) {
          PTT=RX2_PTT & Enable;
          PTTTimeOutTimer = PTT2TimeOut;
        }
      } else {
        PTT=0;
      }
      UpdatePTT=1;
    } else if ((ActiveCOR & 0x02)||(NoCOR && (cor_in&0x02))) {
      output_bit(RX_EN2,0);
      output_bit(RX_EN0,0);
      output_bit(RX_EN1,1);
      ActiveRelay=2;
      if ( Enable & 0x02 ) {
        if ( NewLocalCOR ) {
          PTT=RX1_PTT & Enable;
          PTTTimeOutTimer = PTT1TimeOut;
        }
      } else {
        PTT=0;
      }
      UpdatePTT=1;
    } else if((ActiveCOR & 0x01)||(NoCOR && (cor_in&0x01))) {
      output_bit(RX_EN2,0);
      output_bit(RX_EN1,0);
      output_bit(RX_EN0,1);
      ActiveRelay=1;
      if ( Enable & 0x01 ) {
        if ( NewCOR ) {
          PTT=RX0_PTT & Enable;
          PTTTimeOutTimer = PTT0TimeOut;
        }
      } else {
        PTT=0;
      }
      UpdatePTT=1;
    } else {
      output_bit(RX_EN2,0);
      output_bit(RX_EN1,0);
      output_bit(RX_EN0,0);
      ActiveRelay = 0;
      PTT=0;
      UpdatePTT=1;
    }
  }
}
// }}}
// sync -- Synchronize RAM--EEPROM--FactoryDefaults{{{
void sync(int * reg,int ee,int dv,int action) {
int ee_data;

  if ( action == SYNC_DEFAULT ) {
    *reg = dv;
  } else {
    ee_data = read_eeprom(ee);
    if ( action == SYNC_RAM ) {
      *reg = ee_data;
    } else
    if ( action == SYNC_EEPROM ) {
      if ( ee_data != *reg ) {
        write_eeprom(ee,*reg);
      }
    }
  }
}
// }}}
// sync_data {{{
void sync_data(int action) {
// Action: 
// 0 --> Get data from EEPROM
// 1 --> Save RAM data into EEPROM
// 2 --> Initialize RAM with factory defaults
  int x,d;
  int Default;
  int Instr;
  int PgmPtr;
  int ee_ptr;

  ee_ptr = 0;
// Initialize function defaults 
// EE locations 0 : 39 (0x00 - 0x27)
  for(x=0;x<8;x++) {
    for(d=0;d<4;d++) {
      Default = FunctionDefaults[x][d];
      sync(&Function[x].Digit[d],ee_ptr,Default,action);
      ee_ptr++;
    }
    PgmPtr = FunctionDefaults[x][4];
    sync(&Function[x].PgmPtr,ee_ptr,PgmPtr,action);
    ee_ptr++;
  }
// initialize Program defaults
// ProgramDefaultsNum : 8
// 8 ints per program
// EE locations  32 : 95
// EE locations  40:103 (0x28:0x67)
  for(x=0;x<ProgramDefaultsNum;x++) {
    for(d=0;d<8;d++) {
    Instr = ProgramDefaults[x][d];
      sync(&Program[x].Instruction[d],ee_ptr,Instr,action);
      ee_ptr++;
    }
  }
// Polarity
// EE location : 104
    sync(&Polarity  ,ee_ptr,DEFAULT_POLARITY,action); ee_ptr++; // EE location 104 (0x68)
    sync(&Enable    ,ee_ptr,DEFAULT_ENABLE  ,action); ee_ptr++; // EE location 105 (0x69)
    sync(&RX0_PTT   ,ee_ptr,DEFAULT_RX0_PTT ,action); ee_ptr++; // EE location 106 (0x6A)
    sync(&RX1_PTT   ,ee_ptr,DEFAULT_RX1_PTT ,action); ee_ptr++; // EE location 107 (0x6B)
    sync(&RX2_PTT   ,ee_ptr,DEFAULT_RX2_PTT ,action); ee_ptr++; // EE location 108 (0x6C)
    sync(&SiteID    ,ee_ptr,SITE_ID         ,action); ee_ptr++; // EE location 109 (0x6D)
    sync(&MorseID[0],ee_ptr,MORSE_ID0      ,action); ee_ptr++;   // EE location 110 (0x6E)
    sync(&MorseID[1],ee_ptr,MORSE_ID1      ,action); ee_ptr++;   // EE location 111 (0x6F)
    sync(&MorseID[2],ee_ptr,MORSE_ID2      ,action); ee_ptr++;   // EE location 112 (0x70)
    sync(&MorseID[3],ee_ptr,MORSE_ID3      ,action); ee_ptr++;   // EE location 113 (0x71)
    sync(&MorseID[4],ee_ptr,MORSE_ID4      ,action); ee_ptr++;   // EE location 114 (0x72)
    sync(&MorseID[5],ee_ptr,MORSE_ID5      ,action); ee_ptr++;   // EE location 115 (0x73)
    sync(&PWM_PTT    ,ee_ptr,DEFAULT_PWM_PTT ,action); ee_ptr++; // EE location 116 (0x74)
    sync(&PTT0TimeOut,ee_ptr,DEFAULT_PTT0TO  ,action); ee_ptr++; // EE location 117 (0x75)
    sync(&PTT1TimeOut,ee_ptr,DEFAULT_PTT1TO  ,action); ee_ptr++; // EE location 118 (0x76)
    sync(&PTT2TimeOut,ee_ptr,DEFAULT_PTT2TO  ,action); ee_ptr++; // EE location 119 (0x77)
    for (x=0;x<16;x++) {
      sync(&DTMF_OSCTUNE[x] ,ee_ptr,0,action); ee_ptr++; // EE locations 120:135 (0x78:0x87)
    }
}
// }}}
//
//**************************************************************************
// MAIN
// Read keypad for tone to generate.
// fix x_SWa and x_SWb
//**************************************************************************
// main loop {{{
void main (void)
{
  if ( read_eeprom(0) == 0x0FF ) {
	  sync_data(SYNC_DEFAULT);
	} else {
		sync_data(SYNC_RAM);
	}
  enable_interrupts(GLOBAL);
  enable_interrupts(INT_RA4);
  OSCTUNE=0x00;
  clearDTMFArray();
  init();
  init_pins();

  // The calibrateDigit function updates the DTMF_OSCTUNE array which is handled by the RegPtrArray.
  // The calibrated values can be stored in EEPROM.
  //sync_data(SYNC_EEPROM);
  
  while(1) {
    if (COR_FLAG) {
      ActivateRXRelays((int)COR_IN);
      //ProcessCorFlag();
      COR_FLAG = 0;
      // Clear DTMF array if COR_IN is empty
      if ( ! (int)COR_IN ) {
        clearDTMFArray();
      }
    }
    if (UpdatePTT) {
      update_PTT(0,PTT_NORMAL);
      UpdatePTT=0;
    }
    // The DTMF variable is set by the interrupt
    if ( DTMF.Strobe ) {
      // Collect DTMFs inside aDTMF
      if(pDTMF <= &aDTMF[DTMFArraySize-1]) {
        // pDTMF is the array pointer for the next available DTMF index
        pDTMF->Key = DTMF.Key;
        pDTMF->Strobe = 1;
        pDTMF++;
	  	}
      DTMF.Strobe=0;
      if ( PgmMode ) {
        if ( DTMF.Key == ds ) { // "*" has been pressed
          clearDTMFArray();
        }
        // PGM mode requires 5 digits
        // <OP><REG1><REG0><VAL1><VAL0># 
        if ( DTMF.Key == dp ) {
          RunPgmMode();
        }
      } else {
        ProcessDTMF(); // Check for functions, even if "*" is pressed
	 		}
      if ( DTMF.Key == ds ) { // "*" has been pressed.
        clearDTMFArray();
      }
    }
    if ( PTTTimeOutFlag ) { // Generated once per minute by RTC
      PTTTimeOutFlag=0;
      if ( COR_IN.RX2 ) {  
        DoTimeOut(PTT2TimeOut);
      } else if ( COR_IN.RX1 ) {  
        DoTimeOut(PTT1TimeOut);
      } else if ( COR_IN.RX0 ) {
        DoTimeOut(PTT0TimeOut);
      }
    }
    if ( CALIBRATE_DTMF_FLAG ) {
	delay_ms(1250);
	CalibrateDTMF();
	CALIBRATE_DTMF_FLAG = 0;
    }
  }
}
// main loop }}}
void DoTimeOut(int TOreg) { // {{{
  if ( TOreg ) {
    if ( PTTTimeOutTimer ) {
      PTTTimeOutTimer--;
    } else {
		  PTT=0;
      UpdatePTT=1;
    }
  } 
}// }}}
// PTT function {{{
void update_PTT (int extraPtt,int override) {
    int1 PttVal;
    int lPTT;
    if ( override ) {
      lPTT = extraPtt;
    } else {
      lPTT = PTT | extraPtt;
    }
    if ( Enable & 0x01 ) {
      PttVal = ((lPTT&0x01)!=0);
    } else {
      PttVal = 0;
    }
    output_bit(PTT0_PIN,PttVal);
    if ( Enable & 0x02 ) {
      PttVal = ((lPTT&0x02)!=0);
    } else {
      PttVal = 0;
    }
    output_bit(PTT1_PIN,PttVal);
    if (Enable & 0x04 ) {
      PttVal = ((lPTT&0x04)!=0);
    } else {
      PttVal = 0;
    }
    output_bit(PTT2_PIN,PttVal);
}
// }}}
// clearDTMFArray {{{
void clearDTMFArray(void) {
  int x;
  // Don't bother if the array is already clear
  if ( pDTMF > &aDTMF[0] ) {
    for(x=0;x<DTMFArraySize-1;x++) {
      aDTMF[x] = (sDTMF)0;
    }
  }
  pDTMF=&aDTMF[0];
}
// }}}
// sendDTMF {{{
void sendDTMF(int digit) {
  int iDigit;
  OSCTUNE = DTMF_OSCTUNE[digit&0x0F];
  iDigit = DTMF_map[digit&0x0F];
  x_SWa = auc_frequencyL[(iDigit>>2)&0x03];
  x_SWb = auc_frequencyH[iDigit&0x03];
  disable_interrupts(INT_RA4);
  ActivateRXRelays(0);
  init_pwm();
  delay_ms(DTMF_LEN);
  stop_pwm();
  enable_interrupts(INT_RA4);
  delay_ms(DTMF_LEN);
  OSCTUNE = 0; // Reset it back to manufacturing settings
}
// sendDTMF }}}
// sendDTMFCal {{{ Only used during calibration
void sendDTMFCal(int digit,int1 action) {
  int iDigit;
  if ( action ) { // start
		  iDigit = DTMF_map[digit&0x0F];
		  x_SWa = auc_frequencyL[(iDigit>>2)&0x03];
		  x_SWb = auc_frequencyH[iDigit&0x03];
		  disable_interrupts(INT_RA4);
		  init_pwm();
  } else { // stop
		  stop_pwm();
		  enable_interrupts(INT_RA4);
  }
}
// sendDTMF }}}
// ProcessDTMF {{{
int ProcessDTMF(void) {
  int fn,d;
  //sDTMF cDTMF;
  int1 DigitMatch,StrobeMatch;
  //int1 FunctionLast;
  int program;
  int1 FunctionMatch;
  int FKey,UKey;
  int1 FStrobe,UStrobe,FLast;
  
  for (fn=0;fn<FUNCTION_NUM+1;fn++) {
    FunctionMatch=1;
    d=0;
    while(d<4 && FunctionMatch) {
    	FKey = Function[fn].Digit[d].Key;
      UKey = aDTMF[d].Key;
      FStrobe = Function[fn].Digit[d].Strobe;
      UStrobe = aDTMF[d].Strobe;
      FLast = Function[fn].Digit[d].LastKey;
      DigitMatch = (UKey == FKey);
      StrobeMatch = (UStrobe == FStrobe);
      if ( !DigitMatch ) {
      	FunctionMatch=0;
      }
			if ( FunctionMatch && StrobeMatch && FLast) {
		  	program=Function[fn].PgmPtr;
  			switch(program) {
  			case PGM_MODE:
  				PgmMode = 1;
  				update_PTT(PWM_PTT,PTT_OVERRIDE);
				delay_ms(750);
  				morse(MCHAR('p'));
	  			update_PTT(0,PTT_NORMAL);
				delay_ms(250);
  				break;
  			default:
  				delay_ms(INIT_PAUSE);
  				ExecutePgm(program);
  				break;
  			}
  	 		clearDTMFArray(); // Controller can start accept commands immediately
  		}
  			d++;
  	}
	}
  // Fix this function. It does nothing right now
  return 0;
}
// }}}
// ExecutePgm {{{
void ExecutePgm(int pgm) {
  int i;
  int Op;
  int Arg;
  int PgmCount;

  ExecPgm.PgmPtr=pgm&0x07;
  ExecPgm.RunPgm=1;
  NextPgm = 0;
  PgmCount = 0;

  while(ExecPgm.RunPgm) {  
    i=0;
    NextPgm = 0;
    while(i<8 && ExecPgm.RunPgm) {
      Op = Program[ExecPgm.PgmPtr].Instruction[i].Op;
      if(Op & (MORSE>>4)) {
        // Morse character; only keep 6 bits
        Op = MORSE>>4; // Sanitize the operator for ExecOp function
        Arg = (int)Program[ExecPgm.PgmPtr].Instruction[i] & 0x3F;
      } else {
        Arg = Program[ExecPgm.PgmPtr].Instruction[i].Arg;
      }
      ExecOp(Op,Arg);  
      i++;
    }
    PgmCount++;
    if ( NextPgm.RunPgm && (PgmCount < 4 )) {
        ExecPgm = NextPgm;
    }
  }
}
// }}}
// ExecOp {{{
void ExecOp(int op,int arg) {
    int larg;
    int p;
    int lop;

    lop=(op&0x0F)<<4;

  larg=arg&0x07;
  switch(lop) {
    case DONE: 
      ExecPgm.PgmPtr=0; 
      ExecPgm.RunPgm=0; 
    break;
    case SEND_DTMF: 
      //sendDTMF(DTMF_map[arg&0x0F]); 
            // Activate PTT to selected radios in case COR falls and drops all PTTs
            // after DTMF starts to send.
      update_PTT(PWM_PTT,PTT_OVERRIDE);
      delay_ms(750);
    sendDTMF(arg & 0x0F); 
      delay_ms(250);
//            update_PTT(0,PTT_NORMAL);
    break;
    case MORSE: 
            // Activate PTT to selected radios in case COR falls and drops all PTTs
            // after DTMF starts to send.
      update_PTT(PWM_PTT,PTT_OVERRIDE);
      if ( arg <= 36 ) {
        morse(arg);
      }
//            update_PTT(0,PTT_NORMAL);
    break;
    case PAUSE: 
// Max pause value is 7.
// Add +8 if you want to reset the PTT to normal settings during the pause.
      if ( arg & 0x08 ) {
        update_PTT(0,PTT_NORMAL);
      }
      larg=(arg&0x07)<<2; // arg in seconds
      for(p=0;p<larg;p++) {
        delay_ms(250); 
      }
    break;
    case UPDATE_PWM_PTT: 
      PWM_PTT=larg; 
      update_PTT(PWM_PTT,PTT_OVERRIDE);
    break;
    case UPDATE_ENABLE: 
      Enable=larg; 
    break;
    case BEEP: 
            // Activate PTT to selected radios in case COR falls and drops all PTTs
            // after DTMF starts to send.
            update_PTT(PWM_PTT,PTT_OVERRIDE);
            delay_ms(750);
      		sendDTMF(larg); 
            delay_ms(250);
//            update_PTT(0,PTT_NORMAL);
    break; // Just send a tone
    case JUMP: 
      ExecPgm.RunPgm = 0; // Stop the current program
      NextPgm.PgmPtr = larg; 
      NextPgm.RunPgm = 1; 
    break;
    case PGM: 
      PgmMode = 1; 
    break;
  }
}
// }}}
// RunPgmMode() {{{
//        [*] 'A' <OP><REG1><REG0><VAL1><VAL0># 
// OP :
// 0 --> Exit
// 1 --> ProgramFunction
// 2 --> ProgramInstruction
// 3 --> SyncData
// 4 --> ProgramReg
// 5 --> EchoRegister
// 6 --> CalibrateDTMF
//
void RunPgmMode(void) {
  int PgmFunction;
  int1 Strobe;
  int1 PgmModePrefix;
  int ErrorCode;
  
  PgmFunction=aDTMF[PGM_OP].Key;
  Strobe=aDTMF[PGM_OP].Strobe;
  PgmModePrefix = (aDTMF[0].Key == da);
  ErrorCode = 0;
  if ( Strobe && PgmModePrefix ) {
    switch(PgmFunction) {
      case(0):
        PgmMode=0;
          ErrorCode = MCHAR('d');
        break;
      case(1):
        if (ProgramFunction())
          ErrorCode = 0x00;
				else
          ErrorCode = MCHAR('p');
        break;
      case(2):
		    if ( ProgramInstruction() )
          ErrorCode = 0x00;
				else
          ErrorCode = MCHAR('i');
        break;
      case(3):
         if (syncData())
          ErrorCode = 0x00;
				else
          ErrorCode = MCHAR('s');
        break;
      case(4):
  	    if (ProgramReg())
          ErrorCode = 0x00;
				else
          ErrorCode = MCHAR('r');
        break;
      case(5):
	      if ( EchoRegister())
          ErrorCode = 0x00;
				else
          ErrorCode = MCHAR('e');
        break;
	case(6):
		CALIBRATE_DTMF_FLAG=1;
		ErrorCode = MCHAR('c');
	break;
    }
  }
  if ( ErrorCode ) {
      update_PTT(PWM_PTT,PTT_OVERRIDE);
      delay_ms(750);
      morse(ErrorCode);
      delay_ms(250);
      update_PTT(0,PTT_NORMAL);
  }
}
// }}}
int1 ProgramFunction(void) { // {{{
// aDTMF index         0 1 2 3 4 5 6 7 8
// ProgramFunction : * A a b c w x y z #
// * (clear)           
//  (a=1) --> ProgramFunction
//   b=1  --> set function pwd
//   b=2  --> set function instruction pointer
//   c=<fnPtr> --> Which function to program
//   w x y z --> Function pwd keys.
  int FnType,FnPtr;
  int LastStrobe;
  int Key;
  int d;
  int1 Strobe;
  int1 ValidKeys;
  int1 Error;

  FnType=aDTMF[PGMFN_TYPE].Key;
  FnPtr =aDTMF[PGMFN_PTR].Key;

  ValidKeys=ValidKeyRange(PGMFN_TYPE,PGMFN_ARG0); // Check that keys were strobed

  if ( ValidKeys ) {
    Error=0;
    if(FnType == 1) {
      for(d=0;d<4;d++) {
        if(ValidKey(d+PGMFN_ARG0)) {
          Strobe=aDTMF[d+PGMFN_ARG0].Strobe;
          Key   =aDTMF[d+PGMFN_ARG0].Key;
          Function[FnPtr].Digit[d].Key     = Key;
          Function[FnPtr].Digit[d].Strobe  = Strobe;
          Function[FnPtr].Digit[d].LastKey = 0;
          LastStrobe=d;
        } else {
          Function[FnPtr].Digit[d].Key=0;
          Function[FnPtr].Digit[d].Strobe=0;
          Function[FnPtr].Digit[d].LastKey=0;
        }
      }
      Function[FnPtr].Digit[LastStrobe].LastKey=1;
    } 
    else if ( (FnType == 2) && ValidKey(PGMFN_ARG0)) {
      Function[FnPtr].PgmPtr=aDTMF[PGMFN_ARG0].Key;
    } else {
      Error = 1;
    }
  } else {
    Error=1;
  }
  return(Error);
} // }}}
int1 ProgramReg(void) { // {{{
// aDTMF index         0 1 2 3 4 5 6 7
// ProgramReg      : * A a b c d e f #
// ProgramReg      : * a b c d e f #
// * (clear)           
//  (a=4)      --> ProgramReg
//  {b,c}      --> Register address (See RegPtrArray table in .h file)
//  {d,e,[f]}  --> Register value
  int Reg;
  int Val;
  int *pReg;
  int tmp;
  int1 ValidKeys;
  int1 Error;

  Error = 0;
  Reg =aDTMF[PGMREG_ADD0].Key * 10;
  Reg+=aDTMF[PGMREG_ADD1].Key;
  if (ValidKey(PGMREG_VAL2)) {
    tmp=aDTMF[PGMREG_VAL0].Key;
    if ( tmp == 1 ) { // We only support numbers up to 255
      Val  = 100;
    } else if (tmp == 2 ) {
      Val  = 200;
    } else {
      Val = 0;
      Error=1;
    }
    Val += aDTMF[PGMREG_VAL1].Key*10;
    Val += aDTMF[PGMREG_VAL2].Key;
  } else {
    Val  = aDTMF[PGMREG_VAL0].Key*10;
    Val += aDTMF[PGMREG_VAL1].Key;
  }
  ValidKeys=ValidKeyRange(PGMREG_ADD0,PGMREG_VAL1); // Check that keys were strobed
  if ( ValidKeys ) {
    if ( Reg < sizeof(RegPtrArray)) {
      pReg=RegPtrArray[Reg];
      *pReg=Val;
    } else {
      Error = 1;
    }
  } else {
    Error=1;
  }
  return(Error);
} // }}}
int1 ProgramInstruction(void) { // {{{
// aDTMF index            0 1 2 3    4     5 6
// ProgramInstruction : * A a b c <op> <arg> #
// * (clear)           
//  (a=2) --> ProgramInstruction
//   b=<fnPtr> --> Which function to program
//   c=<InstIndex> --> Which instruction index to program
//   <op> <arg> --> Operator and argument
//   Optionally, the arguments may be specified in decimal from 0 to 255:
//   * A a b c d2 d1 d0 #
  int PgmPtr,InstPtr;
  int Op,Arg;
//  int1 ValidKeys;
  int1 Error;

  PgmPtr  = aDTMF[PGMINS_FPTR].Key;
  InstPtr = aDTMF[PGMINS_IPTR].Key;
  Op      = aDTMF[PGMINS_OP].Key&0x0F;
  Arg     = aDTMF[PGMINS_ARG0].Key&0x0F;

  if ( ValidKeyRange(PGMINS_FPTR,PGMINS_ARG1) ) { // First method : Arguments are in decimal from 0 to 255
    Error=0;
    Arg = aDTMF[PGMINS_ARG1].Key;
    Arg += aDTMF[PGMINS_ARG0].Key*10;
    Arg += aDTMF[PGMINS_OP].Key*100;
    Program[PgmPtr].Instruction[InstPtr] = Arg;
  } 
  else if ( ValidKeyRange(PGMINS_FPTR,PGMINS_ARG0) ) {
    Error=0;
    Program[PgmPtr].Instruction[InstPtr].Op = Op;
    Program[PgmPtr].Instruction[InstPtr].Arg = Arg;
  } else {
    Error=1;
  }
  return(Error);
} // }}}
int1 EchoRegister (void) { // {{{
// aDTMF index            0 1 2 3 4 5  
// ProgramInstruction : * A 4 b c d #
// * (clear)           
//  (a=4) --> EchoRegister
//   b=<Type> 
//     1 --> Program register
//     2 --> Instruction register
//     3 --> RAM register
//   c=<Index> --> Pgm/Instruction ptr 
//   d=<Index> --> Which index to echo
//   <op> <arg> --> Operator and argument
  int RegType;
  int RegPtr;
  int Index;
  int Value;
  int HNibble,LNibble;
  int1 ValidKeys;
  int1 Error;

  RegType = aDTMF[ECHO_TYPE].Key&0x07;
  RegPtr  = aDTMF[ECHO_PTR].Key&0x07;
  Index   = aDTMF[ECHO_INDEX].Key&0x07;

  ValidKeys=ValidKeyRange(ECHO_TYPE,ECHO_INDEX); // Check that keys were strobed

  if ( ValidKeys ) {
    Error=0;
    switch(RegType) {
      case(0x01):
            Value=Function[RegPtr].Digit[Index].Key;
       break;
      case(0x02):
            Value=(int)Program[RegPtr].Instruction[Index];
       break;
      case(0x03): // When RegType is 3, argument c and d are used to specify which register.
      RegPtr =aDTMF[ECHO_PTR].Key*10;
      RegPtr+=aDTMF[ECHO_INDEX].Key;
      if ( RegPtr < sizeof(RegPtrArray) ) {
              Value=*RegPtrArray[RegPtr];
      } else {
              Error=1;
      }
    break;
      default:
        Value = 0;
      break;
    }
    if ( Value ) {
      update_PTT(PWM_PTT,PTT_OVERRIDE);
      delay_ms(750);
      HNibble=(Value&0xF0)>>4;
      if ( HNibble ) {
        morse(HNibble);
        delay_ms(3 * DitDelay * MORSE_MULTIPLIER_ISR_OFF);
      }
      LNibble=Value&0x0F;
      morse(LNibble);
      delay_ms(250);
      update_PTT(0,PTT_NORMAL);
    }
  } else {
    Error=1;
  }
  return(Error);
} // }}}
int1 syncData(void) { // {{{
// aDTMF index         0 1 2 3
// ProgramFunction : * A 3 x #
// 3 == SyncData function
// x == which sync type (0,1 or 2)
// 	0 -> RAM    <-- EEPROM
// 	1 -> EEPROM <-- RAM
// 	2 -> RAM    <-- DEFAULTS
  int1 ValidKeys;
  int SyncMethod;
  int1 Error;

  ValidKeys=ValidKey(SYNC_TYPE);
  SyncMethod=aDTMF[SYNC_TYPE].Key;
  if(ValidKeys && (SyncMethod>=0) && (SyncMethod<=2)) {
    Error=0;
    sync_data(SyncMethod);
  } else {
    Error=1;
  }
  return(Error);
} // }}}
int1 ValidKey(int index) { // {{{
  int1 strobe;
  if(index>=0 && (index <= sizeof(aDTMF))) {
    if(aDTMF[index].Strobe && (aDTMF[index].Key != dp)) {
		strobe=1;
	}else {
		strobe = 0;
	} 
  } else {
    strobe=0;
  }
  return(strobe);
} // }}}
int1 ValidKeyRange(unsigned int a,unsigned int b) { // {{{
  int key;
  int x;
  int1 valid;
  int1 strobe;

  if(b>=a && (a < sizeof(aDTMF))) {
    valid=1;
    for(x=a;x<=b;x++) {
      key=aDTMF[x].Key;
      strobe=aDTMF[x].Strobe;
      if(key==dp || !strobe) {
        valid=0;
      }
    }
  } else {
    valid=0;
  }
  return(valid);
} // }}}
int1 OSCTuneTest(int digit) { // {{{
	int dtmfIn;
	int retVal;
	sendDTMFCal(digit,START);
	delay_ms(70);
	retVal = 0;
	if(input(PIN_A4)) {
		dtmfIn = input_a() & 0x0F;
		if (dtmfIn == 0x0A) {
			dtmfIn = 0x00;
		} else if (dtmfIn == 0x00) {
			dtmfIn = 0x0A;
		}
		if (dtmfIn == digit) { // DTMF locked?
			retVal = 1;
		}
	}
	sendDTMFCal(digit,STOP);
	delay_ms(10);
	return(retVal);
} // }}}
int1 calibrateDigit(int digit) { // {{{
	signed int lLimit,uLimit,tuneTemp,oscval;
	int1 tuneTempValid;
	int cnt;
	int1 retval,FoundOscVal;

	update_PTT(0,PTT_OVERRIDE);
	ActivateRXRelays(0);

	lLimit=uLimit=0;
	cnt=0;
	update_PTT(0,PTT_OVERRIDE); // Disable all PTTs
	output_bit(AUX0_PIN,0);
	output_bit(AUX1_PIN,0);
	// find a good OSCTUNE value to begin searching for up/down limits
	cnt=0;
	FoundOscVal=0;
	oscval=0;
	// Try the following OSCTUNE values : 0, -1, 1, -2, 2, -3, ...
	for(cnt=0;cnt<16;cnt++) {
		if ( !FoundOscVal ) {
			oscval = -oscval;
			OSCTUNE = oscval&0x1F;
			if ( OSCTuneTest(digit) ) {
				FoundOscVal=1;
			} else {
				oscval = -(oscval + 1);
			  OSCTUNE = oscval&0x1F;
				if ( OSCTuneTest(digit) ) {
					FoundOscVal = 1;
				}
			}
		} 	
	}
	if ( FoundOscVal ) {
		tuneTemp = oscval;
		tuneTempValid = 1;
	} else {
		tuneTemp = 0;
		tuneTempValid = 0;
	}
	// When we get out of this for loop, OSCTUNE should be set to a good value.
	// Next: Let's try to center OSCTUNE between the upper and lower limits.
	// go down from tuneTemp
	FoundOscVal=0;
	oscval = tuneTemp;
	while(!FoundOscVal && tuneTempValid) {
		output_bit(AUX0_PIN,1);
		output_bit(AUX1_PIN,0);
		lLimit = oscval;
		oscval--;
	  OSCTUNE = oscval&0x1F;
		if ( !FoundOscVal && !OSCTuneTest(digit) ) {
			FoundOscVal=1;
		} 
	}
	FoundOscVal=0;
	oscval = tuneTemp;	
	while(!FoundOscVal && tuneTempValid) {
		output_bit(AUX0_PIN,0);
		output_bit(AUX1_PIN,1);
		uLimit = oscval;
		oscval++;
	  OSCTUNE = oscval&0x1F;
		if ( !FoundOscVal && !OSCTuneTest(digit) ) {
			FoundOscVal=1;
		} 
	}
	if ( tuneTempValid ) {
		DTMF_OSCTUNE[digit] = ((uLimit + lLimit)/2)&0x1F  ;
		retval=1;
	} else {
		DTMF_OSCTUNE[digit] = 0; // Default value
		retval=0;
	}
	OSCTUNE = 0;
	output_bit(AUX0_PIN,0);
	output_bit(AUX1_PIN,0);
	// debug
	return(retval);
} // }}}
void CalibrateDTMF(void) { // {{{
  int CalSuccess;
  int d;
  // Calibrate all DTMF digits {{{
  CalSuccess=0;
  for(d=0;d<16;d++) {
 	 if(calibrateDigit(d)) {
		 CalSuccess++;
	 }
  }
  if ( CalSuccess == 16 ) {
	output_bit(AUX1_PIN,1);
  } else {
	output_bit(AUX1_PIN,0);
  }
  // Calibrate all DTMF digits }}}
} // }}}
