#include <stddef.h>
#include <16F690.h>

// DTMF Key maps {{{
#define d1 0x01
#define d2 0x02
#define d3 0x03
#define d4 0x04
#define d5 0x05
#define d6 0x06
#define d7 0x07
#define d8 0x08
#define d9 0x09
//#define d0 0x0a
#define d0 0x00
#define ds 0x0b
#define dp 0x0c
#define da 0x0d
#define db 0x0e
#define dc 0x0f
//#define dd 0x00
#define dd 0x0a
// }}}
// Morse parameters {{{
// DitDelay:
//     7  --> 100ms
#define DitDelay 7 
#define TIMER2_DELAY_FACTOR 1/6 // Based on experimentation with trace. 
#define MORSE_MULTIPLIER_ISR_ON 10 * TIMER2_DELAY_FACTOR
#define MORSE_MULTIPLIER_ISR_OFF 10 
#define DAH_DURATION_RATIO 3
#define MCHAR(c)	c-'a'+10
const int8 cMorseChar[] = {
	0b10101010, // 0 (dah dah dah dah dah)	0
	0b01101010, // 1 (dit dah dah dah dah)	1
	0b01011010, // 2 (dit dit dah dah dah)	2
	0b01010110, // 3 (dit dit dit dah dah)	3
	0b01010101, // 4 (dit dit dit dit dah)	4
	0b01010101, // 5 (dit dit dit dit dit)	5
	0b10010101, // 6 (dah dit dit dit dit)	6
	0b10100101, // 7 (dah dah dit dit dit)	7
	0b10101001, // 8 (dah dah dah dit dit)	8
	0b10101010, // 9 (dah dah dah dah dit)	9
	0b01100000, // a (dit dah)		10
	0b10010101, // b (dah dit dit dit)	11
	0b10011001, // c (dah dit dah dit)	12
	0b10010100, // d (dah dit dit)		13
	0b01000000, // e (dit)			14
	0b01011001, // f (dit dit dah dit)	15
	0b10100100, // g (dah dah dit) 		16
	0b01010101, // h (dit dit dit dit)	17
	0b01010000, // i (dit dit)		18
	0b01101010, // j (dit dah dah dah)	19
	0b10011000, // k (dah dit dah)		20
	0b01100101, // l (dit dah dit dit)	21
	0b10100000, // m (dah dah)		22
	0b10010000, // n (dah dit)		23
	0b10101000, // o (dah dah dah)		24
	0b01101001, // p (dit dah dah dit)	25
	0b10100110, // q (dah dah dit dah)	26
	0b01100100, // r (dit dah dit)		27
	0b01010100, // s (dit dit dit)		28
	0b10000000, // t (dah)			29
	0b01011000, // u (dit dit dah)		30
	0b01010110, // v (dit dit dit dah)	31
	0b01101000, // w (dit dah dah)		32
	0b10010110, // x (dah dit dit dah)	33
	0b10011010, // y (dah dit dah dah)	34
	0b10100101 // z (dah dah dit dit)	35
}; // }}}
// {{{ Program symbols
#define INIT_PAUSE 1500
#define DONE       0x00
#define SEND_DTMF  0x10
#define PAUSE      0x30
#define UPDATE_PWM_PTT 0x20
#define UPDATE_ENABLE  0x40
#define BEEP       0x50
#define JUMP       0x60
#define PGM	       0x70
#define MORSE      0x80
// }}}
// Function defaults {{{
#define LastDigit 32
#define Strb 16
#define START 1
#define STOP 0
// }}}
// Data synchronizing params {{{
#define SYNC_RAM	0
#define SYNC_EEPROM	1
#define SYNC_DEFAULT    2
// Data synchronizing params }}}

// structure definitions {{{
// sDTMF {{{
typedef struct { 
			int Key 	: 4;
			int Strobe	: 1;
			int LastKey	: 1;
			int UNUSED	: 2;
} sDTMF;

// sDTMF }}}

typedef struct {
	sDTMF Digit[4];
	int PgmPtr;
} sFunction;

typedef struct {
	int Arg : 4;
        int Op : 4;	
} sInstruction;

typedef struct {
	sInstruction Instruction[8];
} sProgram;

typedef struct {
    int PgmPtr  : 4;
    int RunPgm  : 1;
//    int Count   : 2;
} sNextPgm;

#define FUNCTION_NUM 8
#define PGM_NUM 8

#define PTT_OVERRIDE 1
#define PTT_NORMAL   0

// Special program pointers {{{
// PGM pointers 0:7 are user pgms.
// 8 --> PGM_MODE
// 9 --> Calibrate DTMF
#define PGM_MODE 8
#define CAL_DTMF 9
// PgmMode digit index
// 	      0  1     2      3      4      5    6
// Syntax : * A <OP> <REG1> <REG0> <VAL1> <VAL0> #
#define PGM_OP 1
#define PGM_REG1 2
#define PGM_REG2 3
#define PGM_VAL1 4
#define PGM_VAL2 5
// PgmFunction syntax:
// aDTMF index         0 1 2 3 4 5 6 7 8
// ProgramFunction : * A a b c w x y z #
#define PGMFN_TYPE 2
#define PGMFN_PTR 3
#define PGMFN_ARG0 4
#define PGMFN_ARG1 5
#define PGMFN_ARG2 6
#define PGMFN_ARG3 7
// ProgramInstruction
#define PGMINS_FPTR 2
#define PGMINS_IPTR 3
#define PGMINS_OP   4
#define PGMINS_ARG0  5
#define PGMINS_ARG1  6
// EchoRegister
#define ECHO_TYPE 2
#define ECHO_PTR 3
#define ECHO_INDEX 4
// SyncData
#define SYNC_TYPE 2
// ProgramReg
#define PGMREG_ADD0 2
#define PGMREG_ADD1 3
#define PGMREG_VAL0 4
#define PGMREG_VAL1 5
#define PGMREG_VAL2 6

// Special program pointers }}}

sFunction Function[FUNCTION_NUM+1];
sProgram Program[8];

int PTT;
int1 UpdatePTT;
int Enable;
//int PWM_PTT;
int PgmMode;
sNextPgm ExecPgm,NextPgm;

// Access like this:
//
// Program[0].Instruction[0].Op ==> Points to the operator
// Program[0].Instruction[0].Arg ==> Points to the argument
//

// sCOR {{{
typedef struct {
		int RX0 : 1;
		int RX1 : 1;
		int RX2 : 1;
} sCOR; 
// sCOR }}}
// structure definitions }}}

// Declare EEPROM & RAM variables {{{
// Volatile variables
volatile int1 COR_FLAG;
volatile sDTMF DTMF; // ISR writes to this register.
#define DTMFArraySize 10
sDTMF aDTMF[DTMFArraySize];
sDTMF *pDTMF;
volatile sCOR COR_IN;
int sec;
unsigned int16 min;
int1 PTTTimeOutFlag;
int1 CALIBRATE_DTMF_FLAG;

int Polarity; 
int RX0_PTT,RX1_PTT,RX2_PTT;
int PWM_PTT;
int SiteID;
int MorseID[6];
int PTT0TimeOut;
int PTT1TimeOut;
int PTT2TimeOut;
int PTTTimeOutTimer;
int DTMF_OSCTUNE[16];

                      			// RegPtr index     ROM address
const int* RegPtrArray[32]={&Polarity,   // 00      96  
                            &Enable,     // 01      97 
                            &RX0_PTT,   // 02      98
                            &RX1_PTT,   // 03      99
                            &RX2_PTT,   // 04     100
                            &SiteID,     // 05     101
                            &MorseID[0], // 06     102
                            &MorseID[1], // 07     103
                            &MorseID[2], // 08     104
                            &MorseID[3], // 09     105
                            &MorseID[4], // 10     106
                            &MorseID[5],  // 11     107
                            &PWM_PTT,     // 12     108
			    &PTT0TimeOut, // 13	    109
			    &PTT1TimeOut, // 14     110
			    &PTT2TimeOut,  // 15     111
			    &DTMF_OSCTUNE[0], // 16  112
			    &DTMF_OSCTUNE[1], // 17  113
			    &DTMF_OSCTUNE[2], // 18  114
			    &DTMF_OSCTUNE[3], // 19  115
			    &DTMF_OSCTUNE[4], // 20  116
			    &DTMF_OSCTUNE[5], // 21  117
			    &DTMF_OSCTUNE[6], // 22  118
			    &DTMF_OSCTUNE[7], // 23  119
			    &DTMF_OSCTUNE[8], // 24  120
			    &DTMF_OSCTUNE[9], // 25  121
			    &DTMF_OSCTUNE[10], // 26  122
			    &DTMF_OSCTUNE[11], // 27  123
			    &DTMF_OSCTUNE[12], // 28  124
			    &DTMF_OSCTUNE[13], // 29  125
			    &DTMF_OSCTUNE[14], // 30  126
			    &DTMF_OSCTUNE[15] // 31  127
};

// EEPROM and RAM declarator macros {{{
//typedef struct sRegInfo_t {
//	int8 iEEOffset;
//	int8 *ramPtr;
//} sRegInfo;
//const int8 EEPROM_regs=0;

//#define DeclareVar(name) \
//	int8 name; \
//	const int8 Offset_##name = EEPROM_regs; \
//	const int8 RegCountTemp = EEPROM_regs; \
//	#undef EEPROM_regs \
//	const int8 EEPROM_regs = (RegCountTemp + 1); \
//	#undef RegCountTemp


//#define GetEEOffset(name)	Offset_##name

// EEPROM and RAM declarator macros }}}

//DeclareVar(Instruction0)
//DeclareVar(Polarity)

//const sRegInfo RegInfo []= {
//	{GetEEOffset(Instruction0), &Instruction0}
//};

//const int RegInfoSize = sizeof(RegInfo)/sizeof(sRegInfo);
// Declare EEPROM & RAM variables }}}

// Function header declarations {{{
int1 ProgramFunction(void);
int1 ProgramInstruction(void);
int1 ProgramReg(void);
int1 EchoRegister(void);
int1 ValidKey(int index);
int1 ValidKeyRange(unsigned int a,unsigned int b);
int1 syncData(void);
void RunPgmMode(void);
void init_pwm(void);
void stop_pwm(void);
void sync_data(int type);
void update_PTT (int extraPTT, int override);
void DoTimeOut(int TOreg) ;
// ExecOp {{{
void ExecOp (int op,int arg);
// }}}
// sendDTMF {{{
void sendDTMF(int digit);
void ExecutePgm(int Program);
// sendDTMF }}}
// sendDTMF {{{
void sendDTMFCal(int digit);
void CalibrateDTMF(void);
int1 calibrateDigit(int digit);
int1 OSCTuneTest(int digit);
void ExecutePgm(int Program);
// sendDTMF }}}
// clearDTMFArray {{{
void clearDTMFArray(void);
// }}}
// ProcessDTMF(void) {{{
int ProcessDTMF(void);
// }}}
// ActivateRXRelays(val) {{{
void ActivateRXRelays(int val);
// }}}
//
// Function header declarations }}}

// Define DTMF digits {{{
#define DTMF_1 0x00
#define DTMF_2 0x01
#define DTMF_3 0x02
#define DTMF_A 0x03
#define DTMF_4 0x04
#define DTMF_5 0x05
#define DTMF_6 0x06
#define DTMF_B 0x07
#define DTMF_7 0x08
#define DTMF_8 0x09
#define DTMF_9 0x0A
#define DTMF_C 0x0B
#define DTMF_S 0x0C
#define DTMF_0 0x0D
#define DTMF_P 0x0E
#define DTMF_D 0x0F
// Define DTMF digits }}}

// Mapping from the DTMF receiver value to DTMF generator {{{
const int DTMF_map[] = {DTMF_0, // 0 --> Switched with (A)
				DTMF_1, // 1
				DTMF_2, //2
				DTMF_3, //3
				DTMF_4, //4
				DTMF_5, //5
				DTMF_6, //6
				DTMF_7, //7
				DTMF_8, //8
				DTMF_9, //9
				DTMF_D, //A --> Switched with (0)
				DTMF_S, //B
				DTMF_P, //C
				DTMF_A, //D
				DTMF_B, //E
				DTMF_C  //F
};
// }}}

// PWM variables {{{
#define DTMF_DELAY_RATIO 25/70 // 700ms according to Jonathan
#define DTMF_LEN     350 * DTMF_DELAY_RATIO
#define CCP_PWM_MODE 0x00
#define CCP_PWM_HH   0x0C
// PWM variables }}}

// Fuses {{{
#fuses INTRC
#fuses NOPROTECT
#fuses NOBROWNOUT
#fuses NOMCLR
#fuses NOCPD
#fuses NOWDT // WDT controlled by sw
#fuses NOPUT
#fuses NOFCMEN
#fuses NOIESO
#use delay(internal=8M,restart_wdt)
#byte OSCTUNE = 0x90
#byte CCP1CON = 0x17
#byte PSTRCON = 0x19D
// Fuses }}}

// fast_io settings {{{
#use fast_io(A)
#use fast_io(B)
#use fast_io(C)
#case 
// fast_io settings }}}

// Controller COR/RX/PTT/AUX pin assignments {{{
#define COR0 RB5
#define COR1 RB6
#define COR2 RB7

#define RX_EN0	PIN_C0
#define RX_EN1	PIN_C1
#define RX_EN2	PIN_C2

#define PTT0_PIN PIN_C4
#define PTT1_PIN PIN_C5
#define PTT2_PIN PIN_C6

#define AUX0_PIN PIN_B4
#define AUX1_PIN PIN_A5
// Controller COR/RX/PTT/AUX pin assignments }}}
