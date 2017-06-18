// Default Functions table.
// This table defines the function codes
// and the corresponding Program pointer.
// Example:
// 124* --> Program0
// If 124* is entered, Program 0 is executed.
// RAO = 4
// RBH = 2
// RTQ = 1
// RX0 = RTQ
// RX1 = RBH
// RX2 = RAO


#define SITE_ID	         8
#define DEFAULT_POLARITY 0 // ALL HIGH COR
#define DEFAULT_ENABLE   6 // ENABLE RAO & RBH
#define DEFAULT_PWM_PTT  2 // Morse on RBH only
#define DEFAULT_RX0_PTT  6 // RX RTQ = Nothing by default
#define DEFAULT_RX1_PTT  7 // RX RBH = TX RBH & RAO
#define DEFAULT_RX2_PTT  3 // RX RAO = TX RBH Only
#define MORSE_ID0        MCHAR('v')
#define MORSE_ID1        MCHAR('e')
#define MORSE_ID2        2
#define MORSE_ID3        MCHAR('r')
#define MORSE_ID4        MCHAR('b')
#define MORSE_ID5        MCHAR('h')
#define DEFAULT_PTT0TO	 5
#define DEFAULT_PTT1TO	 5
#define DEFAULT_PTT2TO	 5

const int8 FunctionDefaults[8][5] = {
	{d1+Strb , d5+Strb, d7+Strb , ds+Strb+LastDigit , 0} , // 157* --> Instr 0 (Ouverture)
	{d1+Strb , d5+Strb, d7+Strb , dp+Strb+LastDigit , 2} , // 157# --> Instr 1 (Fermeture)
	{d1+Strb , d0+Strb, d7+Strb , ds+Strb+LastDigit , 0} , // 107* --> Instr 0 (Ouverture)
	{d1+Strb , d0+Strb, d7+Strb , dp+Strb+LastDigit , 2} , // 107# --> Instr 1 (Fermeture)
	{d2+Strb , d3+Strb, d7+Strb , ds+Strb+LastDigit , 4} , // 237* --> Shutdown RAO,
	{d2+Strb , d4+Strb, d7+Strb , ds+Strb+LastDigit , 5} , // 247* --> Shutdown RBH only,
	{d2+Strb , d2+Strb, d7+Strb , ds+Strb+LastDigit , 6} , // 227* --> Shutdown Ripon,
	{ 0 , 0 , 0 , 0 , 0}
};

#define ProgramDefaultsNum 8
// Morse is causing problems because the morse characters span over multiple nibbles!!!
const int8 ProgramDefaults[ProgramDefaultsNum][8] = {
 {UPDATE_PWM_PTT+4, SEND_DTMF+dd,SEND_DTMF+0x01, SEND_DTMF+0x02 , UPDATE_ENABLE + 7, UPDATE_PWM_PTT + 7 ,PAUSE+4, JUMP+1},  // Program 0 --> DTMF(D12)
 {MORSE+MCHAR('v'), MORSE+MCHAR('e'), MORSE+0x02 , MORSE+MCHAR('r'),MORSE+MCHAR('b'),MORSE+MCHAR('h'), DONE+1 , DONE+0}, // Program 1
 {UPDATE_PWM_PTT+7, MORSE+MCHAR('v'), MORSE+MCHAR('e'), MORSE+0x02 , MORSE+MCHAR('r'),MORSE+MCHAR('b'),MORSE+MCHAR('h'), JUMP+3 },  // Program 2
 {UPDATE_PWM_PTT+4, PAUSE+4, SEND_DTMF+dd,SEND_DTMF+0x01, SEND_DTMF+0x00 , UPDATE_ENABLE + 0x06 , DONE+1 , DONE+0 },  // Program 3 --> DTMF(D10)
 {MORSE+MCHAR('v'), MORSE+MCHAR('e'), MORSE+0x02 , MORSE+MCHAR('r'),MORSE+MCHAR('b'),MORSE+MCHAR('h'), UPDATE_ENABLE + 2, DONE+0},  // Program 4 Shutdown RAO
 {MORSE+MCHAR('v'), MORSE+MCHAR('e'), MORSE+0x02 , MORSE+MCHAR('r'),MORSE+MCHAR('b'),MORSE+MCHAR('h'), UPDATE_ENABLE + 5, DONE+0},  // Program 5 Shutdown RBH Only
 {MORSE+MCHAR('v'), MORSE+MCHAR('e'), MORSE+0x02 , MORSE+MCHAR('r'),MORSE+MCHAR('b'),MORSE+MCHAR('h'), UPDATE_ENABLE + 0, DONE+0},  // Program 6 Shutdown Ripon
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0}  // Program 7
};
