// Default Functions table.
// This table defines the function codes
// and the corresponding Program pointer.
// RTQ = 1b ,RBH = 2b, RAO = 4b

#define SITE_ID	         8
#define DEFAULT_POLARITY 0 // ALL HIGH COR
#define DEFAULT_ENABLE   6 // ENABLE RAO & RBH
#define DEFAULT_PWM_PTT  2 // Morse on RBH only
#define DEFAULT_RX0_PTT  2 // RX RTQ = TX RBH Only
#define DEFAULT_RX1_PTT  7 // RX RBH = TX RBH & RAO
#define DEFAULT_RX2_PTT  2 // RX RTQ = TX RBH Only
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
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0} ,
	{ 0 , 0 , 0 , 0 , 0}
};
#define ProgramDefaultsNum 8
// Morse is causing problems because the morse characters span over multiple nibbles!!!
const int8 ProgramDefaults[ProgramDefaultsNum][8] = {
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 4
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 4
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 4
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 4
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 4
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 5
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0},  // Program 6
 {SEND_DTMF+1  , DONE+1  , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0 , DONE+0}  // Program 7
};
