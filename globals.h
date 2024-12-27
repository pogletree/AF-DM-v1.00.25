#ifndef GLOBALS_H
#define GLOBALS_H
#include "Cm110s.h"     // Not referenced but is needed.

#define MODEM_SAMPLE_BLOCK_SIZE     9600  //From soundcard
#define TX_MODEM_SAMPLE_BLOCK_SIZE  9600  //From soundcard

#define VERSION "Release"       //Set to "Alpha", "Beta", or "Release"
#define ALPHA_TIME 30           //Short alpha duration
#define BETA_TIME 180           //Long beta duration


#define APPVERSION "1.00"       //Changes version number in mainwindow title and version message.
#define MINOR ".25"
#define TITLE "AF-DM_v1.00.25"
#define QTVERSION "6.8.0"
typedef enum{CODEC_OPEN,CODEC_CLOSED}CodecState;

#endif // GLOBALS_H
