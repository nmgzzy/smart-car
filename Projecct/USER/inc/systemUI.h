#ifndef __SYSTEM_UI_H_
#define __SYSTEM_UI_H_

#include "headfile.h"

typedef enum {
	MODE_DEBUG=1,
	MODE_STAND,
	MODE_START,
	MODE_STA_SPD,
    MODE_STA_DIR,
    MODE_PWM_TEST,
    MODE_START2,
    MODE_START3
} ModeSelect_t;

typedef enum {
	SPD_LOW=1,
	SPD_MID,
	SPD_HIGH
} SpdSelect_t;


uint8 readKey(void);
void displayUI(void);
void displayDebug(void);
void printLog(int8 message[20]);

#endif