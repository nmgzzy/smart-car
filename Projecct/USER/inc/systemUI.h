#ifndef __SYSTEM_UI_H_
#define __SYSTEM_UI_H_

#include "headfile.h"
#define PAGE_MAX 18
#define WORDS_MAX 15

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

typedef struct {
	uint8 name[WORDS_MAX];
    double data;
} Line_t;

typedef struct {
	Line_t line[8];
} Page_t;

uint8 readKey(void);
void displayUI(void);
void displayDebug(void);
void printLog(int8 message[20]);

extern uint16 distance;
extern float obt;
extern uint8 obstacle_pix2, obstacle_pix3, obstacle_detection_cnt ;

#endif