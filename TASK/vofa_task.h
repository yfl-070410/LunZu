#ifndef VOFA_TASK_H
#define VOFA_TASK_H

#include "main.h"
#include "cmsis_os.h"

typedef struct {
    float buff[10];
    uint8_t tail[4];
} Vofa_JustFloat_TypeDef;

extern Vofa_JustFloat_TypeDef vofaPrintf;

extern void vofa_task(void const * pvParameters);

#endif
