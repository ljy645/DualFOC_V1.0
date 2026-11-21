/** @file cogging.c */
#include "cogging.h"
#include "motor_hw.h"
#include <math.h>

static int16_t tables[MOTOR_AXIS_COUNT][COGGING_POINTS];
static float gains[MOTOR_AXIS_COUNT];

int16_t *Cogging_Table(unsigned int axis) { return tables[axis]; }

void Cogging_SetGain(unsigned int axis, float gain)
{
    gains[axis] = gain;
}

float Cogging_Current(unsigned int axis, float mechanical_angle_rad)
{
    float position, fraction;
    unsigned int index, next;
    if (gains[axis] == 0.0f) { return 0.0f; }
    /* 调用方的机械角度有效且在 [0,2*pi)，末点到首点连续插值。 */
    position = mechanical_angle_rad * (COGGING_POINTS / 6.2831853071795865f);
    index = (unsigned int)position;
    fraction = position - index;
    index %= COGGING_POINTS;
    next = (index + 1U) % COGGING_POINTS;
    return (tables[axis][index] + fraction * (tables[axis][next] - tables[axis][index])) *
           COGGING_SCALE_A * gains[axis];
}
