#pragma once

#include <stdbool.h>

typedef struct {
    float angle; // radians
    float rate;  // radians/second
} lqrAxisState_t;

typedef struct {
    float angleGain;
    float rateGain;
} lqrAxisGains_t;

// Calculates a controller output in the PID sum's existing axis units.
// Gains and state use SI radians and radians/second.
bool lqrCalculateAxis(
    lqrAxisState_t state,
    float targetAngle, // radians
    lqrAxisGains_t gains,
    float *output
);
