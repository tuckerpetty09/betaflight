#pragma once

typedef struct {
    float angle; // degrees
    float rate;  // degrees/second
} lqrAxisState_t;

typedef struct {
    float angleGain;
    float rateGain;
} lqrAxisGains_t;

// Returns a controller output in the PID sum's existing axis units.
float lqrCalculateAxis(
    lqrAxisState_t state,
    float targetAngle, // degrees
    lqrAxisGains_t gains
);
