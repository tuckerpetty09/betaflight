#pragma once

typedef struct {
    float angle;
    float rate;
} lqrAxisState_t;

typedef struct {
    float angleGain;
    float rateGain;
} lqrAxisGains_t;

float lqrCalculateAxis(
    lqrAxisState_t state,
    float targetAngle,
    lqrAxisGains_t gains
);
