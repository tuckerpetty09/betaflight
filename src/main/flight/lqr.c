#include "flight/lqr.h"

float lqrCalculateAxis(
    lqrAxisState_t state,
    float targetAngle,
    lqrAxisGains_t gains
)
{
    const float angleError = state.angle - targetAngle;

    return -(gains.angleGain * angleError
           + gains.rateGain * state.rate);
}
