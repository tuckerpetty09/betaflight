#include "flight/lqr.h"

#include <math.h>
#include <stddef.h>

bool lqrCalculateAxis(
    lqrAxisState_t state,
    float targetAngle,
    lqrAxisGains_t gains,
    float *output
)
{
    if (output == NULL
        || !isfinite(state.angle)
        || !isfinite(state.rate)
        || !isfinite(targetAngle)
        || !isfinite(gains.angleGain)
        || !isfinite(gains.rateGain)) {
        return false;
    }

    const float angleError = state.angle - targetAngle;
    const float calculatedOutput =
        -(gains.angleGain * angleError + gains.rateGain * state.rate);

    if (!isfinite(calculatedOutput)) {
        return false;
    }

    *output = calculatedOutput;
    return true;
}

void lqrForceFallback(lqrSwitchState_t *state)
{
    state->selected = false;
    state->lockedOut = true;
}

bool lqrUpdateSelection(
    lqrSwitchState_t *state,
    bool armed,
    bool requested,
    bool modeAllowed
)
{
    if (!armed) {
        state->lockedOut = false;
        state->selected = requested;
        return false;
    }

    if (state->selected && (!requested || !modeAllowed)) {
        lqrForceFallback(state);
    }

    return state->selected && !state->lockedOut;
}
