#include <cmath>

#include "gtest/gtest.h"

extern "C" {
#include "flight/lqr.h"
}

TEST(Lqr, CalculatesAngleAndRateFeedbackInPidSumUnits)
{
    const lqrAxisState_t state = { 12.0f * static_cast<float>(M_PI) / 180.0f,
                                   30.0f * static_cast<float>(M_PI) / 180.0f };
    const lqrAxisGains_t gains = { 2.0f, 0.5f };
    float output = 0.0f;

    ASSERT_TRUE(lqrCalculateAxis(
        state,
        5.0f * static_cast<float>(M_PI) / 180.0f,
        gains,
        &output
    ));
    EXPECT_NEAR(-29.0f * static_cast<float>(M_PI) / 180.0f, output, 1e-6f);
}

TEST(Lqr, ZeroErrorProducesZeroOutput)
{
    const lqrAxisState_t state = { -8.0f * static_cast<float>(M_PI) / 180.0f, 0.0f };
    const lqrAxisGains_t gains = { 3.0f, 1.0f };
    float output = 1.0f;

    ASSERT_TRUE(lqrCalculateAxis(
        state,
        -8.0f * static_cast<float>(M_PI) / 180.0f,
        gains,
        &output
    ));
    EXPECT_FLOAT_EQ(0.0f, output);
}

TEST(Lqr, RejectsInvalidInputsAndOutputs)
{
    const lqrAxisState_t state = { 0.0f, 0.0f };
    const lqrAxisGains_t gains = { 1.0f, 1.0f };
    float output = 0.0f;

    EXPECT_FALSE(lqrCalculateAxis(state, NAN, gains, &output));
    EXPECT_FALSE(lqrCalculateAxis(state, 0.0f, gains, NULL));

    const lqrAxisGains_t overflowingGains = { HUGE_VALF, 1.0f };
    output = 7.0f;
    EXPECT_FALSE(lqrCalculateAxis(state, 1.0f, overflowingGains, &output));
    EXPECT_FLOAT_EQ(7.0f, output);
}

TEST(LqrSwitch, SelectsLqrBeforeArming)
{
    lqrSwitchState_t state = {};

    EXPECT_FALSE(lqrUpdateSelection(&state, false, true, true));
    EXPECT_TRUE(lqrUpdateSelection(&state, true, true, true));
}

TEST(LqrSwitch, CannotEnableLqrMidFlight)
{
    lqrSwitchState_t state = {};

    EXPECT_FALSE(lqrUpdateSelection(&state, false, false, true));
    EXPECT_FALSE(lqrUpdateSelection(&state, true, true, true));
}

TEST(LqrSwitch, EmergencySwitchLocksOutLqr)
{
    lqrSwitchState_t state = {};

    lqrUpdateSelection(&state, false, true, true);
    EXPECT_TRUE(lqrUpdateSelection(&state, true, true, true));

    EXPECT_FALSE(lqrUpdateSelection(&state, true, false, true));
    EXPECT_TRUE(state.lockedOut);
    EXPECT_FALSE(lqrUpdateSelection(&state, true, true, true));
}

TEST(LqrSwitch, IncompatibleModeTriggersFallback)
{
    lqrSwitchState_t state = {};

    lqrUpdateSelection(&state, false, true, true);
    EXPECT_TRUE(lqrUpdateSelection(&state, true, true, true));

    EXPECT_FALSE(lqrUpdateSelection(&state, true, true, false));
    EXPECT_TRUE(state.lockedOut);
    EXPECT_FALSE(lqrUpdateSelection(&state, true, true, true));
}

TEST(LqrSwitch, ForcedFallbackLocksOutLqr)
{
    lqrSwitchState_t state = {};

    lqrUpdateSelection(&state, false, true, true);
    EXPECT_TRUE(lqrUpdateSelection(&state, true, true, true));

    lqrForceFallback(&state);

    EXPECT_TRUE(state.lockedOut);
    EXPECT_FALSE(lqrUpdateSelection(&state, true, true, true));
}

TEST(LqrSwitch, DisarmingResetsLockout)
{
    lqrSwitchState_t state = {};

    lqrUpdateSelection(&state, false, true, true);
    lqrUpdateSelection(&state, true, true, true);
    lqrForceFallback(&state);

    EXPECT_FALSE(lqrUpdateSelection(&state, false, true, true));
    EXPECT_FALSE(state.lockedOut);
    EXPECT_TRUE(lqrUpdateSelection(&state, true, true, true));
}
