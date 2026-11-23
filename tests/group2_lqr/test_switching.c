#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

// Standalone model of our Betaflight switching logic.
static bool group2LqrSelected = false;
static bool group2LqrLockedOut = false;

static bool updateController(
    bool armed,
    bool user1,
    bool angleMode,
    bool horizonMode,
    bool gpsRescue,
    bool crashRecovery,
    bool launchControl)
{
    if (!armed) {
        group2LqrLockedOut = false;
        group2LqrSelected = user1;
    } else if (group2LqrSelected && !user1) {
        group2LqrSelected = false;
        group2LqrLockedOut = true;
    }

    bool lqrActive = group2LqrSelected && !group2LqrLockedOut;

    if (armed && lqrActive &&
        (!angleMode || horizonMode || gpsRescue ||
         crashRecovery || launchControl)) {
        group2LqrSelected = false;
        group2LqrLockedOut = true;
        lqrActive = false;
    }

    return lqrActive && angleMode && !horizonMode && !gpsRescue
           && !crashRecovery && !launchControl;
}

int main(void)
{
    // Select LQR before arming.
    assert(updateController(false, true, true, false, false, false, false));

    // Arm with LQR selected.
    assert(updateController(true, true, true, false, false, false, false));

    // Emergency: turn USER1 off.
    assert(!updateController(true, false, true, false, false, false, false));

    // Turning USER1 back on must NOT reactivate LQR.
    assert(!updateController(true, true, true, false, false, false, false));

    // Disarm and select LQR again.
    assert(updateController(false, true, true, false, false, false, false));
    assert(updateController(true, true, true, false, false, false, false));

    // Leaving Angle Mode must lock out LQR.
    assert(!updateController(true, true, false, false, false, false, false));

    // Returning to Angle Mode must NOT reactivate LQR.
    assert(!updateController(true, true, true, false, false, false, false));

    // Disarm and rearm to restore LQR.
    assert(updateController(false, true, true, false, false, false, false));
    assert(updateController(true, true, true, false, false, false, false));

    // Crash recovery must trigger PID fallback.
    assert(!updateController(true, true, true, false, false, true, false));
    assert(!updateController(true, true, true, false, false, false, false));

        // Horizon Mode must trigger PID fallback.
    assert(updateController(false, true, true, false, false, false, false));
    assert(updateController(true, true, true, false, false, false, false));
    assert(!updateController(true, true, true, true, false, false, false));
    assert(!updateController(true, true, true, false, false, false, false));

    // GPS Rescue must trigger PID fallback.
    assert(updateController(false, true, true, false, false, false, false));
    assert(updateController(true, true, true, false, false, false, false));
    assert(!updateController(true, true, true, false, true, false, false));
    assert(!updateController(true, true, true, false, false, false, false));

    // Launch control must trigger PID fallback.
    assert(updateController(false, true, true, false, false, false, false));
    assert(updateController(true, true, true, false, false, false, false));
    assert(!updateController(true, true, true, false, false, false, true));
    assert(!updateController(true, true, true, false, false, false, false));

    printf("PASS: All LQR/PID switching tests passed!\n");
    return 0;
}