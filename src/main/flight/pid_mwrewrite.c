#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/gyro_sync.h"

#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "config/runtime_config.h"

void pidMultiWiiRewrite(pidProfile_t *pidProfile,
    controlRateConfig_t *controlRateConfig,
    uint16_t max_angle_inclination,
    rollAndPitchTrims_t *angleTrim,
    rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);

    int axis;
    int32_t PTerm, ITerm, DTerm, delta;
    static int32_t lastError[3] = { 0, 0, 0 };
    static int32_t previousErrorGyroI[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError, gyroRate;

    int8_t horizonLevelStrength = 100;

    if (!deltaStateIsSet && pidProfile->dterm_lpf_hz) {
        for (axis = 0; axis < 3; axis++) BiQuadNewLpf(pidProfile->dterm_lpf_hz, &deltaBiQuadState[axis], 0);
        deltaStateIsSet = true;
    }

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection
        // Using Level D as a Sensitivity for Horizon. 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
        horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * pidProfile->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        uint8_t rate = controlRateConfig->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = ((int32_t)(rate + 27) * rcCommand[YAW]) >> 5;
        } else {
            if (IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
                wow_factor = fabsf(rcCommand[axis] / 500.0f) * ((float)controlRateConfig->AcroPlusFactor / 100.0f); //0-1f
                factor = (int16_t)(wow_factor * (float)rcCommand[axis]) + rcCommand[axis];
            } else {
                factor = rcCommand[axis]; // 200dps to 1200dps max roll/pitch rate
            }
            AngleRateTmp = ((int32_t)(rate + 27) * factor) >> 4;

            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to max configured inclination
#ifdef GPS
                const int32_t errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis],
                    -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#else
                const int32_t errorAngle = constrain(2 * rcCommand[axis],
                    -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode - control is angle based, so control loop is needed
                    AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
                } else {
                    // HORIZON mode - mix up angle error to desired AngleRateTmp to add a little auto-level feel,
                    // horizonLevelStrength is scaled to the stick input
                    AngleRateTmp += (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        gyroRate = gyroADC[axis] / 4;
        RateError = AngleRateTmp - gyroRate;

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;

        if (axis == YAW && pidProfile->yaw_pterm_cut_hz) {
            PTerm = filterApplyPt1(PTerm, &yawPTermState, pidProfile->yaw_pterm_cut_hz, dT);
        }

                // -----calculate I component
                // there should be no division before accumulating the error to integrator, because the precision would be reduced.
                // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
                // Time correction (to avoid different I scaling for different builds based on average cycle time)
                // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (uint16_t)targetESCwritetime) >> 11) * pidProfile->I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);

        ITerm = errorGyroI[axis] >> 13;

        if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
            airModePlus(&airModePlusAxisState[axis], axis, controlRateConfig);
            errorGyroI[axis] *= airModePlusAxisState[axis].iTermScaler;
        }

        if ((IS_RC_MODE_ACTIVE(BOXAIRMODE)) && (allowITermShrinkOnly || motorLimitReached)) {
            if (ABS(errorGyroI[axis]) < ABS(previousErrorGyroI[axis])) {
                previousErrorGyroI[axis] = errorGyroI[axis];
            } else {
                errorGyroI[axis] = constrain(errorGyroI[axis], -ABS(previousErrorGyroI[axis]), ABS(previousErrorGyroI[axis]));
            }
        } else {
            previousErrorGyroI[axis] = errorGyroI[axis];
        }

        //-----calculate D-term
        delta = RateError - lastError[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        if (deltaStateIsSet) {
            delta = lrintf(applyBiQuadFilter((float) delta, &deltaBiQuadState[axis]));
        }

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t) 0xFFFF / ((uint16_t)targetESCwritetime >> 4))) >> 6;

        DTerm = (delta * 3 * (pidProfile->D8[axis] / 4) * PIDweight[axis] / 100) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;

        if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
            axisPID[axis] = lrintf(airModePlusAxisState[axis].factor + airModePlusAxisState[axis].wowFactor * axisPID[axis]);
        }

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

#ifdef BLACKBOX
        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
#endif
    }
}
