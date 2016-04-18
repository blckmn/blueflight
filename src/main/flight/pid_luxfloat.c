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

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "config/runtime_config.h"

void pidLuxFloat(pidProfile_t *pidProfile,
	controlRateConfig_t *controlRateConfig,
	uint16_t max_angle_inclination,
	rollAndPitchTrims_t *angleTrim,
	rxConfig_t *rxConfig)
{
	float RateError, AngleRate, gyroRate;
	float ITerm, PTerm, DTerm;
	static float lastError[3];
	float delta;
	int axis;
	float horizonLevelStrength = 1;
	static float previousErrorGyroIf[3] = { 0.0f, 0.0f, 0.0f };

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
		horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
		if (pidProfile->H_sensitivity == 0) {
			horizonLevelStrength = 0;
		} else {
			horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->H_sensitivity)) + 1, 0, 1);
		}
	}

	    // ----------PID controller----------
	for (axis = 0; axis < 3; axis++) {
	    // -----Get the desired angle rate depending on flight mode
		uint8_t rate = controlRateConfig->rates[axis];

		if (axis == FD_YAW) {
		    // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
			AngleRate = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
		} else {
		    // ACRO mode, control is GYRO based, direct sticks control is applied to rate PID
			if (IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
				wow_factor = fabsf(rcCommand[axis] / 500.0f) * ((float)controlRateConfig->AcroPlusFactor / 100.0f); //0-1f
				factor = (int16_t)(wow_factor * (float)rcCommand[axis]) + rcCommand[axis];
			} else {
				factor = rcCommand[axis]; // 200dps to 1200dps max roll/pitch rate
			}
			AngleRate = (float)((rate + 20) * factor) / 50.0f; // 200dps to 1200dps max roll/pitch rate

			        	 //25 wf + 650

			if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
			   // calculate error angle and limit the angle to the max inclination
#ifdef GPS
				const float errorAngle = (constrain(rcCommand[axis] + GPS_angle[axis],
					-((int) max_angle_inclination),
					+max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#else
				const float errorAngle = (constrain(rcCommand[axis],
					-((int) max_angle_inclination),
					+max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#endif
				if (FLIGHT_MODE(ANGLE_MODE)) {
				    // ANGLE mode - control is angle based, so control loop is needed
					AngleRate = errorAngle * pidProfile->A_level;
				} else {
				    // HORIZON mode - direct sticks control is applied to rate PID
				    // mix up angle error to desired AngleRate to add a little auto-level feel
					AngleRate += errorAngle * pidProfile->H_level * horizonLevelStrength;
				}
			}
		}

		gyroRate = gyroADC[axis] * gyro.scale; // gyro output scaled to dps

		        // --------low-level gyro-based PID. ----------
		        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
		        // -----calculate scaled error.AngleRates
		        // multiplication of rcCommand corresponds to changing the sticks scaling here
		RateError = AngleRate - gyroRate;

		        // -----calculate P component
		PTerm = RateError * (pidProfile->P_f[axis] / 4) * PIDweight[axis] / 100;

		if (axis == YAW && pidProfile->yaw_pterm_cut_hz) {
			PTerm = filterApplyPt1(PTerm, &yawPTermState, pidProfile->yaw_pterm_cut_hz, dT);
		}

		        // -----calculate I component.
		errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (pidProfile->I_f[axis] / 2)  * 10, -250.0f, 250.0f);


		if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
			airModePlus(&airModePlusAxisState[axis], axis, controlRateConfig);
			errorGyroIf[axis] *= airModePlusAxisState[axis].iTermScaler;
		}

		if ((IS_RC_MODE_ACTIVE(BOXAIRMODE)) && (allowITermShrinkOnly || motorLimitReached)) { //only in airmode do we affect Ki.
			if (ABS(errorGyroIf[axis]) < ABS(previousErrorGyroIf[axis])) {
				previousErrorGyroIf[axis] = errorGyroIf[axis];
			} else {
				errorGyroIf[axis] = constrain(errorGyroIf[axis], -ABS(previousErrorGyroIf[axis]), ABS(previousErrorGyroIf[axis]));
			}
		} else {
			previousErrorGyroIf[axis] = errorGyroIf[axis];
		}

		        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
		        // I coefficient (I8) moved before integration to make limiting independent from PID settings

		ITerm = errorGyroIf[axis];

		        //-----calculate D-term
		delta = RateError - lastError[axis];
		lastError[axis] = RateError;

		if (deltaStateIsSet) {
			delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
		}

		        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
		        // would be scaled by different dt each time. Division by dT fixes that.
		delta *= (1.0f / dT);


		DTerm = constrainf(delta * (pidProfile->D_f[axis] / 10) * PIDweight[axis] / 100, -300.0f, 300.0f);


		        // -----calculate total PID output
		axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);

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
