/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/filter.h"
#include "io/rc_controls.h"

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter
#define IS_POSITIVE(x) ((x > 0) ? true : false)

typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    PID_CONTROLLER_MWREWRITE = 1,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pidControllerType_e;

#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == PID_CONTROLLER_LUX_FLOAT)

typedef struct pidProfile_s {
    uint8_t pidController;                  // 1 = rewrite from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671, 2 = Luggi09s new baseflight pid

    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];

    float P_f[XYZ_AXIS_COUNT];                           // float p i and d factors for lux float pid controller
    float I_f[XYZ_AXIS_COUNT];
    float D_f[XYZ_AXIS_COUNT];
    float A_level;
    float H_level;
    uint8_t H_sensitivity;

    uint8_t AcroPlusFactor;                 // Acro+ factor
    uint8_t gyro_lpf_hz;                    // Gyro Soft filter in hz
    uint8_t dterm_lpf_hz;                   // Delta Filter in hz
    uint8_t yaw_pterm_cut_hz;               // Used for filering Pterm noise on noisy frames

#ifdef GTUNE
    uint8_t  gtune_lolimP[XYZ_AXIS_COUNT];               // [0..200] Lower limit of P during G tune
    uint8_t  gtune_hilimP[XYZ_AXIS_COUNT];               // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
    uint8_t  gtune_pwr;                     // [0..10] Strength of adjustment
    uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
    uint8_t  gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
#endif
} pidProfile_t;

typedef struct airModePlus {
    float factor;
    float wowFactor;
    float iTermScaler;
} airModePlus_t;

extern int16_t axisPID[XYZ_AXIS_COUNT];
extern int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];

extern float factor0;
extern float factor1;
extern float wow_factor0;
extern float wow_factor1;
extern float Throttle_p;

extern float dT;
extern bool motorLimitReached;
extern bool allowITermShrinkOnly;

extern float wow_factor;
extern int16_t factor;

extern airModePlus_t airModePlusAxisState[XYZ_AXIS_COUNT];
extern bool deltaStateIsSet;
extern biquad_t deltaBiQuadState[XYZ_AXIS_COUNT];
extern uint8_t PIDweight[XYZ_AXIS_COUNT];
extern filterStatePt1_t yawPTermState;

extern int32_t errorGyroI[XYZ_AXIS_COUNT];
extern float errorGyroIf[XYZ_AXIS_COUNT];

void pidSetController(pidControllerType_e type);
void pidResetErrorGyro(void);
void airModePlus(airModePlus_t *axisState, int axis, controlRateConfig_t *controlRateConfig);

