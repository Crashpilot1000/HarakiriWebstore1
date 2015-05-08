#include "board.h"
#include "mw.h"

uint8_t useServo = 0;
int16_t motor[MAX_MOTORS];
int16_t servo[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
static motorMixerflt_t currentMixer[MAX_MOTORS];

// throttle * MixerMultiply, roll * MixerMultiply, pitch * MixerMultiply, yaw * MixerMultiply
// "float" limit -1.99f - +1.99f
static const motorMixer16_t mixerTri[] =
{
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.333333f * MixerMultiply,  0.0f * MixerMultiply },     // REAR
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -0.666667f * MixerMultiply,  0.0f * MixerMultiply },     // RIGHT
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -0.666667f * MixerMultiply,  0.0f * MixerMultiply },     // LEFT
};

static const motorMixer16_t mixerQuadP[] =
{
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply },          // RIGHT
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply },          // LEFT
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // FRONT
};

static const motorMixer16_t mixerQuadX[] =
{
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },          // FRONT_R
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_L
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // FRONT_L
};

static const motorMixer16_t mixerBi[] =
{
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply },          // LEFT
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply },          // RIGHT
};

static const motorMixer16_t mixerY6[] =
{
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.333333f * MixerMultiply,  1.0f * MixerMultiply },     // REAR
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -0.666667f * MixerMultiply, -1.0f * MixerMultiply },     // RIGHT
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -0.666667f * MixerMultiply, -1.0f * MixerMultiply },     // LEFT
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.333333f * MixerMultiply, -1.0f * MixerMultiply },     // UNDER_REAR
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -0.666667f * MixerMultiply,  1.0f * MixerMultiply },     // UNDER_RIGHT
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -0.666667f * MixerMultiply,  1.0f * MixerMultiply },     // UNDER_LEFT
};

static const motorMixer16_t mixerHex6P[] =
{
    { 1.0f * MixerMultiply, -0.866025f * MixerMultiply,  0.5f * MixerMultiply,  1.0f * MixerMultiply },     // REAR_R
    { 1.0f * MixerMultiply, -0.866025f * MixerMultiply, -0.5f * MixerMultiply, -1.0f * MixerMultiply },     // FRONT_R
    { 1.0f * MixerMultiply,  0.866025f * MixerMultiply,  0.5f * MixerMultiply,  1.0f * MixerMultiply },     // REAR_L
    { 1.0f * MixerMultiply,  0.866025f * MixerMultiply, -0.5f * MixerMultiply, -1.0f * MixerMultiply },     // FRONT_L
    { 1.0f * MixerMultiply,  0.0f      * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },     // FRONT
    { 1.0f * MixerMultiply,  0.0f      * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },     // REAR
};

static const motorMixer16_t mixerY4[] =
{
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_TOP CW
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply },          // FRONT_R CCW
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_BOTTOM CCW
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply },          // FRONT_L CW
};

static const motorMixer16_t mixerHex6X[] =
{
    { 1.0f * MixerMultiply, -0.5f * MixerMultiply,  0.866025f * MixerMultiply,  1.0f * MixerMultiply },     // REAR_R
    { 1.0f * MixerMultiply, -0.5f * MixerMultiply, -0.866025f * MixerMultiply,  1.0f * MixerMultiply },     // FRONT_R
    { 1.0f * MixerMultiply,  0.5f * MixerMultiply,  0.866025f * MixerMultiply, -1.0f * MixerMultiply },     // REAR_L
    { 1.0f * MixerMultiply,  0.5f * MixerMultiply, -0.866025f * MixerMultiply, -1.0f * MixerMultiply },     // FRONT_L
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f      * MixerMultiply, -1.0f * MixerMultiply },     // RIGHT
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  0.0f      * MixerMultiply,  1.0f * MixerMultiply },     // LEFT
};

static const motorMixer16_t mixerHex6H[] =
{
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },          // FRONT_R
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_L
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // FRONT_L
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply },          // RIGHT
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply,  0.0f * MixerMultiply },          // LEFT
};

static const motorMixer16_t mixerOctoX8[] =
{
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },          // FRONT_R
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_L
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // FRONT_L
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // UNDER_REAR_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // UNDER_FRONT_R
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // UNDER_REAR_L
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },          // UNDER_FRONT_L
};

static const motorMixer16_t mixerOctoFlatP[] =
{
    { 1.0f * MixerMultiply,  0.707107f * MixerMultiply, -0.707107f * MixerMultiply,  1.0f * MixerMultiply },// FRONT_L
    { 1.0f * MixerMultiply, -0.707107f * MixerMultiply, -0.707107f * MixerMultiply,  1.0f * MixerMultiply },// FRONT_R
    { 1.0f * MixerMultiply, -0.707107f * MixerMultiply,  0.707107f * MixerMultiply,  1.0f * MixerMultiply },// REAR_R
    { 1.0f * MixerMultiply,  0.707107f * MixerMultiply,  0.707107f * MixerMultiply,  1.0f * MixerMultiply },// REAR_L
    { 1.0f * MixerMultiply,  0.0f      * MixerMultiply, -1.0f      * MixerMultiply, -1.0f * MixerMultiply },// FRONT
    { 1.0f * MixerMultiply, -1.0f      * MixerMultiply,  0.0f      * MixerMultiply, -1.0f * MixerMultiply },// RIGHT
    { 1.0f * MixerMultiply,  0.0f      * MixerMultiply,  1.0f      * MixerMultiply, -1.0f * MixerMultiply },// REAR
    { 1.0f * MixerMultiply,  1.0f      * MixerMultiply,  0.0f      * MixerMultiply, -1.0f * MixerMultiply },// LEFT
};

static const motorMixer16_t mixerOctoFlatX[] =
{
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -0.5f * MixerMultiply,  1.0f * MixerMultiply },          // MIDFRONT_L
    { 1.0f * MixerMultiply, -0.5f * MixerMultiply, -1.0f * MixerMultiply,  1.0f * MixerMultiply },          // FRONT_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.5f * MixerMultiply,  1.0f * MixerMultiply },          // MIDREAR_R
    { 1.0f * MixerMultiply,  0.5f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_L
    { 1.0f * MixerMultiply,  0.5f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply },          // FRONT_L
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -0.5f * MixerMultiply, -1.0f * MixerMultiply },          // MIDFRONT_R
    { 1.0f * MixerMultiply, -0.5f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_R
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply,  0.5f * MixerMultiply, -1.0f * MixerMultiply },          // MIDREAR_L
};

static const motorMixer16_t mixerVtail4[] =
{
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply,  1.0f * MixerMultiply },          // REAR_R
    { 1.0f * MixerMultiply, -1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply },          // FRONT_R
    { 1.0f * MixerMultiply,  0.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply },          // REAR_L
    { 1.0f * MixerMultiply,  1.0f * MixerMultiply, -1.0f * MixerMultiply,  0.0f * MixerMultiply },          // FRONT_L
};

// Keep this synced with MultiType struct in mw.h!
  const mixer_t mixers[] = {       // Motors, UseServo, Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
    { 6, 0, mixerY6 },             // MULTITYPE_Y6
    { 6, 0, mixerHex6P },          // MULTITYPE_HEX6
    { 1, 1, NULL },                // * MULTITYPE_FLYING_WING
    { 4, 0, mixerY4 },             // MULTITYPE_Y4
    { 6, 0, mixerHex6X },          // MULTITYPE_HEX6X
    { 8, 0, mixerOctoX8 },         // MULTITYPE_OCTOX8
    { 8, 0, mixerOctoFlatP },      // MULTITYPE_OCTOFLATP
    { 8, 0, mixerOctoFlatX },      // MULTITYPE_OCTOFLATX
    { 4, 0, mixerVtail4 },         // MULTITYPE_VTAIL4
    { 6, 0, mixerHex6H },          // MULTITYPE_HEX6H
    { 0, 0, NULL },                // MULTITYPE_CUSTOM
};

void mixerInit(void)
{
    uint8_t i;
    int16_t yw = (int16_t)cfg.tri_ydir;
    NumberOfMotors = 0;
    useServo = mixers[cfg.mixerConfiguration].useServo;                           // enable servos for mixes that require them. note, this shifts motor counts.
    if (feature(FEATURE_SERVO_TILT)) useServo = 1;                                // if we want camstab/trig, that also enables servos, even if mixer doesn't

    if (cfg.mixerConfiguration == MULTITYPE_CUSTOM)
    {
        for (i = 0; i < MAX_MOTORS; i++)                                          // load custom mixer into currentMixer
        {
            if (!cfg.customMixer[i].throttle) break;                              // check if done
            currentMixer[i].throttle = Int16MixToFloat(cfg.customMixer[i].throttle);
            currentMixer[i].roll     = Int16MixToFloat(cfg.customMixer[i].roll);
            currentMixer[i].pitch    = Int16MixToFloat(cfg.customMixer[i].pitch);
            currentMixer[i].yaw      = Int16MixToFloat(cfg.customMixer[i].yaw * yw);// Do yaw direction here instead of every time in the mixer
            NumberOfMotors++;
        }
    }
    else
    {
        NumberOfMotors = mixers[cfg.mixerConfiguration].numberMotor;
        if (mixers[cfg.mixerConfiguration].motor)                                 // copy motor-based mixers
        {
            for (i = 0; i < NumberOfMotors; i++)
            {
                currentMixer[i].throttle = Int16MixToFloat(mixers[cfg.mixerConfiguration].motor[i].throttle);
                currentMixer[i].roll     = Int16MixToFloat(mixers[cfg.mixerConfiguration].motor[i].roll);
                currentMixer[i].pitch    = Int16MixToFloat(mixers[cfg.mixerConfiguration].motor[i].pitch);
                currentMixer[i].yaw      = Int16MixToFloat(mixers[cfg.mixerConfiguration].motor[i].yaw * yw);// Do yaw direction here instead of every time in the mixer
            }
        }
    }
}

float Int16MixToFloat(int16_t input)                                              // Get mixer from storage form
{
    return (float)input / MixerMultiply;
}

void mixerLoadMix(int index)
{
    int i;
    index++;                                                                      // we're 1-based
    for (i = 0; i < MAX_MOTORS; i++) cfg.customMixer[i].throttle = 0;             // clear existing
    if (mixers[index].motor != NULL)                                              // do we have anything here to begin with?
    {
        for (i = 0; i < mixers[index].numberMotor; i++) cfg.customMixer[i] = mixers[index].motor[i];
    }
}

void writeServos(void)
{
    static uint32_t yawarmdelaytimer = 0;
    if (!useServo) return;
    switch (cfg.mixerConfiguration)
    {
    case MULTITYPE_BI:
        pwmWriteServo(0, servo[4]);
        pwmWriteServo(1, servo[5]);
        break;

    case MULTITYPE_TRI:
        if (!cfg.tri_ydel)
        {
            pwmWriteServo(0, servo[5]);                                           // like always
        }
        else
        {
            if (f.ARMED)
            {
                if (!yawarmdelaytimer) yawarmdelaytimer = currentTimeMS + (uint32_t)cfg.tri_ydel;
                if (currentTimeMS >= yawarmdelaytimer) pwmWriteServo(0, servo[5]);// like always
                else pwmWriteServo(0, cfg.tri_ymid);                              // Give middlesignal to yaw servo when disarmed
            }
            else
            {
                yawarmdelaytimer = 0;
                pwmWriteServo(0, cfg.tri_ymid);                                   // Give middlesignal to yaw servo when disarmed
            }
        }
        break;

    case MULTITYPE_GIMBAL:
        pwmWriteServo(0, servo[0]);
        pwmWriteServo(1, servo[1]);
        break;

    default:                                                                      // Two servos for SERVO_TILT, if enabled
        if (feature(FEATURE_SERVO_TILT))
        {
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
        }
        break;
    }
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;
    for (i = 0; i < NumberOfMotors; i++)
    {
        motor[i] = mc;                                                            // Sends commands to all motors
        pwmWriteMotor(i, mc);
    }
}

void mixTableAndWriteMotors(void)
{
    static int32_t RangeShift = 0, UpperLimit, LowerLimit;
    int32_t Overshoot, MinMotor, MaxMotor, MwiiPitchPid = 0, MwiiYawPid = 0, newmot;
    float   Throttleflt;
    uint8_t i;

    if (!RangeShift)                                                              // Inirun
    {
        int32_t Range, Range85;
        Range      = cfg.esc_max - cfg.esc_min;
        Range85    = (Range * 109) >> 7;                                          // ca. 85% overshoot range for Maximal motor value. Note: This could be a constant as well
        UpperLimit = cfg.esc_max + Range85;
        LowerLimit = max(cfg.esc_min - (Range85 >> 1), 0);                        // Use Halfrange85limit for Minimal motor value influence;
        RangeShift = Range << MixerShift;
    }

    if ((cfg.mixerConfiguration == MULTITYPE_BI) || (cfg.mixerConfiguration == MULTITYPE_TRI))
    {
        MwiiPitchPid = axisPIDflt[PITCH];
        MwiiYawPid   = axisPIDflt[YAW];
        MwiiYawPid  *= (int32_t)cfg.tri_ydir;
    }

    switch (cfg.mixerConfiguration)                                               // airplane / servo mixes
    {
    case MULTITYPE_BI:
        servo[4] = constrain_int(1500 + MwiiYawPid + MwiiPitchPid, 1020, 2000);   // LEFT
        servo[5] = constrain_int(1500 + MwiiYawPid - MwiiPitchPid, 1020, 2000);   // RIGHT
        break;

    case MULTITYPE_TRI:
        servo[5] = constrain_int(cfg.tri_ymid + MwiiYawPid, cfg.tri_ymin, cfg.tri_ymax); // REAR
        break;

    case MULTITYPE_GIMBAL:
        servo[0] = constrain_int(cfg.gbl_pmd + (((int32_t)cfg.gbl_pgn * (int32_t)angle[PITCH]) >> 4) + rcCommand[PITCH], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain_int(cfg.gbl_rmd + (((int32_t)cfg.gbl_rgn * (int32_t)angle[ROLL])  >> 4) + rcCommand[ROLL] , cfg.gbl_rmn, cfg.gbl_rmx);
        break;

    default:
        break;
    }

    if (feature(FEATURE_SERVO_TILT))                                              // do camstab
    {
        int32_t aux[2] = {cfg.gbl_pmd, cfg.gbl_rmd};
        if ((cfg.gbl_flg & GIMBAL_NORMAL) || (cfg.gbl_flg & GIMBAL_TILTONLY)) aux[0] += rcData[AUX3] - cfg.rc_mid;
        if (!(cfg.gbl_flg & GIMBAL_DISABLEAUX34)) aux[1] += rcData[AUX4] - cfg.rc_mid;

        if (rcOptions[BOXCAMSTAB])
        {
            if (cfg.gbl_flg & GIMBAL_MIXTILT)
            {
                aux[0] -= (((int32_t)(-cfg.gbl_pgn) * (int32_t)angle[PITCH]) - ((int32_t)cfg.gbl_rgn * (int32_t)angle[ROLL])) >> 4;
                aux[1] += (((int32_t)(-cfg.gbl_pgn) * (int32_t)angle[PITCH]) + ((int32_t)cfg.gbl_rgn * (int32_t)angle[ROLL])) >> 4;
            }
            else
            {
                aux[0] += ((int32_t)cfg.gbl_pgn * (int32_t)angle[PITCH]) >> 4;
                aux[1] += ((int32_t)cfg.gbl_rgn * (int32_t)angle[ROLL])  >> 4;
            }
        }
        servo[0] = constrain_int(aux[0], cfg.gbl_pmn, cfg.gbl_pmx);
        servo[1] = constrain_int(aux[1], cfg.gbl_rmn, cfg.gbl_rmx);
    }

    if (cfg.gbl_flg & GIMBAL_FORWARDAUX)
    {
        uint8_t offset;
        if (feature(FEATURE_SERVO_TILT)) offset = 2;
        else offset = 0;
        for (i = 0; i < 4; i++) pwmWriteServo(i + offset, rcData[AUX1 + i]);
    }

    if (feature(FEATURE_LED) && (cfg.LED_Type == 1))
    {
        if (feature(FEATURE_SERVO_TILT)) pwmWriteServo(2, LED_Value);
        else pwmWriteServo(0, LED_Value);
    }

    if (!f.ARMED)                                                                 // If not armed
    {
        writeAllMotors(cfg.esc_moff);                                             // Stop all motors
        return;                                                                   // And stop the rest from happening
    }

    if (rcData[THROTTLE] < cfg.rc_minchk)
    {
        switch (cfg.rc_motor)                                                     // [0-2] Behaviour when thr < rc_minchk: 0= minthrottle no regulation, 1= minthrottle&regulation, 2= Motorstop 
        {
        case 0:
            writeAllMotors(cfg.esc_min);
            return;                                                               // end here
        case 1:
            break;                                                                // keep going
        case 2:
            writeAllMotors(cfg.esc_moff);
            return;                                                               // end here
        }
    }

    if (NumberOfMotors < 2)                                                       // single motor - only servo mixes
    {
        pwmWriteMotor(0, rcCommand[THROTTLE]);                                    // write the only motor that may be connected with the RC throttle value
        return;                                                                   // end here since no further motormixing is required.
    }

    MinMotor = cfg.esc_min, MaxMotor = cfg.esc_max, Throttleflt = rcCommand[THROTTLE];
    for (i = 0; i < NumberOfMotors; i++)
    {
        newmot = Throttleflt       * currentMixer[i].throttle +
                 axisPIDflt[PITCH] * currentMixer[i].pitch    +
                 axisPIDflt[ROLL]  * currentMixer[i].roll     +
                 axisPIDflt[YAW]   * currentMixer[i].yaw;
        motor[i] = constrain_int(newmot, -32767 ,32767);                          // Ensure int16_t range just in case...
        if (motor[i] < MinMotor) MinMotor = motor[i];
        motor[i] = constrain(motor[i], cfg.esc_min, UpperLimit);
        if (motor[i] > MaxMotor) MaxMotor = motor[i];
    }

    if (MaxMotor > cfg.esc_max)
    {
        if(cfg.esc_nwmx)                                                          // 0 = mwii style, 1 = scaled handling of maxthrottlesituations
        {
            Overshoot = RangeShift / (MaxMotor - max(MinMotor, LowerLimit));
            for (i = 0; i < NumberOfMotors; i++)
            {
                motor[i] = min(cfg.esc_min + (((int32_t)(motor[i] - cfg.esc_min) * Overshoot) >> MixerShift), cfg.esc_max);
                pwmWriteMotor(i, motor[i]);
            }
        }
        else
        {
            Overshoot = MaxMotor - cfg.esc_max;
            for (i = 0; i < NumberOfMotors; i++)
            {
                motor[i] = max(motor[i] - Overshoot, cfg.esc_min);
                pwmWriteMotor(i, motor[i]);
            }
        }
    }
    else
    {
        for (i = 0; i < NumberOfMotors; i++) pwmWriteMotor(i, motor[i]);
    }
}

/*
This is the not speedoptimized version

    if (MaxMotor > cfg.esc_max)
    {
        if(cfg.esc_nwmx) Overshoot = RangeShift / (MaxMotor - max(MinMotor, LowerLimit)); // 0 = mwii style, 1 = scaled handling of maxthrottlesituations
        else Overshoot = MaxMotor - cfg.esc_max;
    }
    else Overshoot = 0;
    for (i = 0; i < NumberOfMotors; i++)
    {
        if(Overshoot)
        {
            if(cfg.esc_nwmx) motor[i] = min(cfg.esc_min + (((int32_t)(motor[i] - cfg.esc_min) * Overshoot) >> MixerShift), cfg.esc_max);
            else motor[i] = max(motor[i] - Overshoot, cfg.esc_min);
        }
        pwmWriteMotor(i, motor[i]);
    }


*/
