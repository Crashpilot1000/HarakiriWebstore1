#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

typedef enum
{
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct                              // Different order/packing saves 1432 Bytes here
{
    const uint8_t type;
    const uint8_t lcd;                      // 1 = Displayed in LCD // 0 = Not displayed
    const int16_t min;
    const int32_t max;
    void *ptr;
    const char *name;
} clivalue_t;

// we unset this on 'exit'
static void cliAuxset(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefault(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliScanbus(char *cmdline);
static void cliPassgps(char *cmdline);
static void cliFlash(char *cmdline);
static void cliErrorMessage(void);
static void cliRecal(void);
static void cliPrintVar(const clivalue_t *var, uint32_t full);
static void cliWpflush(char *cmdline);

// serial LCD
static void LCDinit(void);
static void LCDclear(void);
static void LCDline2(void);
static void LCDchangeval(const clivalue_t *var, const int8_t adder);

// OLED-Display
extern bool i2cLCD;                         // true, if an OLED-Display is connected

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;
extern float   gyroZero[3];                 // Populated upon initialization
extern bool    havel3g4200d;


// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// gpspass
static void GPSbyteRec(uint16_t c);
static uint8_t NewGPSByte;
static bool HaveNewGpsByte;

// buffer
static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] =
{
    "TRI",
    "QUADP",
    "QUADX",
    "BI",
    "GIMBAL",
    "Y6",
    "HEX6",
    "FLYING_WING",
    "Y4",
    "HEX6X",
    "OCTOX8",
    "OCTOFLATP",
    "OCTOFLATX",
    "VTAIL4",
    "HEX6H",
    "CUSTOM",
  NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] =
{
    "PPM",
    "VBAT",
    "SPEKTRUM",
    "GRAUPNERSUMH",
    "SERVO_TILT",
    "LED",
    "GPS",
    "FAILSAFE",
    "SONAR",
    "PASS",
    "LCD",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] =
{
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

const char * const accNames[] =
{
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct
{
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] =
{
    { "auxset",  "alternative to GUI",                cliAuxset   },
    { "cmix",    "design custom mixer",               cliCMix     },
    { "default", "load defaults & reboot",            cliDefault  },
    { "dump",    "dump config",                       cliDump     },
    { "exit",    "exit & reboot",                     cliExit     },
    { "feature", "list or -val or val",               cliFeature  },
    { "flash",   "flashmode",                         cliFlash    },
    { "help",    "this text",                         cliHelp     },
    { "map",     "mapping of rc channel order",       cliMap      },
    { "mixer",   "mixer name or list",                cliMixer    },
    { "passgps", "pass through gps data",             cliPassgps  },
    { "save",    "save and reboot",                   cliSave     },
    { "scanbus", "scan i2c bus",                      cliScanbus  },
    { "set",     "name=value or blank or * for list", cliSet      },
    { "status",  "sys status & stats",                cliStatus   },
    { "version", "",                                  cliVersion  },
    { "wpflush", "clear wp list",                     cliWpflush  },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))
  
const clivalue_t valueTable[] =
{
//    type, Lcd, int16_t min,int32_t max, *ptr,                        *name
    { VAR_UINT8,  1,       0,         32, &cfg.rc_db,                  "rc_db"                  },
    { VAR_UINT8,  1,       0,         90, &cfg.rc_dbyw,                "rc_dbyw"                },
    { VAR_UINT8,  1,       1,        100, &cfg.rc_dbah,                "rc_dbah"                },
    { VAR_UINT8,  1,       0,        100, &cfg.rc_dbgps,               "rc_dbgps"               },
    { VAR_FLOAT,  1,    -300,        300, &cfg.angleTrim[ROLL],        "rc_trm_rll"             },
    { VAR_FLOAT,  1,    -300,        300, &cfg.angleTrim[PITCH],       "rc_trm_ptch"            },
    { VAR_UINT16, 0,    1001,       1250, &cfg.rc_minchk,              "rc_minchk"              },
    { VAR_UINT16, 0,    1400,       1600, &cfg.rc_mid,                 "rc_mid"                 },
    { VAR_UINT16, 0,    1750,       1999, &cfg.rc_maxchk,              "rc_maxchk"              },
    { VAR_UINT8,  0,       0,          1, &cfg.rc_lowlat,              "rc_lowlat"              },
    { VAR_UINT8,  0,       0,          1, &cfg.rc_rllrm,               "rc_rllrm"               },
    { VAR_UINT16, 1,       0,      10000, &cfg.rc_killt,               "rc_killt"               },
    { VAR_UINT8,  1,       0,          3, &cfg.rc_flpsp,               "rc_flpsp"               },
    { VAR_UINT8,  1,       0,          2, &cfg.rc_motor,               "rc_motor"               },
    { VAR_UINT8,  0,       0,          6, &cfg.rc_auxch,               "rc_auxch"               },
    { VAR_UINT8,  1,       0,        250, &cfg.rcRate8,                "rc_rate"                },
    { VAR_UINT8,  1,       0,        100, &cfg.rcExpo8,                "rc_expo"                },
    { VAR_UINT8,  1,       0,        100, &cfg.thrMid8,                "thr_mid"                },
    { VAR_UINT8,  1,       0,        250, &cfg.thrExpo8,               "thr_expo"               },
    { VAR_UINT8,  1,       0,        100, &cfg.rollPitchRate,          "roll_pitch_rate"        },
    { VAR_UINT8,  1,       0,        100, &cfg.yawRate,                "yawrate"                },
    { VAR_UINT8,  1,       0,          1, &cfg.rc_oldyw,               "rc_oldyw"               },
    { VAR_UINT8,  0,       0,          1, &cfg.devorssi,               "devorssi"               },
    { VAR_UINT8,  0,       0,         80, &cfg.rssicut,                "rssicut"                },
    { VAR_UINT8,  1,      10,        200, &cfg.gt_lolimP[ROLL],        "gt_loP_rll"             },
    { VAR_UINT8,  1,      10,        200, &cfg.gt_lolimP[PITCH],       "gt_loP_ptch"            },
    { VAR_UINT8,  1,      10,        200, &cfg.gt_lolimP[YAW],         "gt_loP_yw"              },
    { VAR_UINT8,  1,       0,        200, &cfg.gt_hilimP[ROLL],        "gt_hiP_rll"             },
    { VAR_UINT8,  1,       0,        200, &cfg.gt_hilimP[PITCH],       "gt_hiP_ptch"            },
    { VAR_UINT8,  1,       0,        200, &cfg.gt_hilimP[YAW],         "gt_hiP_yw"              },
    { VAR_INT8,   1,       0,         10, &cfg.gt_pwr,                 "gt_pwr"                 },
    { VAR_INT16,  0,       0,       2000, &cfg.esc_min,                "esc_min"                },
    { VAR_INT16,  0,       0,       2000, &cfg.esc_max,                "esc_max"                },
    { VAR_UINT16, 1,       0,       2000, &cfg.esc_nfly,               "esc_nfly"               },
    { VAR_UINT8,  1,       0,          1, &cfg.esc_nwmx,               "esc_nwmx"               },
    { VAR_UINT16, 0,       0,       2000, &cfg.esc_moff,               "esc_moff"               },
    { VAR_UINT16, 0,      50,        498, &cfg.esc_pwm,                "esc_pwm"                },
    { VAR_UINT16, 0,      50,        498, &cfg.srv_pwm,                "srv_pwm"                },
    { VAR_UINT8,  0,       0,         10, &cfg.pass_mot,               "pass_mot"               },
    { VAR_UINT8,  1,       0,         40, &cfg.fs_delay,               "fs_delay"               },
    { VAR_UINT8,  1,       0,        200, &cfg.fs_ofdel,               "fs_ofdel"               },
    { VAR_UINT16, 1,    1000,       2000, &cfg.fs_rcthr,               "fs_rcthr"               },
    { VAR_UINT8,  1,       0,        250, &cfg.fs_ddplt,               "fs_ddplt"               },
    { VAR_UINT8,  1,       0,          1, &cfg.fs_jstph,               "fs_jstph"               },
    { VAR_UINT8,  1,       0,          1, &cfg.fs_nosnr,               "fs_nosnr"               },
    { VAR_UINT32, 0,    1200,     115200, &cfg.serial_baudrate,        "serial_baudrate"        },
    { VAR_UINT8,  1,       0,          3, &cfg.tele_prot,              "tele_prot"              },
    { VAR_UINT8,  0,       0,          1, &cfg.spektrum_hires,         "spektrum_hires"         },
    { VAR_UINT8,  0,      10,        200, &cfg.vbatscale,              "vbatscale"              },
    { VAR_UINT8,  0,      10,         50, &cfg.vbatmaxcellvoltage,     "vbatmaxcellvoltage"     },
    { VAR_UINT8,  0,      10,         50, &cfg.vbatmincellvoltage,     "vbatmincellvoltage"     },
    { VAR_UINT8,  0,       0,          9, &cfg.power_adc_channel,      "power_adc_channel"      },
    { VAR_INT8,   0,      -1,          1, &cfg.tri_ydir,               "tri_ydir"               },
    { VAR_UINT16, 1,       0,       2000, &cfg.tri_ymid,               "tri_ymid"               },
    { VAR_UINT16, 1,       0,       2000, &cfg.tri_ymin,               "tri_ymin"               },
    { VAR_UINT16, 1,       0,       2000, &cfg.tri_ymax,               "tri_ymax"               },
    { VAR_UINT16, 1,       0,       1000, &cfg.tri_ydel,               "tri_ydel"               },
    { VAR_UINT8,  0,       0,        255, &cfg.gbl_flg,                "gbl_flg"                },
    { VAR_INT8,   0,    -100,        100, &cfg.gbl_pgn,                "gbl_pgn"                },
    { VAR_INT8,   0,    -100,        100, &cfg.gbl_rgn,                "gbl_rgn"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_pmn,                "gbl_pmn"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_pmx,                "gbl_pmx"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_pmd,                "gbl_pmd"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_rmn,                "gbl_rmn"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_rmx,                "gbl_rmx"                },
    { VAR_UINT16, 0,     100,       3000, &cfg.gbl_rmd,                "gbl_rmd"                },
    { VAR_UINT8,  1,      10,        200, &cfg.al_barolr,              "al_barolr"              },
    { VAR_UINT8,  1,      10,        200, &cfg.al_snrlr,               "al_snrlr"               },
    { VAR_UINT8,  1,       0,         20, &cfg.al_debounce,            "al_debounce"            },
    { VAR_UINT16, 1,     100,       5000, &cfg.al_tobaro,              "al_tobaro"              },
    { VAR_UINT16, 1,     100,       5000, &cfg.al_tosnr,               "al_tosnr"               },
    { VAR_UINT8,  1,      50,        250, &cfg.as_lnchr,               "as_lnchr"               },
    { VAR_UINT8,  1,      50,        250, &cfg.as_clmbr,               "as_clmbr"               },
    { VAR_UINT8,  1,       0,        255, &cfg.as_trgt,                "as_trgt"                },
    { VAR_UINT8,  1,       5,         20, &cfg.as_stdev,               "as_stdev"               },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_GYRO][0],   "align_gyro_x"           },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_GYRO][1],   "align_gyro_y"           },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_GYRO][2],   "align_gyro_z"           },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_ACCEL][0],  "align_acc_x"            },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_ACCEL][1],  "align_acc_y"            },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_ACCEL][2],  "align_acc_z"            },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_MAG][0],    "align_mag_x"            },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_MAG][1],    "align_mag_y"            },
    { VAR_INT8,   0,      -3,          3, &cfg.align[ALIGN_MAG][2],    "align_mag_z"            },
    { VAR_UINT8,  0,       0,          3, &cfg.align_board_yaw,        "align_board_yaw"        },
    { VAR_UINT8,  0,       0,          3, &cfg.acc_hdw,                "acc_hdw"                },
    { VAR_FLOAT,  1,       0,        100, &cfg.acc_lpfhz,              "acc_lpfhz"              },
    { VAR_UINT8,  1,       1,        100, &cfg.acc_altlpfhz,           "acc_altlpfhz"           },
    { VAR_UINT8,  1,       1,        100, &cfg.acc_gpslpfhz,           "acc_gpslpfhz"           },
    { VAR_UINT16, 0,       0,        256, &cfg.gy_lpf,                 "gy_lpf"                 },
    { VAR_UINT16, 1,       1,      10000, &cfg.gy_gcmpf,               "gy_gcmpf"               },
    { VAR_UINT16, 1,       1,      10000, &cfg.gy_mcmpf,               "gy_mcmpf"               },
    { VAR_UINT8,  0,       5,        100, &cfg.gy_stdev,               "gy_stdev"               },
    { VAR_FLOAT,  1,       0,          1, &cfg.accz_vcf,               "accz_vcf"               },
    { VAR_FLOAT,  1,       0,          1, &cfg.accz_acf,               "accz_acf"               },
    { VAR_FLOAT,  1,       0,         10, &cfg.bar_lag,                "bar_lag"                },
    { VAR_FLOAT,  1,       0,          1, &cfg.bar_dscl,               "bar_dscl"               },
    { VAR_UINT8,  0,       0,          1, &cfg.bar_dbg,                "bar_dbg"                },
    { VAR_INT16,  1,  -18000,      18000, &cfg.mag_dec,                "mag_dec"                },
    { VAR_UINT8,  1,       1,          6, &cfg.mag_time,               "mag_time"               },
    { VAR_UINT8,  1,       0,          1, &cfg.mag_gain,               "mag_gain"               },
    { VAR_UINT32, 0,    1200,     115200, &cfg.gps_baudrate,           "gps_baudrate"           },
    { VAR_UINT8,  0,       0,          9, &cfg.gps_type,               "gps_type"               },
    { VAR_FLOAT,  1,       0,          1, &cfg.gps_ins_vel,            "gps_ins_vel"            },
    { VAR_UINT16, 1,       0,      10000, &cfg.gps_lag,                "gps_lag"                },
    { VAR_UINT8,  1,       5,         10, &cfg.gps_ph_minsat,          "gps_ph_minsat"          },
    { VAR_UINT8,  1,       0,         99, &cfg.gps_expo,               "gps_expo"               },
    { VAR_UINT8,  1,       1,        200, &cfg.gps_ph_settlespeed,     "gps_ph_settlespeed"     },
    { VAR_UINT8,  1,      10,         45, &cfg.gps_maxangle,           "gps_maxangle"           },
    { VAR_UINT8,  1,       1,         45, &cfg.gps_ph_brakemaxangle,   "gps_ph_brakemaxangle"   },
    { VAR_UINT8,  1,       1,         99, &cfg.gps_ph_minbrakepercent, "gps_ph_minbrakepercent" },
    { VAR_UINT16, 1,       1,        500, &cfg.gps_ph_brkacc,          "gps_ph_brkacc"          },
    { VAR_UINT16, 1,       0,       2000, &cfg.gps_wp_radius,          "gps_wp_radius"          },
    { VAR_UINT8,  1,       0,        200, &cfg.rtl_mnh,                "rtl_mnh"                },
    { VAR_UINT8,  1,      10,        200, &cfg.rtl_cr,                 "rtl_cr"                 },
    { VAR_UINT8,  1,       0,         50, &cfg.rtl_mnd,                "rtl_mnd"                },
    { VAR_UINT8,  1,       0,        100, &cfg.gps_rtl_flyaway,        "gps_rtl_flyaway"        },
    { VAR_UINT8,  1,      20,        150, &cfg.gps_yaw,                "gps_yaw"                },
    { VAR_UINT8,  1,       0,          1, &cfg.nav_rtl_lastturn,       "nav_rtl_lastturn"       },
    { VAR_UINT8,  1,      10,        200, &cfg.nav_speed_min,          "nav_speed_min"          },
    { VAR_UINT16, 1,      50,       2000, &cfg.nav_speed_max,          "nav_speed_max"          },
    { VAR_UINT8,  1,       2,         10, &cfg.nav_approachdiv,        "nav_approachdiv"        },
    { VAR_UINT8,  1,       0,        100, &cfg.nav_tiltcomp,           "nav_tiltcomp"           },
    { VAR_FLOAT,  1,       0,         10, &cfg.nav_ctrkgain,           "nav_ctrkgain"           },
    { VAR_UINT8,  1,       0,          1, &cfg.nav_controls_heading,   "nav_controls_heading"   },
    { VAR_UINT8,  1,       0,          1, &cfg.nav_tail_first,         "nav_tail_first"         },
    { VAR_UINT8,  1,       0,          1, &cfg.stat_clear,             "stat_clear"             },
    { VAR_UINT8,  1,       0,        200, &cfg.P8[PIDPOS],             "gps_pos_p"              },
    { VAR_UINT8,  0,       0,        200, &cfg.I8[PIDPOS],             "gps_pos_i"              },
    { VAR_UINT8,  0,       0,        200, &cfg.D8[PIDPOS],             "gps_pos_d"              },
    { VAR_UINT8,  1,       0,        200, &cfg.P8[PIDPOSR],            "gps_posr_p"             },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[PIDPOSR],            "gps_posr_i"             },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[PIDPOSR],            "gps_posr_d"             },
    { VAR_UINT8,  1,       0,        200, &cfg.P8[PIDNAVR],            "gps_nav_p"              },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[PIDNAVR],            "gps_nav_i"              },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[PIDNAVR],            "gps_nav_d"              },
    { VAR_UINT16, 1,    1000,       9000, &cfg.looptime,               "looptime"               },
    { VAR_UINT8,  1,       0,          1, &cfg.mainpidctrl,            "mainpidctrl"            },
    { VAR_UINT8,  1,       1,        100, &cfg.maincuthz,              "maincuthz"              },
    { VAR_UINT8,  1,       0,          9, &cfg.flt_rp,                 "flt_rp"                 },
    { VAR_UINT8,  1,       0,          9, &cfg.flt_yw,                 "flt_yw"                 },
    { VAR_UINT8,  1,       1,        100, &cfg.gpscuthz,               "gpscuthz"               },
    { VAR_UINT8,  1,       1,        200, &cfg.P8[PITCH],              "p_pitch"                },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[PITCH],              "i_pitch"                },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[PITCH],              "d_pitch"                },
    { VAR_UINT8,  1,       1,        200, &cfg.P8[ROLL],               "p_roll"                 },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[ROLL],               "i_roll"                 },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[ROLL],               "d_roll"                 },
    { VAR_UINT8,  1,       1,        200, &cfg.P8[YAW],                "p_yaw"                  },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[YAW],                "i_yaw"                  },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[YAW],                "d_yaw"                  },
    { VAR_UINT8,  1,       1,        200, &cfg.P8[PIDALT],             "p_alt"                  },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[PIDALT],             "i_alt"                  },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[PIDALT],             "d_alt"                  },
    { VAR_UINT8,  1,       1,        200, &cfg.P8[PIDLEVEL],           "p_level"                },
    { VAR_UINT8,  1,       0,        200, &cfg.I8[PIDLEVEL],           "i_level"                },
    { VAR_UINT8,  1,       0,        200, &cfg.D8[PIDLEVEL],           "d_level"                },
    { VAR_UINT8,  0,       0,          6, &cfg.snr_type,               "snr_type"               },
    { VAR_UINT8,  1,      20,        200, &cfg.snr_min,                "snr_min"                },
    { VAR_UINT16, 1,      50,        700, &cfg.snr_max,                "snr_max"                },
    { VAR_UINT8,  0,       0,          1, &cfg.snr_dbg,                "snr_dbg"                },
    { VAR_UINT8,  1,      10,         50, &cfg.snr_tilt,               "snr_tilt"               },
    { VAR_FLOAT,  1,       0,          1, &cfg.snr_cf,                 "snr_cf"                 },
    { VAR_UINT8,  1,       0,        200, &cfg.snr_diff,               "snr_diff"               },
    { VAR_UINT8,  1,       0,          1, &cfg.snr_land,               "snr_land"               },
    { VAR_UINT8,  0,       0,          1, &cfg.LED_invert,             "LED_invert"             },
    { VAR_UINT8,  0,       0,          3, &cfg.LED_Type,               "LED_Type"               },
    { VAR_UINT8,  0,       0,          1, &cfg.LED_Pinout,             "LED_pinout"             },
    { VAR_UINT8,  0,       1,         12, &cfg.LED_ControlChannel,     "LED_ControlChannel"     },
    { VAR_UINT8,  1,       0,          1, &cfg.LED_Armed,              "LED_ARMED"              },
    { VAR_UINT8,  0,       0,        255, &cfg.LED_Toggle_Delay1,      "LED_Toggle_Delay1"      },
    { VAR_UINT8,  0,       0,        255, &cfg.LED_Toggle_Delay2,      "LED_Toggle_Delay2"      },
    { VAR_UINT8,  0,       0,        255, &cfg.LED_Toggle_Delay3,      "LED_Toggle_Delay3"      },
    { VAR_UINT32, 0,       0, 0x7FFFFFFF, &cfg.LED_Pattern1,           "LED_Pattern1"           },
    { VAR_UINT32, 0,       0, 0x7FFFFFFF, &cfg.LED_Pattern2,           "LED_Pattern2"           },
    { VAR_UINT32, 0,       0, 0x7FFFFFFF, &cfg.LED_Pattern3,           "LED_Pattern3"           },
};
#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0) a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36)) r = 10;
    if (i < 0)
    {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    }
    else *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
// 06-May-2015 Crashpilot: Stripped down the Exponent part, since it's not used in this project
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    float value = 0.0f, sign = 1.0f, pow10 = 10.0f;
    while (white_space(*p)) p++;                                        // Skip leading white space, if any.
    if (*p == '-')                                                      // Get sign, if any.
    {
        sign = -1.0f;
        p++;
    }
    else if (*p == '+') p++;

    while (valid_digit(*p))                                             // Get digits before decimal point or exponent, if any.
    {
        value = value * 10.0f + (*p - '0');
        p++;
    }
    if (*p == '.')                                                      // Get digits after decimal point, if any.
    {
        p++;
        while (valid_digit(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p++;
        }
    }
    return sign * value;                                                // Return signed and scaled floating point result.
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0) x += 0.0005f;
    else x -= 0.0005f;
    value = (int32_t) (x * 1000.0f);                                    // Convert float * 1000 to an integer

    itoa(abs_int(value), intString1, 10);                               // Create string from abs of integer value

    if (value >= 0) intString2[0] = ' ';                                // Positive number, add a pad space
    else intString2[0] = '-';                                           // Negative number, add a negative sign

    if (strlen(intString1) == 1)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 2)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 3)
    {
        intString2[1] = '0';
        strcat(intString2, intString1);
    }
    else
    {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);
    return floatString;
}

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void PrintBox(uint8_t number, bool fillup)                       // Prints Out Boxname, left aligned and 8 chars. Cropping or filling with blank may occur
{
#define MaxCharInline 8
    uint8_t i = 0, k = 0;
    bool DoBlank = false;

    if (number >= CHECKBOXITEMS) return;
    while (k != number)
    {
        if(boxnames[i] == ';') k++;
        i++;
    }
    k = i;
    for (i = 0; i < MaxCharInline; i++)
    {
        if (boxnames[k + i] == ';')
        {
            if (fillup) DoBlank = true;
            else return;
        }
        if (DoBlank) printf(" ");
        else printf("%c", boxnames[k + i]);          
    }
}

static void PrintoutAUX(void)
{
    uint8_t  i, k, MaxAuxNumber = max(cfg.rc_auxch, 4);
    char     buf[3];
    uint32_t val;
    printf("ID|AUXCHAN :");
    for (i = 0; i < MaxAuxNumber; i++) printf(" %02u  ", i + 1);
    for (i = 0; i < CHECKBOXITEMS; i++)                                 // print out aux channel settings
    {
        printf("\r\n%02u|", i);
        PrintBox(i, true);
        printf(":");
        for (k = 0; k < MaxAuxNumber; k++)
        {
            strcpy(buf,"---");
            val = cfg.activate[i];
            val = val >> (k * 3);
            if (val & 1) buf[0] = 'L';
            if (val & 2) buf[1] = 'M';
            if (val & 4) buf[2] = 'H';
            printf(" %s ", buf);
        }
    }
    printf("\r\n");
}

static void cliAuxset(char *cmdline)
{
    uint32_t val = 1;
    bool     remove = false;
    char     *ptr = cmdline, buf[4];
    uint8_t  i, AuxChNr, ItemID, len = strlen(cmdline), MaxAuxNumber = max(cfg.rc_auxch, 4);

    if (len && *ptr == '-')
    {
        ptr++;
        if (*ptr == '-')
        {
            for (i = 0; i < CHECKBOXITEMS; i++) cfg.activate[i] = 0;
            printf("Wiped aux.\r\n");
            PrintoutAUX();
            return;
        }
        remove = true;
    }

    if (!len || len < 5)
    {
        printf("\r\nSet: auxset ID aux state(H/M/L)\r\n");
        printf("Remove: auxset -ID etc.\r\n");
        printf("Wipe all: auxset --\r\n");
        printf("Ex: auxset 1 4 h Sets Box 1 to Aux4 High\r\n\r\n");
        PrintoutAUX();
        return;
    }

    ItemID = atoi(ptr);
    ptr = strchr(ptr, ' ') + 1;
    AuxChNr = atoi(ptr);
    if (AuxChNr > MaxAuxNumber || !AuxChNr || ItemID >= CHECKBOXITEMS)
    {
        cliErrorMessage();
        return;
    }
    AuxChNr--;
    ptr = strchr(ptr, ' ') + 1;
    i   = AuxChNr * 3;
    switch(*ptr)
    {
    case 'L':
    case 'l':
        strcpy(buf,"LOW ");
        val <<= i;
        break;
    case 'M':
    case 'm':
        strcpy(buf,"MED ");
        val <<= (i + 1);
        break;
    case 'H':
    case 'h':
        strcpy(buf,"HIGH");
        val <<= (i + 2);
        break;
    default:
        cliErrorMessage();
        return;
    }
    cfg.activate[ItemID] |= val;                                        // Set it here in any case, so we can eor it away if needed
    if(remove)
    {
        cfg.activate[ItemID] ^= val;
        printf("Removing ");
    }
    else
    {
        printf("Setting ");
    }
    PrintBox(ItemID, false);
    printf(" Aux %02u %s\r\n", AuxChNr + 1, buf);
    PrintoutAUX();
}

#define GoodMixThresh (int32_t)(MixerMultiply * 0.02f)                  // Mixersum for each axis should be "0" but we define a margin here that still lets the mixer be ok in the gui printout
static void cliCMix(char *cmdline)
{
    int32_t i, motnum, paracnt, tmp[4];
    uint8_t len = strlen(cmdline);
    char    buf[16];
    char   *ptr;

    if (!len)
    {
    PrintCmix:                                                          // could be a void printcurrentmix(void) as well but this is shorter
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (motnum = 0; motnum < MAX_MOTORS; motnum++)
        {
            tmp[0] = cfg.customMixer[motnum].throttle;
            tmp[1] = cfg.customMixer[motnum].roll;
            tmp[2] = cfg.customMixer[motnum].pitch;
            tmp[3] = cfg.customMixer[motnum].yaw;
            if (!tmp[0]) break;
            printf("#%d:\t", motnum + 1);
            for (i = 0; i < 4; i++) printf("%s\t", ftoa(Int16MixToFloat(tmp[i]), buf));
            uartPrint("\r\n");
        }
        for (i = 0; i < 3; i++) tmp[i] = 0;                             // Fix by meister
        for (i = 0; i < motnum; i++)
        {
            tmp[0] += cfg.customMixer[i].roll;                          // we only need 3 elements of the 4 in "tmp"
            tmp[1] += cfg.customMixer[i].pitch;
            tmp[2] += cfg.customMixer[i].yaw;
        }
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++) uartPrint(abs_int(tmp[i]) > GoodMixThresh ? "NG\t" : "OK\t");
        uartPrint("\r\n");
        return;
    }
    else if (!strncasecmp(cmdline, "load", 4))
    {
        ptr = strchr(cmdline, ' ');
        if (ptr)
        {
            len = strlen(++ptr);
            for (i = 0; ; i++)
            {
                if (mixerNames[i] == NULL)
                {
                    cliErrorMessage();                                  // uartPrint("Invalid mixer type...\r\n");
                    break;
                } else if (!strncasecmp(ptr, mixerNames[i], len))
                {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    goto PrintCmix;                                     // goto is used to save codesize and avoid recursion "cliCMix("");"
                }
            }
        }
    }
    else
    {
        ptr = cmdline;
        i   = atoi(ptr) - 1;                                            // get motor number
        if (i >= 0 && i < MAX_MOTORS)
        {
            for (paracnt = 0; paracnt < 4; paracnt++)
            {
                ptr = strchr(ptr, ' ');
                if (!ptr) break;
                tmp[paracnt] = constrain_int(SpecialIntegerRoundUp(_atof(++ptr) * MixerMultiply), -32767, 32767); // "FloatMixToInt16"
            }
            if (paracnt == 4)
            {
                cfg.customMixer[i].throttle = tmp[0];
                cfg.customMixer[i].roll     = tmp[1];
                cfg.customMixer[i].pitch    = tmp[2];
                cfg.customMixer[i].yaw      = tmp[3];
                goto PrintCmix;                                        // goto is used to save codesize and avoid recursion "cliCMix("");"
            }
            else uartPrint("Invalid number of arguments\r\n");
        }
        else printf("Motor nr not in range 1 - %d\r\n", MAX_MOTORS);
    }
}

static void cliDefault(char *cmdline)
{
    uartPrint("Resetting to defaults\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting");
    systemReset(false);
}

static void cliDump(char *cmdline)
{
    printf("Config:\r\n");
    printf("FW: %s\r\n", FIRMWARE);
    PrintoutAUX();
    cliMixer(cmdline);
    cliCMix(cmdline);
    cliFeature(cmdline);
    cliMap(cmdline);
    cliSet(cmdline);
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len = strlen(cmdline);
    uint32_t mask = featureMask();

    if (!len)
    {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            if (mask & (1 << i)) printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available features: \r\n");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            printf("%s \r\n", featureNames[i]);
        }
        uartPrint("\r\n");
        return;
    }
    else
    {
        bool remove = false;
        bool fpass  = feature(FEATURE_PASS);
        if (cmdline[0] == '-')
        {
            remove = true;                                              // remove feature
            cmdline++;                                                  // skip over -
            len--;
        }

        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL)
            {
                cliErrorMessage();                                      // uartPrint("Invalid feature name\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0)
            {
                if (remove)
                {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                }
                else
                {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                if (fpass != feature(FEATURE_PASS)) cfg.pass_mot = 0;   // Reset to all motors if feature pass was changed
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;
    uartPrint("Available commands:\r\n\r\n");
    for (i = 0; i < CMD_COUNT; i++) printf("%s\t %s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint32_t i;
    char out[9];

    if (len == 8)
    {
        // uppercase it
        for (i = 0; i < 8; i++) cmdline[i] = toupper((unsigned char)cmdline[i]); // toupper(cmdline[i]);
        for (i = 0; i < 8; i++)
        {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++) out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len = strlen(cmdline);

    if (!len)
    {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++)
        {
            if (mixerNames[i] == NULL) break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++)
    {
        if (mixerNames[i] == NULL)
        {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0)
        {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode without saving\r\n");
    uartPrint("\r\nRebooting...");
    systemReset(false);                                                 // Just Reset without saving makes more sense
}

void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[16];                                                       // Reserve 32Bit more than 12Bytes. ftoa may need it

    switch (var->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)var->ptr;
        break;
    case VAR_INT8:
        value = *(int8_t *)var->ptr;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)var->ptr;
        break;
    case VAR_INT16:
        value = *(int16_t *)var->ptr;
        break;
    case VAR_UINT32:
        value = *(uint32_t *)var->ptr;
        break;
    case VAR_FLOAT:
        printf("%s", ftoa(*(float *)var->ptr, buf));
        if (full)
        {
            printf(" %s", ftoa((float)var->min, buf));
            printf(" %s", ftoa((float)var->max, buf));
        }
        return;                                                         // return from case for float only
    }
    printf("%d", value);
    if (full) printf(" %d %d", var->min, var->max);
}

static bool cliSetVar(const clivalue_t *var, int32_t intvalue, float fltvalue)
{
    bool inrange;

    if(var->type == VAR_FLOAT) inrange = fltvalue >= (float)var->min   && fltvalue <= (float)var->max;
    else                       inrange = intvalue >= (int32_t)var->min && intvalue <= var->max;

    if (!inrange) return false;
    switch (var->type)
    {
    case VAR_UINT8:                                                     // Note: The value range must be set correctly in the list so it doesn't overflow the datatype
        *(uint8_t *)var->ptr  = (uint8_t)intvalue;
        break;
    case VAR_INT8:
        *(int8_t *)var->ptr   = (int8_t)intvalue;
        break;
    case VAR_UINT16:
        *(uint16_t *)var->ptr = (uint16_t)intvalue;
        break;
    case VAR_INT16:
        *(int16_t *)var->ptr  = (int16_t)intvalue;
        break;
    case VAR_UINT32:
        *(uint32_t *)var->ptr = (uint32_t)intvalue;
        break;
    case VAR_FLOAT:
        *(float *)var->ptr = fltvalue;
        break;
    }
    return true;
}

static void cliSet(char *cmdline)
{
    uint32_t i, len = strlen(cmdline);
    const    clivalue_t *val;
    char     *eqptr = NULL;
    int32_t  value = 0;
    float    valuef = 0.0f;
    uint8_t  acc_hdwsave = cfg.acc_hdw, mag_gainsave = cfg.mag_gain;
    bool     needcal = false;

    if (!len || (len == 1 && cmdline[0] == '*'))
    {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            printf("%s = ", val->name);
            cliPrintVar(val, len);                                      // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    }
    else if ((eqptr = strstr(cmdline, "=")))                            // has equal, set var
    {
        eqptr++;
        len--;
        value  = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            if (!strncasecmp(cmdline, val->name, strlen(val->name)))
            {
                if (cliSetVar(val, value, valuef))                      // Feed both, cliSetVar knows what to do and reports back success or not
                {
                    printf("%s set to ", val->name);                    // Print out the success
                    cliPrintVar(val, 0);
                    if(acc_hdwsave != cfg.acc_hdw)                      // Check for hardwarechanges
                    {
                        needcal = cfg.acc_calibrated;                   // Acc was calibrated needs recal now
                        cfg.angleTrim[ROLL] = cfg.angleTrim[PITCH] = 0.0f;
                        cfg.accZero[ROLL] = cfg.accZero[PITCH] = cfg.accZero[YAW] = 0.0f;
                        cfg.sens_1G  = 1;
                        cfg.acc_calibrated = 0;
                    }
                    if(mag_gainsave != cfg.mag_gain)
                    {
                        needcal = cfg.mag_calibrated;                   // Mag was calibrated and gain is changed now needs recal
                        cfg.mag_calibrated = 0;
                    }
                    if (needcal) cliRecal();
                }
                else
                {
                    cliErrorMessage();                                  // uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliErrorMessage();                                              // uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void EEPROMandFloppyStatusReport(void)
{
    printf("EEPROM:\r\n");
    printf("Total : %d B FreeFlash: %d B\r\n", cfg.size, FLASH_PAGE_SIZE * FLASH_PAGES_FORCONFIG - cfg.size);
    printf("Config: %d B\r\n", cfg.size - FDByteSize);
    printf("Floppy: %d B, %d Datasets of %d possible\r\n\r\n", FDByteSize, cfg.FDUsedDatasets, FDByteSize / sizeof(wp_t));
}

static void printxyzcalval(float *off)
{
    uint8_t i;
    char    X = 'X';
    for (i = 0; i < 3; i++) printf("\r\n%c Offset: %d", X++, (int32_t)off[i]);

}

static void cliStatus(char *cmdline)
{
    uint8_t  i, k;
    uint32_t mask;
    uint16_t tmpu16;
    char     X = 'X';

    printf("\r\nSystem Uptime: %d sec, Volt: %d * 0.1V (%dS battery)\r\n", currentTimeMS / 1000, vbat, batteryCellCount);
    mask = sensorsMask();
    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++)
    {
        if (sensorNames[i] == NULL) break;
        if (mask & (1 << i)) printf("%s ", sensorNames[i]);
    }
    printf("\r\nCycle Time: %d, I2C Errors: %d\r\n\r\n", (int16_t)AvgCyclTime, i2cGetErrorCounter());
    EEPROMandFloppyStatusReport();
    printf("SENSORS:");
    printf("\r\nGyro ");
    if(havel3g4200d) printf("L3G4200D");
    else printf("MPU");
    printf("\r\nActual");
    printxyzcalval(gyroZero);
    if (cfg.ShakyDataAvail)
    {
        printf("\r\nFallback");
        printxyzcalval(cfg.ShakyGyroZero);
    }
    printf("\r\nTemp: %d",(int32_t)telemTemperature1);
    if (sensors(SENSOR_ACC))
    {
        printf("\r\n\r\nAcc %s", accNames[accHardware]);
        printf("\r\nStatus: ");
        if(cfg.acc_calibrated)
        {
            printf("calibrated");
            printxyzcalval(cfg.accZero);
            printf("\r\n1G val: %d", cfg.sens_1G);
        }
        else cliRecal();
    }
    if (sensors(SENSOR_MAG))
    {
        printf("\r\n\r\nMag HMC5883");
        printf("\r\nStatus: ");
        if (cfg.mag_calibrated)
        {
            printf("calibrated");
            printxyzcalval(cfg.magZero);
            for (i = 0; i < 3; i++) printf("\r\n%c Gain*1000: %d", X++, (int32_t)(magCal[i] * 1000));
        }
        else cliRecal();
        printf("\r\nGain ");
        if (maggainok) printf("OK"); else printf("NOT OK");
    }
    if (sensors(SENSOR_BARO))
    {
        printf("\r\n\r\nBaro ");
        if(baro.baro_type == 1) printf("BMP085");
        else printf("MS5611");
        printf("\r\nTemp: %d",(int32_t)BaroActualTemp);
    }
    printf("\r\n\r\nSTATS:");
    if (sensors(SENSOR_BARO) || sensors(SENSOR_GPS))
    {
        if (sensors(SENSOR_GPS))
        {
            printf("\r\nGPS:");
            tmpu16 = (uint16_t)((float)cfg.MAXGPSspeed * 0.036f);
            printf("\r\nMax Dist: %d m", cfg.GPS_MaxDistToHome );
            printf("\r\nMax Speed: %dcm/s = %dKm/h", cfg.MAXGPSspeed, tmpu16);
        }
        if (sensors(SENSOR_BARO))
        {
            printf("\r\nHight:");
            printf("\r\nMax Alt AGL: %d m", cfg.MaxAltMeter);
            printf("\r\nMin Alt AGL: %d m", cfg.MinAltMeter);
        }
    }
    printf("\r\nMotor:\r\n");
    printf("Actual Range: %d - %d at %d Hz PWM.\r\n", cfg.esc_min, cfg.esc_max, cfg.esc_pwm);
    tmpu16 = (cfg.esc_max - cfg.esc_min) / 100;
    if(motorpercent[0])
    {
        k = min(NumberOfMotors, MAX_MONITORED_MOTORS);
        for (i = 0; i < k; i++) printf("Mot: %d Session Usage: %d%% Abs PWM: %d Rel to PWM range: %d%%\r\n", i + 1, motorpercent[i], motorabspwm[i],(motorabspwm[i] - cfg.esc_min) / tmpu16);
    } else printf("No Stats\r\n");
}

static void cliWpflush(char *cmdline)
{
    EEPROMandFloppyStatusReport();
    FloppyClear();
    printf("Flushing.\r\n\r\n");
    EEPROMandFloppyStatusReport();
}

static void cliVersion(char *cmdline)
{
    uartPrint(FIRMWARE);
}

// LCD functions
// Were designed to work with serial Sparkfun LCD-09395. Since it has only unidirectional communication the initialization
// can fail when the module is confused with other serial data from telemetry. The user can issue a re-initialization by
// putting Throttle and Pitch stick to maximum. An stick commands oversight is included in the HarakiriREADME folder.
// "Johannes" ported the I2C Oled functions from multiwii. Together with cGiesen the Oled compatibility was improved.
// See: http://www.fpv-treff.de/viewtopic.php?f=18&t=2346
// The printf.c has been altered for oled support and the I2C initialization autodetects I2C adr. in drv_system.c.
// Considerations: I2C on stm is 3,3V. All sensors are on that I2C line. IMHO you better stick to the serial one..
// The LCD now remembers the Dataset on EEPROM save, so you don't have to scroll through the list to find that parameter again
// you are currently working on.
// Limitations of the LCD function:
// - first element of valueTable will be displayed even if lcd parameter is set to 0
void serialOSD(void)
{
#define RcEndpoint 50
#define RcMax cfg.rc_maxchk - RcEndpoint
#define RcMin cfg.rc_minchk + RcEndpoint
  
    uint8_t  input, lastinput, brake, brakeval, speeduptimer, exitLCD;
    int8_t   adder;
    uint16_t i, k;
    const clivalue_t *tablePtr;

RestartLCD:                                                             // c++ doesn't like that very much
    input = 0, lastinput = 0, brake = 0, brakeval = 0, speeduptimer = 0;// Preset values
    cfg.LCDcurrDataset = constrain_int(cfg.LCDcurrDataset, 0,(VALUE_COUNT - 1)); // Ensure valid range in case of EEPROM fail
    tablePtr = &valueTable[cfg.LCDcurrDataset];
    LCDinit();
    printf(FIRMWAREFORLCD);                                             // Defined in mw.h
    LCDline2();
    k = 0;
    for (i = 0; i < VALUE_COUNT; i++) if (tablePtr->lcd) k++;           // Count parameters for LCD
    printf("%d LCD Datasets", k);
    while (input != 1) if (DoGetRc50HzTimer() && rcData[PITCH] < RcMax && rcData[THROTTLE] < RcMin) input = 1; // Wait for Pitch to move back and Thr down when coming from restart
    if (k)
    {
        LCDclear();
        printf("%s", tablePtr->name);                                   // Display first item anyway (even if lcd == 0) no need for special attention
        LCDline2();
        cliPrintVar(tablePtr, 0);
        exitLCD = 0;
    }
    else exitLCD = 1;                                                   // No Datasets Quit don't save     

    while (!exitLCD)
    {
        if (DoGetRc50HzTimer())                                         // Start of 50Hz Loop Gathers all Rc Data
        {
            LED1_TOGGLE
            LED0_TOGGLE
            if (rcData[THROTTLE] < RcMin && rcData[PITCH] > RcMax)
            {
                if (rcData[YAW] > RcMax) exitLCD = 1;                   // Quit don't save
                if (rcData[YAW] < RcMin) exitLCD = 2;                   // Quit and save
            }
            
            if (rcData[THROTTLE] > RcMax && rcData[PITCH] > RcMax)      // Serial Lcd can be confused by telemtry data
            goto RestartLCD;                                            // So user can force re-initialization
            
            input = 0;
            if (!exitLCD)
            {
                if (rcData[PITCH] < RcMin)      input = 1;              // Pitch wins over Roll here
                else if (rcData[PITCH] > RcMax) input = 2;
                else if (rcData[ROLL]  < RcMin) input = 4;
                else if (rcData[ROLL]  > RcMax) input = 8;
            }
            
            if (lastinput == input)                                     // Adjust Inputspeed
            {
                if (speeduptimer > 100)
                {
                    brakeval = 8;
                    adder    = 10;
                }
                else
                {
                    brakeval = 17;
                    adder    = 1;
                    speeduptimer++;
                }
            }
            else
            {
                brakeval     = 0;
                adder        = 1;
                speeduptimer = 0;
            }
            lastinput = input;
            brake++;

            if (brake >= brakeval) brake = 0;
            else input = 0;
            
            if (input)
            {
                if (input & 3)                                          // Pitch
                {
                    do                                                  // Search for next Dataset
                    {
                        if (input == 1)                                 // Pitch stick lo, next dataset
                        {
                            cfg.LCDcurrDataset++;
                            if (cfg.LCDcurrDataset == VALUE_COUNT) cfg.LCDcurrDataset = 0;
                        }
                        else                                            // Pitch stick hi, previous dataset
                        {
                            if (!cfg.LCDcurrDataset) cfg.LCDcurrDataset = VALUE_COUNT;
                            cfg.LCDcurrDataset--;
                        }
                        tablePtr = &valueTable[cfg.LCDcurrDataset];
                    }
                    while (!tablePtr->lcd);
                    LCDclear();
                    printf("%s", tablePtr->name);
                }
                else                                                    // Roll
                {
                    if (input == 4) LCDchangeval(tablePtr, -adder);     // Roll left Substract within the limit
                    else LCDchangeval(tablePtr, adder);                 // Roll right Add within the limit
                }
                LCDline2();
                cliPrintVar(tablePtr, 0);
            }
        }                                                               // End of 50Hz Loop
    }
    delay(500);
    LCDclear();
    printf(" Rebooting and ");
    LCDline2();
    switch (exitLCD)
    {
    case 1:
        printf(" NOT Saving");
        break;
    case 2:
        printf("!!!!!Saving!!!!!");
        writeParams(0);
        break;
    }
    delay(1500);
    LCDoff();                                                           // Reset coming from LCD so reset it.
    systemReset(false);
}

static void LCDchangeval(const clivalue_t *var, const int8_t adder)
{
    int32_t value  = *(uint32_t *)var->ptr;                             // Do case VAR_UINT32 here to avoid "uninitialized variable" warning in GCC
    float   valuef = *(float *)var->ptr;                                // Do case VAR_FLOAT here to avoid "uninitialized variable" warning in GCC

    switch (var->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)var->ptr;
        break;
    case VAR_INT8:
        value = *(int8_t *)var->ptr;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)var->ptr;
        break;
    case VAR_INT16:
        value = *(int16_t *)var->ptr;
        break;
    default:
        break;
    }
    value  = constrain_int(value  + adder                 , var->min, var->max);
    valuef = constrain_flt(valuef + (float)adder / 1000.0f, var->min, var->max);
    cliSetVar(var, value, valuef);
}

// Codes for Sparkfun LCD-09395
#define CommandCharFE  0xFE                                             // Prefix for most commands
#define CommandChar7C  0x7C                                             // Prefix for Brightness command and Baud
#define FullBright     0x9D                                             // Brightness Range: 0x80 - 0x9D
#define DisplayON      0x0C
#define DisplayOFF     0x08
#define ClearScr       0x01                                             // Clears Line 1 and 2
#define CursorLine1    0x80
#define CursorLine2    0xC0
#define LCDdelay         12                                             // 12ms Delay between commands

static void SendSerialLCD(uint8_t val)
{
    delay(LCDdelay);
    uartWrite(val);
}
static void SendSerialLCDCmdFE(uint8_t val)
{
    SendSerialLCD(CommandCharFE);
    SendSerialLCD(val);
}  

static void LCDinit(void)                                               // changed Johannes
{
    if (!initI2cLCD(true))                                              // Will set i2cLCD
    {
        serialInit(9600);                                               // INIT LCD HERE
        LCDoff();
        SendSerialLCD(CommandChar7C);
        SendSerialLCD(FullBright);
        LCDclear();
        SendSerialLCDCmdFE(DisplayON);
    }
}

void LCDoff(void)
{
    if (i2cLCD) i2c_clear_OLED();                                       // Johannes
    else SendSerialLCDCmdFE(DisplayOFF);
}

static void LCDclear(void)                                              // clear screen, cursor line 1, pos 0
{
    if (i2cLCD)
    {
        i2c_clear_OLED();                                               // Johannes
        i2c_clr_row(6);
    }
    else
    {
        SendSerialLCDCmdFE(ClearScr);
        SendSerialLCDCmdFE(CursorLine1);
    }
    delay(LCDdelay);
}

static void LCDline2(void)                                              // Sets LCD Cursor to line 2 pos 0
{
    if (i2cLCD) i2c_clr_row(7);                                         // Johannes
    else
    {
        SendSerialLCDCmdFE(CursorLine2);
        delay(LCDdelay);
        printf("                ");                                     // Clear Line
        SendSerialLCDCmdFE(CursorLine2);                                // Cursor moved, reset it.
    }
    delay(LCDdelay);    
}

void cliProcess(void)
{
    static uint32_t bufferIndex = 0;
    char   cliBuffer[48], dummy;
    writeAllMotors(cfg.esc_moff);                                       // Set all motors to OFF just to be sure if user is messing in cli without saving
    memset(cliBuffer, 0, sizeof(cliBuffer));
    uartPrint("\r\nEntering CLI Mode, type 'exit' or 'save' to return, or 'help' \r\n\r\n");
    cliVersion(&dummy);
    uartPrint("\r\n\r\n");
    cliHelp(&dummy);
    cliPrompt();
    for(; ;)                                                            // CLI ENDLESS LOOP THAT SAVES STACK
    {
        while (uartAvailable())
        {
            uint8_t c = uartRead();
            if (c == '\t' || c == '?')
            {
                const clicmd_t *cmd, *pstart = NULL, *pend = NULL;      // do tab completion
                int i = bufferIndex;
                for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++)
                {
                    if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0)) continue;
                    if (!pstart) pstart = cmd;
                    pend = cmd;
                }
                if (pstart)                                             // Buffer matches one or more commands
                {
                    for (; ; bufferIndex++)
                    {
                        if (pstart->name[bufferIndex] != pend->name[bufferIndex]) break;
                        if (!pstart->name[bufferIndex])
                        {
                            cliBuffer[bufferIndex++] = ' ';             // Unambiguous -- append a space */
                            break;
                        }
                        cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                    }
                }
                if (!bufferIndex || pstart != pend)
                {
                    uartPrint("\r\033[K");                              // Print list of ambiguous matches
                    for (cmd = pstart; cmd <= pend; cmd++)
                    {
                        uartPrint(cmd->name);
                        uartWrite('\t');
                    }
                    cliPrompt();
                    i = 0;                                              // Redraw prompt
                }
                for (; i < bufferIndex; i++) uartWrite(cliBuffer[i]);
            }
            else if (!bufferIndex && c == 4)
            {
                cliExit(cliBuffer);
                return;
            }
            else if (c == 12)                                           // clear screen
            {
                uartPrint("\033[2J\033[1;1H");
                cliPrompt();
            }
            else if (bufferIndex && (c == '\n' || c == '\r'))           // enter pressed
            {
                clicmd_t *cmd = NULL;
                clicmd_t target;
                uartPrint("\r\n");
                cliBuffer[bufferIndex] = 0;                             // null terminate
                target.name  = cliBuffer;
                target.param = NULL;
                cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
                if (cmd) cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else cliErrorMessage();
                memset(cliBuffer, 0, sizeof(cliBuffer));
                bufferIndex = 0;
                cliPrompt();
            }
            else if (c == 127)
            {
                if (bufferIndex)                                        // backspace
                {
                    cliBuffer[--bufferIndex] = 0;
                    uartPrint("\010 \010");
                }
            }
            else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126)
            {
                if (!bufferIndex && c == 32) continue;
                cliBuffer[bufferIndex++] = c;
                uartWrite(c);
            }
        }
    }
}

// ************************************************************************************************************
// TestScan on the I2C bus
// ************************************************************************************************************
#define MMA8452_ADDRESS     0x1C
#define HMC5883L_ADDRESS    0x1E                        // 0xA
#define DaddyW_SONAR        0x20                        // Daddy Walross Sonar
#define EagleTreePowerPanel 0x3B                        // Eagle Tree Power Panel
#define OLD1_ADDRESS        0x3C                        // OLED at address 0x3C in 7bit
#define OLD2_ADDRESS        0x3D                        // OLED at address 0x3D in 7bit
#define ADXL345_ADDRESS     0x53
#define BMA180_ADDRESS      0x64                        // don't respond ??
#define MPU6050_ADDRESS     0x68                        // 0x75     or 0x68  0x15
#define L3G4200D_ADDRESS    0x68                        // 0x0f
#define BMPandMS_ADDRESS    0x77                        // 0xD0
#define MBandSRF_ADDRESS    0x70                        // Devantech I2C SONAR, Standard address 0x70 (SRF02, SRF235, SRF08, SRF10)

/*
new May 15 2013 Johannes && Some stuff from me as well :)
*/
static void cliScanbus(char *cmdline)
{
    bool    ack, msbaro = false, L3G4200D = false;
    uint8_t address, nDevices = 0, sig, bufsnr[2];
    char    buf[20];

    printf("\r\nScanning I2C-Bus\r\n\r\n");
    i2cFastSpeed(false);                                // set I2C Standard mode
    for(address = 1; address < 127; address++ )
    {
        sig = 0;
        ack = i2cRead(address, address, 1, &sig);       // Do a blind read. Perhaps it's sufficient? Otherwise the hard way...
        if(!ack)                                        // Try to get ack with more force
        {
            switch(address)
            {
            case MMA8452_ADDRESS:
                i2cRead(MMA8452_ADDRESS, 0x0D, 1, &sig);
                if (sig == 0x2A || sig == 0x1A) ack = true;
                else ack = false;
                break;
            case DaddyW_SONAR:
                ack = i2cRead(DaddyW_SONAR, 0x32, 2, bufsnr);
                break;
            case BMPandMS_ADDRESS:
                ack = i2cRead(BMPandMS_ADDRESS, 0xA0, 1, &sig);  // Sig is irrelevant?
                msbaro = ack;
                break;
            case MPU6050_ADDRESS:
                i2cRead(MPU6050_ADDRESS, 0x0F, 1, &sig);
                if (sig == 0xD3)
                {
                    ack = true;
                    L3G4200D = true;
                }
                break;
            }
        }
        if (ack)
        {
            printf("I2C device at 0x");
            if (address<16) printf("0");
            printf("%x",address);
            switch (address)
            {
            case MMA8452_ADDRESS:                       // Detection altered
                strcpy(buf, "MMA8452");
                break;
            case HMC5883L_ADDRESS:
                strcpy(buf, "HMC5883L");
                break;
            case DaddyW_SONAR:                          // Summarize as "Sonar"
            case MBandSRF_ADDRESS:
                strcpy(buf, "Sonar");
                break;
            case EagleTreePowerPanel:                   // Summarize as "Display"
            case OLD1_ADDRESS:
            case OLD2_ADDRESS:
                strcpy(buf, "Display");
                break;
            case ADXL345_ADDRESS:                       // ADXL added
                strcpy(buf, "ADXL345");
                break;
            case BMA180_ADDRESS:                        // Sensor currently not supported by a driver
                strcpy(buf, "BMA180");
                break;
            case MPU6050_ADDRESS:
                if (L3G4200D) strcpy(buf, "L3G4200D");
                else strcpy(buf, "MPU3050/MPU6050");
                break;
            case BMPandMS_ADDRESS:
                if(msbaro) strcpy(buf, "MS5611");
                else strcpy(buf, "BMP085");
                break;
            default:                                    // Unknown case added
                strcpy(buf, "UNKNOWN");
                break;
            }
            printf(" probably %s \r\n",buf);
            nDevices++;
        }
        delay(50);
    }
    uartPrint("\r\n");
    i2cFastSpeed(true);                                 // set I2C I2C Fast mode
    if (!nDevices) printf("No I2C devices\r\n");
    else printf("%d Devices\r\n",nDevices);
}

// ************************************************************************************************************
// More or less Dumb passthrough for gps config - Hacky but working
// Maybe we miss a byte or something but GPS communication is checksummed, so GPS and Tool will keep it valid.
// ************************************************************************************************************
static void cliPassgps(char *cmdline)
{
    uint8_t  serbyte, i;
    uint32_t wantedbaud = 0;
    bool     HaveMTK;

    if (!feature(FEATURE_GPS))                          // Don't ask for sensors(gps) here, it may not be initialized
    {
        printf("GPS not enabled!\r\n");
        return;
    }

    if (cfg.gps_type == 2 || cfg.gps_type == 3) HaveMTK = true;
    else HaveMTK = false;

    if (!strlen(cmdline))
    {
        printf("Need option\r\n");
        printf("Writing ubx conf with different baudsetting must fail.\r\n");
        printf("Set Baud of planned config now. Repower after ucenter.\r\n");
        printf("MTK go with '0', set type to NMEA, set Baud of FW.\r\n\r\n");
        printf("Select Ublox Options\r\n\r\n");
        printf("0 No Options. All GPS\r\n");
        printf("1 UBX Force Sgnlstrngth\r\n");
        printf("2 UBX 115K Bd\r\n");
        printf("3 UBX  57K Bd\r\n");
        printf("4 UBX  38K Bd\r\n");
        printf("5 UBX  19K Bd\r\n\r\n");
        if (HaveMTK) printf("Actual MTK 57K Bd.\r\n");// GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
        else
        {
            if (!cfg.gps_type) printf("Actual NMEA");
            else printf("Actual UBLOX");
            printf(" %d Bd.\r\n", cfg.gps_baudrate);
        }
    }
    else
    {
        if (HaveMTK && cmdline[0] != '0')
        {
            cliErrorMessage();
            return;
        }
        switch(cmdline[0])
        {
        case '0':
            if (HaveMTK) wantedbaud = 57600;
            else wantedbaud = cfg.gps_baudrate;
            break;
        case '1':
            wantedbaud = cfg.gps_baudrate;
            UblxSignalStrength();
            break;
        case '2':
            wantedbaud = 115200;
            break;
        case '3':
            wantedbaud = 57600;
            break;
        case '4':
            wantedbaud = 38400;
            break;
        case '5':
            wantedbaud = 19200;
            break;
        default:
            cliErrorMessage();
            return;
        }

        if(!HaveMTK)
        {
            if (wantedbaud == cfg.gps_baudrate) printf("Keeping GPS Bd: %d.", wantedbaud);
            else
            {
                printf("Setting %d Bd.", wantedbaud);
                UbloxForceBaud(wantedbaud);
            }
        }

        printf("\r\nProceeding. Close Terminal.");
        delay(2000);
        HaveNewGpsByte = false;
        serialInit(wantedbaud);                         // Set USB Baudrate
        uart2Init(wantedbaud, GPSbyteRec, false);       // Set GPS Baudrate and callbackhandler
        i = 0;
        while (i < 3)
        {
            if (uartAvailable())
            {
                serbyte = uartRead();                   // Read from USB
                if (serbyte == '\r') i++;               // Break out with 3 times RETURN
                else i = 0;
                uart2Write(serbyte);                    // Write to GPS
                while (!uart2TransmitEmpty())
                {
                    ;                                   // wait for GPS Byte to be send
                }
                LED1_TOGGLE
            }
            if (HaveNewGpsByte)
            {
                serbyte        = NewGPSByte;            // Read from GPS
                HaveNewGpsByte = false;
                uartWrite(serbyte);                     // Write to USB
                LED0_TOGGLE
            }
        }
        uartPrint("Rebooting");
        systemReset(false);
    }
}

static void GPSbyteRec(uint16_t c)
{
    NewGPSByte = c;
    HaveNewGpsByte = true;
}

static void cliFlash(char *cmdline)
{
    printf("Close terminal & flash\r\n");
    systemReset(true);                                  // reboot to bootloader
}

static void cliErrorMessage(void)
{
    uartPrint("That was Harakiri, try 'help'");
}

static void cliRecal(void)
{
    uartPrint(" Needs Calibr.");
}

// MAVLINK STUFF AFFECTING CLI GOES HERE
bool baseflight_mavlink_send_paramlist(bool Reset)
{
    static  int16_t i = 0;
    float   value = 0;
    char    buf[17];                                    // Always send 16 chars reserve one zero byte
    mavlink_message_t msg;
    const   clivalue_t *tableptr;

    if(Reset)
    {
        i = 0;
        AllowProtocolAutosense = true;
        return true;                                    // Return status not relevant but true because the "Reset" was a success
    }
    AllowProtocolAutosense = false;                     // Block Autodetect during transmission
    if(i < 0 || i > ((int16_t)VALUE_COUNT - 1)) return true; // Done with error but DONE
    memset (buf, 0, 17);                                // Fill with 0 For Stringtermination
    tableptr = &valueTable[i];
    memcpy (buf, tableptr->name, min(strlen(tableptr->name), 16)); // Copy max 16 Bytes
    switch(tableptr->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)tableptr->ptr;
        break;
    case VAR_INT8:
        value = *(int8_t *)tableptr->ptr;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)tableptr->ptr;
        break;
    case VAR_INT16:
        value = *(int16_t *)tableptr->ptr;
        break;
    case VAR_UINT32:
        value = *(uint32_t *)tableptr->ptr;
        break;
    case VAR_FLOAT:
        value = *(float *)tableptr->ptr;
        break;
    }
    mavlink_msg_param_value_pack(MLSystemID, MLComponentID, &msg, buf, value, MAVLINK_TYPE_FLOAT, VALUE_COUNT, i);
		baseflight_mavlink_send_message(&msg);
    i++;
    if (i == VALUE_COUNT)
    {
        i = 0;
        AllowProtocolAutosense = true;                  // Allow Autodetection again
        return true;                                    // I am done
    }
    else return false;
}

bool baseflight_mavlink_set_param(mavlink_param_set_t *packet)
{
    mavlink_message_t msg;
    const    clivalue_t *tableptr;
    uint16_t i;
    bool     returnval = false;
    float    value;

    if (strcmp(packet->param_id, ""))                   // Filter trash Message here
    {
        for (i = 0; i < VALUE_COUNT; i++)
        {
            tableptr = &valueTable[i];
            if (!strncasecmp(tableptr->name, packet->param_id, min(strlen(tableptr->name), 16))) // Strings match ?
            {
                value = packet->param_value;
                if (cliSetVar(tableptr, value, value))
                {
                    mavlink_msg_param_value_pack(MLSystemID, MLComponentID, &msg, packet->param_id, value, packet->param_type, VALUE_COUNT, i); // Report parameter back if everything was fine.
                    baseflight_mavlink_send_message(&msg);
                    returnval = true;
                }
            }
        }
    }
    return returnval;
}
