#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "common/axis.h"
#include "common/utils.h"
#include "common/maths.h"
#include "config/runtime_config.h"
#include "io/beeper.h"
#include "io/rc_controls.h"
#include "flight/pid.h"
#include "flight/navigation.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "flight/imu.h"
}

extern "C" {
    uint32_t fixedMicros;
    uint8_t GPS_numSat;
    int32_t GPS_coord[2];
    int32_t GPS_hold[2];
    extern navigationMode_e nav_mode;
    
    int16_t magADC[XYZ_AXIS_COUNT];
    int16_t heading;

    void onGpsNewData();
    void GPS_set_next_wp(int32_t *lat, int32_t *lon);
    void GPS_reset_nav();
    void navigationInit(gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile);
    
    void imuInit();
    void imuUpdate(rollAndPitchTrims_t *accelerometerTrims);
    uint32_t millis();
}

// FIXME Cut&pasted from config.c
static void resetPidProfile(pidProfile_t *pidProfile)
{
    pidProfile->pidController = 0;
    
    pidProfile->P8[ROLL] = 40;
    pidProfile->I8[ROLL] = 30;
    pidProfile->D8[ROLL] = 23;
    pidProfile->P8[PITCH] = 40;
    pidProfile->I8[PITCH] = 30;
    pidProfile->D8[PITCH] = 23;
    pidProfile->P8[YAW] = 85;
    pidProfile->I8[YAW] = 45;
    pidProfile->D8[YAW] = 0;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15; // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0; // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34; // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 14; // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 53; // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 25; // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 33; // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 83; // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 90;
    pidProfile->I8[PIDLEVEL] = 10;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 120;
    pidProfile->I8[PIDVEL] = 45;
    pidProfile->D8[PIDVEL] = 1;
    
    pidProfile->yaw_p_limit = YAW_P_LIMIT_MAX;
    pidProfile->dterm_cut_hz = 0;
    pidProfile->pterm_cut_hz = 0;
    pidProfile->gyro_cut_hz = 0;
    
    pidProfile->P_f[ROLL] = 1.5f;     // new PID with preliminary defaults test carefully
    pidProfile->I_f[ROLL] = 0.4f;
    pidProfile->D_f[ROLL] = 0.03f;
    pidProfile->P_f[PITCH] = 1.5f;
    pidProfile->I_f[PITCH] = 0.4f;
    pidProfile->D_f[PITCH] = 0.03f;
    pidProfile->P_f[YAW] = 2.5f;
    pidProfile->I_f[YAW] = 1.0f;
    pidProfile->D_f[YAW] = 0.00f;
    pidProfile->A_level = 5.0f;
    pidProfile->H_level = 3.0f;
    pidProfile->H_sensitivity = 75;
}
void resetGpsProfile(gpsProfile_t *gpsProfile)
{
    gpsProfile->gps_wp_radius = 200;
    gpsProfile->gps_lpf = 20;
    gpsProfile->nav_slew_rate = 30;
    gpsProfile->nav_controls_heading = 1;
    gpsProfile->nav_speed_min = 100;
    gpsProfile->nav_speed_max = 300;
    gpsProfile->ap_mode = 40;
}

bool sensors(uint32_t mask)
{
    return (SENSOR_ACC | SENSOR_GYRO | SENSOR_GPS | SENSOR_MAG) & mask;
}

int main(int argc, char *argv[]) {
    UNUSED(argc);
    
    FILE *fp = fopen(argv[1], "r");
    if (fp == NULL) {
        fprintf(stderr, "Can't open file\n");
        exit(1);
    }
    
    gpsProfile_t gpsProfile;
    resetGpsProfile(&gpsProfile);
    pidProfile_t pidProfile;
    resetPidProfile(&pidProfile);
    //gpsUseProfile(&gpsProfile);
    navigationInit(&gpsProfile, &pidProfile);
    
    ENABLE_STATE(GPS_FIX);
    ENABLE_STATE(GPS_FIX_HOME);
    ENABLE_STATE(SMALL_ANGLE);
    ENABLE_ARMING_FLAG(ARMED);
    
    ENABLE_FLIGHT_MODE(HORIZON_MODE);
    ENABLE_FLIGHT_MODE(GPS_HOLD_MODE);
    
    char line[1024];
    bool initialized = false;
    
    rollAndPitchTrims_t rollAndPitchTrims;
    rollAndPitchTrims.values.roll = 0;
    rollAndPitchTrims.values.pitch = 0;

    imuRuntimeConfig_t imuRuntimeConfig;
    imuRuntimeConfig.gyro_cmpf_factor = 600;
    imuRuntimeConfig.gyro_cmpfm_factor = 250;
    imuRuntimeConfig.acc_lpf_factor = 4;
    imuRuntimeConfig.acc_unarmedcal = 1;
    imuRuntimeConfig.small_angle = 25;
    
    accDeadband_t accDeadband = { 40, 40 };
    imuConfigure(
                 &imuRuntimeConfig,
                 &pidProfile,
                 &accDeadband,
                 5.0,
                 800
                 );
    imuInit();

    int32_t oldGPS_coord[2] = { 0, 0 };
    
    while (fgets(line, sizeof(line), fp) != NULL) {
        int fieldNum = 0;
        char *p = line;
        
//        puts(p);
        
        while (p) {
            char *end = strchr(p, ',');
            
            switch (fieldNum) {
                case 1:
                    // Time (microsecond)
                    fixedMicros = atoi(p);
                    break;
                case 15:
                    magADC[0] = atoi(p);
                    break;
                case 16:
                    magADC[1] = atoi(p);
                    break;
                case 17:
                    magADC[2] = atoi(p);
                    break;
                case 34:
                    // GPS_numSat
                    GPS_numSat = atoi(p);
                    break;
                case 35:
                    // GPS_coord[0]
                    GPS_coord[0] = llrint(atof(p) * 10000000.0);
                    break;
                case 36:
                    // GPS_coord[1]
                    GPS_coord[1] = llrint(atof(p) * 10000000.0);
                    break;
            }
            p = end;
            if (p != NULL)
                p++;
            fieldNum++;
        }
        if (GPS_coord[0] != oldGPS_coord[0] || GPS_coord[1] != oldGPS_coord[1]) {
            if (!initialized) {
                initialized = true;
                GPS_hold[0] = GPS_coord[0];
                GPS_hold[1] = GPS_coord[1];
                GPS_set_next_wp(&GPS_hold[0], &GPS_hold[1]);
                nav_mode = NAV_MODE_POSHOLD;
                GPS_reset_nav();
            }
            //printf("T=%d lat=%d lon=%d\n", millis(), GPS_coord[0], GPS_coord[1]);
            printf("heading=%d\n", heading);
            onGpsNewData();
            updateGpsStateForHomeAndHoldMode();
            printf("GPS_angle: ROLL=%d, PITCH=%d\n", GPS_angle[0], GPS_angle[1]);
            imuUpdate(&rollAndPitchTrims);

            oldGPS_coord[0] = GPS_coord[0];
            oldGPS_coord[1] = GPS_coord[1];
        }
    }
    fclose(fp);
}

// STUBS

extern "C" {
    uint32_t millis(void) {
        return fixedMicros / 1000;
    }
    
    void resetMillis(void) {
        fixedMicros = 0;
    }

    uint32_t micros(void) {
        return fixedMicros;
    }
    
    int16_t magHold;
    
    uint8_t stateFlags;
    uint8_t armingFlags = 0;
    
    uint32_t rcModeActivationMask;
    
    uint16_t flightModeFlags;
    
    uint16_t enableFlightMode(flightModeFlags_e mask)
    {
        return flightModeFlags |= (mask);
    }
    
    uint16_t disableFlightMode(flightModeFlags_e mask)
    {
        return flightModeFlags &= ~(mask);
    }
    
    void beeper(beeperMode_e mode) {
        UNUSED(mode);
    }
    
    bool areSticksInApModePosition(uint16_t ap_mode)
    {
        UNUSED(ap_mode);
        return true;
    }
    
    uint16_t acc_1G = 512 * 8;
    gyro_t gyro;
    int16_t gyroADC[XYZ_AXIS_COUNT];
    
    int16_t accADC[XYZ_AXIS_COUNT];
    
    void gyroUpdate(void)
    {
    }
    void updateAccelerationReadings(rollAndPitchTrims_t *rollAndPitchTrims)
    {
        UNUSED(rollAndPitchTrims);
    }
}
