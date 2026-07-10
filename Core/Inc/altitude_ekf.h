#ifndef ALTITUDE_EKF_H
#define ALTITUDE_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Altitude / velocity / acceleration / accelerometer-bias Kalman filter.
 *
 * State vector:
 *   x[0] = h = altitude above ground level, meters
 *   x[1] = v = vertical velocity, m/s, positive upward
 *   x[2] = a = true vertical acceleration, m/s^2, gravity removed, positive upward
 *   x[3] = b = accelerometer vertical bias, m/s^2, positive upward
 *
 * Measurement model:
 *   IMU/baro-AHRS vertical acceleration: z = a + b, H = [0 0 1 1]
 *   Barometer altitude:                  z = h,     H = [1 0 0 0]
 *
 * This file is dependency-free and is intended for STM32CubeIDE projects.
 */

typedef struct
{
    float x[4];       /* State estimate: [h, v, a, b] */
    float p[4][4];    /* Error covariance matrix */

    float var_process_jerk;  /* Process noise: jerk variance, m^2/s^5 */
    float var_process_bias;  /* Process noise: bias random walk variance, m^2/s^5 */
    float var_baro;          /* Barometer altitude measurement variance, m^2 */
    float var_imu_accel;     /* IMU vertical acceleration measurement variance, m^2/s^4 */
} AltitudeEKF_t;

void AltitudeEKF_Init(AltitudeEKF_t *ekf,
                      float initial_altitude_m,
                      float var_baro,
                      float var_imu_accel,
                      float var_process_jerk,
                      float var_process_bias);

void AltitudeEKF_Reset(AltitudeEKF_t *ekf, float altitude_m);

void AltitudeEKF_UpdateImu(AltitudeEKF_t *ekf,
                           float vertical_accel_m_s2,
                           float dt_s);

void AltitudeEKF_UpdateBaro(AltitudeEKF_t *ekf,
                            float altitude_m);

float AltitudeEKF_GetAltitudeAGL(const AltitudeEKF_t *ekf);
float AltitudeEKF_GetVerticalVelocity(const AltitudeEKF_t *ekf);
float AltitudeEKF_GetVerticalAcceleration(const AltitudeEKF_t *ekf);
float AltitudeEKF_GetAccelBias(const AltitudeEKF_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_EKF_H */
