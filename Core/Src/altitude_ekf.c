#include "altitude_ekf.h"

#ifndef ALTITUDE_EKF_EPSILON
#define ALTITUDE_EKF_EPSILON (1.0e-9f)
#endif

static void AltitudeEKF_Predict(AltitudeEKF_t *ekf, float dt_s);
static void AltitudeEKF_Correct(AltitudeEKF_t *ekf, const float h[4], float z, float r);
static void Mat4Mul(float out[4][4], float a[4][4], float b[4][4]);
static void Mat4Transpose(float out[4][4], float a[4][4]);
static void Mat4Symmetrize(float a[4][4]);

void AltitudeEKF_Init(AltitudeEKF_t *ekf,
                      float initial_altitude_m,
                      float var_baro,
                      float var_imu_accel,
                      float var_process_jerk,
                      float var_process_bias)
{
    uint32_t i;
    uint32_t j;

    if (ekf == 0)
    {
        return;
    }

    /* Avoid divide-by-zero or invalid covariance growth if a caller passes 0. */
    if (var_baro <= 0.0f)         { var_baro = 1.0e-3f; }
    if (var_imu_accel <= 0.0f)    { var_imu_accel = 1.0e-3f; }
    if (var_process_jerk <= 0.0f) { var_process_jerk = 1.0f; }
    if (var_process_bias <= 0.0f) { var_process_bias = 1.0e-4f; }

    ekf->x[0] = initial_altitude_m;
    ekf->x[1] = 0.0f;
    ekf->x[2] = 0.0f;
    ekf->x[3] = 0.0f;

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            ekf->p[i][j] = 0.0f;
        }
    }

    /* Initial covariance from the Rust implementation. */
    ekf->p[0][0] = var_baro;
    ekf->p[1][1] = 100.0f;
    ekf->p[2][2] = var_imu_accel * 10.0f;
    ekf->p[3][3] = 0.25f;

    ekf->var_process_jerk = var_process_jerk;
    ekf->var_process_bias = var_process_bias;
    ekf->var_baro = var_baro;
    ekf->var_imu_accel = var_imu_accel;
}

void AltitudeEKF_Reset(AltitudeEKF_t *ekf, float altitude_m)
{
    if (ekf == 0)
    {
        return;
    }

    AltitudeEKF_Init(ekf,
                     altitude_m,
                     ekf->var_baro,
                     ekf->var_imu_accel,
                     ekf->var_process_jerk,
                     ekf->var_process_bias);
}

void AltitudeEKF_UpdateImu(AltitudeEKF_t *ekf,
                           float vertical_accel_m_s2,
                           float dt_s)
{
    static const float h_imu[4] = {0.0f, 0.0f, 1.0f, 1.0f};

    if (ekf == 0)
    {
        return;
    }

    AltitudeEKF_Predict(ekf, dt_s);
    AltitudeEKF_Correct(ekf, h_imu, vertical_accel_m_s2, ekf->var_imu_accel);
}

void AltitudeEKF_UpdateBaro(AltitudeEKF_t *ekf, float altitude_m)
{
    static const float h_baro[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    if (ekf == 0)
    {
        return;
    }

    AltitudeEKF_Correct(ekf, h_baro, altitude_m, ekf->var_baro);
}

float AltitudeEKF_GetAltitudeAGL(const AltitudeEKF_t *ekf)
{
    return (ekf != 0) ? ekf->x[0] : 0.0f;
}

float AltitudeEKF_GetVerticalVelocity(const AltitudeEKF_t *ekf)
{
    return (ekf != 0) ? ekf->x[1] : 0.0f;
}

float AltitudeEKF_GetVerticalAcceleration(const AltitudeEKF_t *ekf)
{
    return (ekf != 0) ? ekf->x[2] : 0.0f;
}

float AltitudeEKF_GetAccelBias(const AltitudeEKF_t *ekf)
{
    return (ekf != 0) ? ekf->x[3] : 0.0f;
}

static void AltitudeEKF_Predict(AltitudeEKF_t *ekf, float dt_s)
{
    float dt;
    float dt2;
    float dt3;
    float dt4;
    float dt5;
    float f[4][4];
    float ft[4][4];
    float q[4][4];
    float fp[4][4];
    float p_new[4][4];
    float x_new[4];
    uint32_t i;
    uint32_t j;

    if (dt_s <= 0.0f)
    {
        return;
    }

    dt = dt_s;
    dt2 = dt * dt;
    dt3 = dt2 * dt;
    dt4 = dt3 * dt;
    dt5 = dt4 * dt;

    /* State transition matrix F. */
    f[0][0] = 1.0f; f[0][1] = dt;   f[0][2] = 0.5f * dt2; f[0][3] = 0.0f;
    f[1][0] = 0.0f; f[1][1] = 1.0f; f[1][2] = dt;         f[1][3] = 0.0f;
    f[2][0] = 0.0f; f[2][1] = 0.0f; f[2][2] = 1.0f;       f[2][3] = 0.0f;
    f[3][0] = 0.0f; f[3][1] = 0.0f; f[3][2] = 0.0f;       f[3][3] = 1.0f;

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            q[i][j] = 0.0f;
        }
    }

    /* Wiener-process jerk noise. */
    q[0][0] += ekf->var_process_jerk * (dt5 / 20.0f);
    q[0][1] += ekf->var_process_jerk * (dt4 / 8.0f);
    q[0][2] += ekf->var_process_jerk * (dt3 / 6.0f);

    q[1][0] += ekf->var_process_jerk * (dt4 / 8.0f);
    q[1][1] += ekf->var_process_jerk * (dt3 / 3.0f);
    q[1][2] += ekf->var_process_jerk * (dt2 / 2.0f);

    q[2][0] += ekf->var_process_jerk * (dt3 / 6.0f);
    q[2][1] += ekf->var_process_jerk * (dt2 / 2.0f);
    q[2][2] += ekf->var_process_jerk * dt;

    /* Bias random-walk noise. */
    q[3][3] += ekf->var_process_bias * dt;

    /* x = F*x */
    x_new[0] = f[0][0] * ekf->x[0] + f[0][1] * ekf->x[1] + f[0][2] * ekf->x[2] + f[0][3] * ekf->x[3];
    x_new[1] = f[1][0] * ekf->x[0] + f[1][1] * ekf->x[1] + f[1][2] * ekf->x[2] + f[1][3] * ekf->x[3];
    x_new[2] = f[2][0] * ekf->x[0] + f[2][1] * ekf->x[1] + f[2][2] * ekf->x[2] + f[2][3] * ekf->x[3];
    x_new[3] = f[3][0] * ekf->x[0] + f[3][1] * ekf->x[1] + f[3][2] * ekf->x[2] + f[3][3] * ekf->x[3];

    for (i = 0U; i < 4U; i++)
    {
        ekf->x[i] = x_new[i];
    }

    /* P = F*P*F' + Q */
    Mat4Transpose(ft, f);
    Mat4Mul(fp, f, ekf->p);
    Mat4Mul(p_new, fp, ft);

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            ekf->p[i][j] = p_new[i][j] + q[i][j];
        }
    }

    Mat4Symmetrize(ekf->p);
}

static void AltitudeEKF_Correct(AltitudeEKF_t *ekf, const float h[4], float z, float r)
{
    float hx;
    float innovation;
    float ph_t[4];
    float s;
    float k[4];
    float i_kh[4][4];
    float i_kh_t[4][4];
    float temp[4][4];
    float p_new[4][4];
    uint32_t i;
    uint32_t j;

    if (r <= 0.0f)
    {
        return;
    }

    /* innovation = z - H*x */
    hx = h[0] * ekf->x[0] + h[1] * ekf->x[1] + h[2] * ekf->x[2] + h[3] * ekf->x[3];
    innovation = z - hx;

    /* ph_t = P*H' */
    for (i = 0U; i < 4U; i++)
    {
        ph_t[i] = 0.0f;
        for (j = 0U; j < 4U; j++)
        {
            ph_t[i] += ekf->p[i][j] * h[j];
        }
    }

    /* s = H*P*H' + R */
    s = r;
    for (i = 0U; i < 4U; i++)
    {
        s += h[i] * ph_t[i];
    }

    if (s <= ALTITUDE_EKF_EPSILON)
    {
        return;
    }

    /* k = P*H'/s */
    for (i = 0U; i < 4U; i++)
    {
        k[i] = ph_t[i] / s;
    }

    /* x = x + k*innovation */
    for (i = 0U; i < 4U; i++)
    {
        ekf->x[i] += k[i] * innovation;
    }

    /* i_kh = I - k*H */
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            i_kh[i][j] = ((i == j) ? 1.0f : 0.0f) - (k[i] * h[j]);
        }
    }

    /* Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K' */
    Mat4Transpose(i_kh_t, i_kh);
    Mat4Mul(temp, i_kh, ekf->p);
    Mat4Mul(p_new, temp, i_kh_t);

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            ekf->p[i][j] = p_new[i][j] + (k[i] * r * k[j]);
        }
    }

    Mat4Symmetrize(ekf->p);
}

static void Mat4Mul(float out[4][4], float a[4][4], float b[4][4])
{
    float result[4][4];
    uint32_t i;
    uint32_t j;
    uint32_t k;

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            result[i][j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            out[i][j] = result[i][j];
        }
    }
}

static void Mat4Transpose(float out[4][4], float a[4][4])
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            out[i][j] = a[j][i];
        }
    }
}

static void Mat4Symmetrize(float a[4][4])
{
    float avg;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < 4U; i++)
    {
        for (j = i + 1U; j < 4U; j++)
        {
            avg = 0.5f * (a[i][j] + a[j][i]);
            a[i][j] = avg;
            a[j][i] = avg;
        }
    }
}
