#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * Airbrake controller constants.
 *
 * Units:
 * - altitude_agl: meters
 * - vertical_acceleration: m/s^2
 * - vertical_velocity: m/s
 * - dt_s: seconds
 * - target_apogee: meters
 */

#ifndef CONTROLLER_G
#define CONTROLLER_G 9.80665f
#endif

#define CONTROLLER_VELOCITY_CUTOFF       5.0f
#define CONTROLLER_MAX_CHANGE_PER_SECOND 1.0f
#define CONTROLLER_DECAY_RANGE           7.5f
#define CONTROLLER_KP_PER_SECOND         (CONTROLLER_MAX_CHANGE_PER_SECOND / CONTROLLER_DECAY_RANGE)

#define CONTROLLER_DEPLOYMENT_MIN        0.0f
#define CONTROLLER_DEPLOYMENT_MAX        1.0f

/*
 * Positional data that will come from ekf
 */
typedef struct
{
    float altitude_agl;
    float vertical_acceleration;
    float vertical_velocity;
} ControllerData;

/*
 * Airbrake controller state.
 * deployment is always intended to stay between 0.0f and 1.0f.
 */
typedef struct
{
    float deployment;
    float target_apogee;
} Controller;

/* Initialize the controller with a target apogee in meters. */
void Controller_Init(Controller *controller, float target_apogee_m);

/* Get the current deployment value, from 0.0f closed to 1.0f fully deployed. */
float Controller_GetDeployment(const Controller *controller);

/* Get the target apogee in meters. */
float Controller_GetTargetApogee(const Controller *controller);

/* Move the airbrakes toward closed by the maximum allowed rate. */
void Controller_Close(Controller *controller, float dt_s);

/*
 * Calculate, store, and return the updated airbrake deployment.
 * positional_data and dt_s should contain finite metric values.
 */
float Controller_NewDeployment(Controller *controller,
                               ControllerData positional_data,
                               float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
