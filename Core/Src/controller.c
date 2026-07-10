#include "controller.h"

#include <math.h>
#include <stddef.h>

static float Controller_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float Controller_NewDeploymentInternal(const Controller *controller,
                                              ControllerData positional_data,
                                              float dt_s)
{
    const float max_change = CONTROLLER_MAX_CHANGE_PER_SECOND * dt_s;

    /* Starts airbrake de-deployment when close to apogee. */
    if (positional_data.vertical_velocity <= CONTROLLER_VELOCITY_CUTOFF)
    {
        return controller->deployment - max_change;
    }

    const float vert_velo_2 = positional_data.vertical_velocity * positional_data.vertical_velocity;
    const float k = -(positional_data.vertical_acceleration + CONTROLLER_G) / vert_velo_2;

    float delta_h;

    if (k > 0.0f)
    {
        const float x = k * vert_velo_2 / CONTROLLER_G;
        delta_h = log1pf(x) / (k + k);
    }
    else
    {
        /* Fallback path, most likely near apogee. */
        delta_h = vert_velo_2 / (CONTROLLER_G + CONTROLLER_G);
    }

    const float predicted_apogee = positional_data.altitude_agl + delta_h;
    const float apogee_error = predicted_apogee - controller->target_apogee;

    const float desired_change = apogee_error * CONTROLLER_KP_PER_SECOND * dt_s;
    const float actual_change = Controller_ClampFloat(desired_change, -max_change, max_change);

    return controller->deployment + actual_change;
}

void Controller_Init(Controller *controller, float target_apogee_m)
{
    if (controller == NULL)
    {
        return;
    }

    controller->deployment = CONTROLLER_DEPLOYMENT_MIN;
    controller->target_apogee = target_apogee_m;
}

float Controller_GetDeployment(const Controller *controller)
{
    if (controller == NULL)
    {
        return CONTROLLER_DEPLOYMENT_MIN;
    }

    return controller->deployment;
}

float Controller_GetTargetApogee(const Controller *controller)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return controller->target_apogee;
}

void Controller_Close(Controller *controller, float dt_s)
{
    if (controller == NULL)
    {
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        return;
    }

    const float max_change = CONTROLLER_MAX_CHANGE_PER_SECOND * dt_s;
    controller->deployment = Controller_ClampFloat(controller->deployment - max_change,
                                                   CONTROLLER_DEPLOYMENT_MIN,
                                                   CONTROLLER_DEPLOYMENT_MAX);
}

float Controller_NewDeployment(Controller *controller,
                               ControllerData positional_data,
                               float dt_s)
{
    if (controller == NULL)
    {
        return CONTROLLER_DEPLOYMENT_MIN;
    }

    if (!isfinite(positional_data.altitude_agl) ||
        !isfinite(positional_data.vertical_acceleration) ||
        !isfinite(positional_data.vertical_velocity) ||
        !isfinite(dt_s) ||
        dt_s <= 0.0f)
    {
        return controller->deployment;
    }

    controller->deployment = Controller_ClampFloat(
        Controller_NewDeploymentInternal(controller, positional_data, dt_s),
        CONTROLLER_DEPLOYMENT_MIN,
        CONTROLLER_DEPLOYMENT_MAX);

    return controller->deployment;
}
