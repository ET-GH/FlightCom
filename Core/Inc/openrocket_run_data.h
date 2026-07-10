#ifndef OPENROCKET_RUN_DATA_H
#define OPENROCKET_RUN_DATA_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float time_s;
    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;
} OpenRocketSample;

typedef enum {
    OPENROCKET_EVENT_IGNITION,
    OPENROCKET_EVENT_LAUNCH,
    OPENROCKET_EVENT_LIFTOFF,
    OPENROCKET_EVENT_LAUNCHROD,
    OPENROCKET_EVENT_BURNOUT,
    OPENROCKET_EVENT_TUMBLE,
    OPENROCKET_EVENT_APOGEE,
    OPENROCKET_EVENT_RECOVERY_DEVICE_DEPLOYMENT,
    OPENROCKET_EVENT_GROUND_HIT,
    OPENROCKET_EVENT_SIMULATION_END
} OpenRocketEventType;

typedef struct {
    OpenRocketEventType type;
    float time_s;
} OpenRocketEvent;

extern const OpenRocketSample openrocket_run_samples[];
extern const size_t openrocket_run_sample_count;
extern const OpenRocketEvent openrocket_run_events[];
extern const size_t openrocket_run_event_count;

/* Returns NULL when index is outside the sample array. */
const OpenRocketSample *openrocket_run_get(size_t index);

/* Returns the sample whose timestamp is closest to time_s. */
const OpenRocketSample *openrocket_run_find_nearest(float time_s);

/*
 * Linearly interpolates all fields at time_s.
 * Values before/after the run are clamped to the first/last sample.
 * Returns false only when out_sample is NULL or the data set is empty.
 */
bool openrocket_run_interpolate(float time_s, OpenRocketSample *out_sample);

const char *openrocket_event_name(OpenRocketEventType type);

#ifdef __cplusplus
}
#endif

#endif /* OPENROCKET_RUN_DATA_H */
