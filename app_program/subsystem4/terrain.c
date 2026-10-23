/** @file terrain.c
 *
 * @brief Buddy 4 - hump tracker and motion classifier (see terrain.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include "car_config.h"
#include "car_math.h"
#include "terrain.h"

#define LEVEL_CONFIRM_MM   (30.0f)   /* level travel that ends a hump    */
#define MAX_HUMP_LENGTH_MM (1500.0f) /* longer = ramp or drift, abandon  */
#define RETURN_FRACTION    (0.35f)   /* must come back below 35 % peak   */
#define STATIONARY_MM_S    (15.0f)   /* slower than this = standing still */
#define PREROLL_MIN_DEG    (1.0f)    /* consistent tilt worth tracking   */

/* Names indexed by terrain_event_t. */
static char const * const g_event_names[] = {
    "STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING",
    "TURNING",    "CLIMBING", "DESCENDING",   "IMPACT"};

static void hump_watch(terrain_hump_t * p_hump, float32_t pitch_deg,
                       float32_t roll_deg, float32_t delta_h, float32_t ds_mm);
static void hump_track(terrain_hump_t * p_hump, bool b_level,
                       float32_t delta_h, float32_t ds_mm);

/*!
 * @brief Clear a hump tracker (also clears the run totals).
 *
 * @param[out] p_hump Tracker to initialise.
 */
void
terrain_hump_init (terrain_hump_t * p_hump)
{
    p_hump->state       = HUMP_FLAT;
    p_hump->height_mm   = 0.0f;
    p_hump->peak_mm     = 0.0f;
    p_hump->max_peak_mm = 0.0f;
    p_hump->active_mm   = 0.0f;
    p_hump->level_mm    = 0.0f;
    p_hump->preroll_mm  = 0.0f;
    p_hump->b_one_wheel = false;
    p_hump->roll_mm     = 0.0f;
    p_hump->humps       = 0u;
}

/*!
 * @brief Feed one sample to the hump tracker.
 *
 * A non-finite sample (Rule 5.4.b.v) is ignored.  A hump under both
 * wheels is integrated from the pitch; once the roll passes
 * HUMP_ROLL_ON_DEG the hump is under one wheel, and its height is the
 * wheel's lift, HUMP_ROLL_LEVER_MM * tan(|roll|), until the car is level.
 *
 * @param[in,out] p_hump    Tracker.
 * @param[in]     pitch_deg Pitch, nose-up positive.
 * @param[in]     roll_deg  Roll, either side.
 * @param[in]     ds_mm     Signed distance since the previous sample.
 */
void
terrain_hump_update (terrain_hump_t * p_hump, float32_t pitch_deg,
                     float32_t roll_deg, float32_t ds_mm)
{
    float32_t roll_rad = car_math_abs(roll_deg) * CAR_MATH_RAD_PER_DEG;
    float32_t delta_h = ds_mm * car_math_sin(pitch_deg * CAR_MATH_RAD_PER_DEG);
    float32_t wheel_h  = (HUMP_ROLL_LEVER_MM * car_math_sin(roll_rad))
                        / car_math_cos(roll_rad);
    bool      b_valid  = (car_math_is_finite(delta_h))
                       && (car_math_is_finite(wheel_h));
    bool      b_level  = false;

    if (b_valid && (HUMP_FLAT == p_hump->state))
    {
        hump_watch(p_hump, pitch_deg, roll_deg, delta_h, ds_mm);
    }
    else if (b_valid && (!p_hump->b_one_wheel)
             && (car_math_abs(roll_deg) > HUMP_ROLL_ON_DEG))
    {
        /* A pitch hump that turns out to be under one wheel: the pitch
           height so far is wrong, so restart from the roll. */
        p_hump->b_one_wheel = true;
        p_hump->height_mm   = 0.0f;
        p_hump->peak_mm     = 0.0f;
        p_hump->roll_mm     = 0.0f;
    }
    else
    {
        /* Not valid, or the hump keeps its kind. */
    }

    /* A hump that has just started is tracked from this sample on. */
    if (b_valid && (HUMP_FLAT != p_hump->state))
    {
        if (p_hump->b_one_wheel)
        {
            b_level = (car_math_abs(roll_deg) < HUMP_ROLL_OFF_DEG);
            /* Wheels still: hold the height, so lifting the car by hand
               with a hump open does not raise the peak. */
            delta_h = (car_math_abs(ds_mm) > 0.0f)
                          ? (wheel_h - p_hump->height_mm)
                          : 0.0f;
            p_hump->roll_mm += (car_math_abs(roll_deg) > HUMP_ROLL_ON_DEG)
                                   ? car_math_abs(ds_mm)
                                   : 0.0f;
        }
        else
        {
            b_level = (car_math_abs(pitch_deg) < HUMP_PITCH_OFF_DEG);
        }

        hump_track(p_hump, b_level, delta_h, ds_mm);
    }
}

/*!
 * @brief Classify the car's motion; the highest-priority event wins
 *        (impact > hump > turn > acceleration > speed).  A hump is only
 *        reported while the car moves: one left open by braking, or by
 *        stopping on a hump, reads STATIONARY until the car rolls on.
 *
 * @param[in] speed_mm_s    Forward speed.
 * @param[in] accel_mm_s2   Forward acceleration.
 * @param[in] turn_rate_dps Turn rate both the encoders and compass agree on.
 * @param[in] hump          Hump tracker state.
 * @param[in] b_impact      true if an impact was detected.
 *
 * @return The motion event.
 */
terrain_event_t
terrain_classify (float32_t speed_mm_s, float32_t accel_mm_s2,
                  float32_t turn_rate_dps, terrain_hump_state_t hump,
                  bool b_impact)
{
    terrain_event_t event    = EVT_CRUISING;
    bool            b_moving = (car_math_abs(speed_mm_s) >= STATIONARY_MM_S);

    if (b_impact)
    {
        event = EVT_IMPACT;
    }
    else if (b_moving && (HUMP_CLIMBING == hump))
    {
        event = EVT_CLIMBING;
    }
    else if (b_moving && (HUMP_DESCENDING == hump))
    {
        event = EVT_DESCENDING;
    }
    else if (car_math_abs(turn_rate_dps) > TURN_RATE_EVENT_DPS)
    {
        event = EVT_TURNING;
    }
    else if (accel_mm_s2 > ACCEL_EVENT_MM_S2)
    {
        event = EVT_ACCELERATING;
    }
    else if (accel_mm_s2 < -ACCEL_EVENT_MM_S2)
    {
        event = EVT_DECELERATING;
    }
    else if (car_math_abs(speed_mm_s) < STATIONARY_MM_S)
    {
        event = EVT_STATIONARY;
    }
    else
    {
        event = EVT_CRUISING;
    }

    return (event);
}

/*!
 * @brief Printable name of a motion event.
 *
 * @param[in] event Motion event.
 *
 * @return Upper-case name, or "?" for an out-of-range value.
 */
char const *
terrain_event_name (terrain_event_t event)
{
    /* Cast: enumeration constants are non-negative, so the conversion to
       an unsigned index keeps their value; the range is checked below. */
    uint32_t     index  = (uint32_t) event;
    char const * p_name = "?";

    if (index < (sizeof(g_event_names) / sizeof(g_event_names[0])))
    {
        p_name = g_event_names[index];
    }

    return (p_name);
}

/*!
 * @brief Direction the car is driving, for the impact detector.
 *
 * @param[in] pwm_pct    Mean motor duty, signed (forward positive).
 * @param[in] speed_mm_s Mean wheel speed.
 * @param[in] last_sign  Result of the previous sample.
 *
 * @return +1 forward, -1 reversing, 0 parked.  With the motors off but the
 *         wheels still turning (coasting, braking) the last direction holds.
 */
float32_t
terrain_travel_sign (float32_t pwm_pct, float32_t speed_mm_s,
                     float32_t last_sign)
{
    float32_t sign = 0.0f;

    if (pwm_pct > IMPACT_DRIVE_PCT)
    {
        sign = 1.0f;
    }
    else if (pwm_pct < -IMPACT_DRIVE_PCT)
    {
        sign = -1.0f;
    }
    else if (car_math_abs(speed_mm_s) >= STATIONARY_MM_S)
    {
        sign = last_sign;
    }
    else
    {
        sign = 0.0f;
    }

    return (sign);
}

/*!
 * @brief The part of a sample-to-sample acceleration change that can be a
 *        collision.
 *
 * Parked, any jolt counts (knocked by something).  Driving, only the jolt
 * against the direction of travel: a crash slows the car, while its own
 * motors starting push it the way it is going (forward when starting,
 * backward when reversing), which by size alone looks like a crash.
 *
 * @param[in] delta_fwd_mg  Change along the car's forward axis.
 * @param[in] delta_side_mg Change sideways.
 * @param[in] delta_up_mg   Change vertically.
 * @param[in] travel_sign   From terrain_travel_sign().
 *
 * @return Jerk to compare with IMPACT_JERK_MG, in mg (never negative).
 */
float32_t
terrain_impact_jerk (float32_t delta_fwd_mg, float32_t delta_side_mg,
                     float32_t delta_up_mg, float32_t travel_sign)
{
    float32_t jerk = 0.0f;

    if (car_math_abs(travel_sign) > 0.0f)
    {
        jerk = -travel_sign * delta_fwd_mg;
        jerk = (jerk > 0.0f) ? jerk : 0.0f;
    }
    else
    {
        jerk = car_math_sqrt((delta_fwd_mg * delta_fwd_mg)
                             + (delta_side_mg * delta_side_mg)
                             + (delta_up_mg * delta_up_mg));
    }

    return (jerk);
}

/*!
 * @brief Time left in which impacts are ignored, updated once per IMU step.
 *
 * Starting, stopping, braking and reversing jump the car's own motor duty
 * and jolt it, braking even against the direction of travel.  A crash
 * comes while the duty is steady, so it still counts.
 *
 * @param[in] blank_ms        Time left from the previous step.
 * @param[in] duty_change_pct Change of the mean motor duty since then.
 *
 * @return IMPACT_BLANK_MS after a jump of more than IMPACT_DUTY_STEP_PCT,
 *         else the time left, one IMU step less (never below 0).
 */
uint32_t
terrain_impact_blank (uint32_t blank_ms, float32_t duty_change_pct)
{
    uint32_t left = 0u;

    if (car_math_abs(duty_change_pct) > IMPACT_DUTY_STEP_PCT)
    {
        left = IMPACT_BLANK_MS;
    }
    else
    {
        left = (blank_ms > IMU_PERIOD_MS) ? (blank_ms - IMU_PERIOD_MS) : 0u;
    }

    return (left);
}

/*!
 * @brief While flat: accumulate pre-roll height and start a hump once the
 *        pitch passes HUMP_PITCH_ON_DEG, or the roll HUMP_ROLL_ON_DEG,
 *        while the wheels are turning.
 *
 * Pre-roll keeps the height gained while climbing gently, so the part of
 * the ramp below the trigger angle is not lost.  A tilt without wheel
 * travel (ds is 0: the car lifted, carried or tilted by hand) is not a
 * hump, so it never opens one that only rolling could close.
 *
 * @param[in,out] p_hump    Tracker (state HUMP_FLAT).
 * @param[in]     pitch_deg Pitch, nose-up positive.
 * @param[in]     roll_deg  Roll, either side.
 * @param[in]     delta_h   Height change of this sample.
 * @param[in]     ds_mm     Signed distance of this sample.
 */
static void
hump_watch (terrain_hump_t * p_hump, float32_t pitch_deg, float32_t roll_deg,
            float32_t delta_h, float32_t ds_mm)
{
    bool b_one_wheel = (car_math_abs(roll_deg) > HUMP_ROLL_ON_DEG);

    if ((car_math_abs(pitch_deg) > PREROLL_MIN_DEG)
        && ((delta_h * p_hump->preroll_mm) >= 0.0f))
    {
        p_hump->preroll_mm += delta_h;
    }
    else
    {
        p_hump->preroll_mm = 0.0f;
    }

    if ((b_one_wheel || (car_math_abs(pitch_deg) > HUMP_PITCH_ON_DEG))
        && (car_math_abs(ds_mm) > 0.0f))
    {
        /* Start a new hump.  delta_h is added again by hump_track(); a
           one-wheel hump starts at 0 and reads its height from the roll. */
        p_hump->state       = HUMP_CLIMBING;
        p_hump->b_one_wheel = b_one_wheel;
        p_hump->roll_mm     = 0.0f;
        p_hump->height_mm   = ((p_hump->preroll_mm > 0.0f) && (!b_one_wheel))
                                  ? (p_hump->preroll_mm - delta_h)
                                  : 0.0f;
        p_hump->peak_mm    = 0.0f;
        p_hump->active_mm  = 0.0f;
        p_hump->level_mm   = 0.0f;
        p_hump->preroll_mm = 0.0f;
    }
}

/*!
 * @brief While on a hump: integrate the height and decide when it ends.
 *
 * @param[in,out] p_hump  Tracker (state not HUMP_FLAT).
 * @param[in]     b_level true if the car reads level on this sample.
 * @param[in]     delta_h Height change of this sample.
 * @param[in]     ds_mm   Signed distance of this sample.
 */
static void
hump_track (terrain_hump_t * p_hump, bool b_level, float32_t delta_h,
            float32_t ds_mm)
{
    p_hump->height_mm += delta_h;
    p_hump->active_mm += car_math_abs(ds_mm);
    p_hump->peak_mm = (p_hump->height_mm > p_hump->peak_mm) ? p_hump->height_mm
                                                            : p_hump->peak_mm;

    if (p_hump->height_mm < (p_hump->peak_mm - HUMP_STATE_HYST_MM))
    {
        p_hump->state = HUMP_DESCENDING;
    }
    else if (p_hump->height_mm >= p_hump->peak_mm)
    {
        p_hump->state = HUMP_CLIMBING;
    }
    else
    {
        /* Near the peak (noise, or stopped): keep the state. */
    }

    p_hump->level_mm =
        b_level ? (p_hump->level_mm + car_math_abs(ds_mm)) : 0.0f;

    if ((p_hump->level_mm >= LEVEL_CONFIRM_MM)
        && (p_hump->height_mm <= (RETURN_FRACTION * p_hump->peak_mm)))
    {
        /* Level again and back down: the hump is over (and counts if it
           was high enough; a one-wheel hump also if the roll lasted, so a
           jolt does not count). */
        if ((p_hump->peak_mm >= HUMP_MIN_HEIGHT_MM)
            && ((!p_hump->b_one_wheel)
                || (p_hump->roll_mm >= HUMP_ROLL_MIN_MM)))
        {
            p_hump->humps++;
            p_hump->max_peak_mm = (p_hump->peak_mm > p_hump->max_peak_mm)
                                      ? p_hump->peak_mm
                                      : p_hump->max_peak_mm;
        }

        p_hump->state     = HUMP_FLAT;
        p_hump->height_mm = 0.0f;
    }
    else if (p_hump->active_mm > MAX_HUMP_LENGTH_MM)
    {
        /* Too long for a hump: a ramp or drift, so give up. */
        p_hump->state     = HUMP_FLAT;
        p_hump->height_mm = 0.0f;
    }
    else
    {
        /* Still on the hump. */
    }
}

/*** end of file ***/
