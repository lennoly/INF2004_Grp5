/* Test the actual motor owner's latch, including rejected later commands. */
#include <assert.h>
#include <stdio.h>
#include "../../../app_program/subsystem2/motion.c"

static UINT flags;
static float32_t duty_left;
static float32_t duty_right;

ER tk_clr_flg(ID id, UINT mask) { (void)id; flags &= mask; return E_OK; }
ER tk_set_flg(ID id, UINT bits) { (void)id; flags |= bits; return E_OK; }
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *out, INT timeout)
{
    (void)id; (void)mode; (void)timeout; *out = flags;
    return (flags & bits) ? E_OK : E_LIMIT;
}
void encoder_read(uint32_t wheel, encoder_reading_t *out)
{
    (void)wheel; *out = (encoder_reading_t){0};
}
float32_t encoder_mm_per_tick(void) { return 5.0f; }
void motor_set(float32_t left, float32_t right)
{
    duty_left = left; duty_right = right;
}

int main(void)
{
    pid_init(&g_pid_l, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_KF, 0, 100);
    pid_init(&g_pid_r, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_KF, 0, 100);
    motion_set_velocity(180, 180); control_step();
    assert(duty_left > 0 && duty_right > 0);
    motion_emergency_stop(); control_step();
    assert(motion_stop_requested() && duty_left == 0 && duty_right == 0);
    motion_set_velocity(180, 180); control_step();
    assert(duty_left == 0 && duty_right == 0);
    motion_move_forward(100, 180); control_step();
    assert(duty_left == 0 && duty_right == 0);
    assert(motion_wait(0) == MOTION_WAIT_FAIL);
    motion_clear_stop(); control_step();
    assert(duty_left == 0 && duty_right == 0);
    motion_set_velocity(180, 180); control_step();
    assert(duty_left > 0 && duty_right > 0);
    puts("Motor owner: STOP latch, rejected commands and explicit restart PASS");
    return 0;
}
