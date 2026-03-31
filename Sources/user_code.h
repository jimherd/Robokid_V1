
#ifndef __user_code_H
#define __user_code_H

#include "global.h"

/* MODULE user_code */
 
//----------------------------------------------------------------------------
// function prototypes
//
void user_init(void);
void DelayMs(uint16_t count);
void disable_wheel_count(void);
void enable_wheel_count(void);
uint8_t  update_PID(int8_t error, int8_t old_error); 
void set_vehicle_state(void);
void set_motor_speed_dir(motor_t motor_unit, int int_pwm_width);
void set_motor(motor_t unit, motor_state_t state, uint8_t pwm_width);
void vehicle_stop(void);
int16_t abs16(int16_t  value);
void self_test(void);
void run_bot(void);
void check_for_power_on_test(void);
void set_log_pin(void);
void clear_log_pin(void);

//
// Definition of structure of data in the FLASH constant area
//
typedef volatile struct {
    uint8_t     GUARD_BYTE; 
    uint8_t     LEFT_WHEEL_THRESHOLD, RIGHT_WHEEL_THRESHOLD;
} FLASH_data_t;

#endif

