//----------------------------------------------------------------------------
//                  Robokid
//----------------------------------------------------------------------------
// follow.c : code to run follow modes
// ========
//
// Description
//
// Author                Date          Comment
//----------------------------------------------------------------------------
// Jim Herd          14/02/09      
//----------------------------------------------------------------------------

#include "global.h"

#include <stdlib.h>

//----------------------------------------------------------------------------
// run_follow_mode : run one of a set of follow activities
// ===============
//
// Notes
//
//      Active switches are 
//          switch A = go/stop button
//          switch C = exit to main slection level
//          switch D = step through selections
//
uint8_t run_follow_mode(void) {

uint8_t     activity;

    F_MODE_LEDS;
    activity = FIRST_FOLLOW_MODE;
    show_dual_chars('F', ('0'+ activity), (A_TO_FLASH | 10));
//
// main loop
//       
    FOREVER {
//
//  check and act on the switches
//
        if (switch_D == PRESSED) {
            WAIT_SWITCH_RELEASED(switch_D);
            SOUND_NEXT_SELECTION;
            activity++;
            if (activity > LAST_FOLLOW_MODE) {
                activity = FIRST_FOLLOW_MODE;
            }
            show_dual_chars('F', ('0'+ activity), (A_TO_FLASH | 10));
        }
        if (switch_C == PRESSED) {            //  back to main 
            SOUND_EXIT_SELECTION;
            WAIT_SWITCH_RELEASED(switch_C);   
            return 0;
        }        
        if (switch_A == PRESSED) {              
            set_LED(LED_A, FLASH_ON); 
            set_LED(LED_D, FLASH_ON);
            WAIT_SWITCH_RELEASED(switch_A); 
        } else {
            continue;
        }
//
// run a follow activity
// 
        show_dual_chars('F', ('0'+ activity), 0); 
        push_LED_display();      
        switch (activity) {
                case LINE_FOLLOW_MODE :                          // in progress
                    run_follow_line_mode();
                    break;
                case LIGHT_FOLLOW_MODE :                         // in progress
                    run_follow_light_mode();
                    break;
                default :
                    break;
        }
        pop_LED_display();
        show_dual_chars('F', ('0'+ activity), (A_TO_FLASH | 10));
        F_MODE_LEDS;            
    } 
}

//----------------------------------------------------------------------------
// run_follow_line_mode : run vehicle mode with 2 line follow sensors
// ====================
//
// Description
//      Vehicle uses the two floor sensors that straddle a 19mm black tape line.
//      These sensors are measured and converted to digital values.  The sensors give a value between 0
//      and 255
//                0 = BLACK (complete IR absorption)   
//                1 = WHITE (complete IR reflection)
//      The values never reach these extremes due to non-ideal absoption/reflection
//      characteristics of floor and tape materials.
//
// Notes
//
//      Active switches are 
//          switch A = go/stop button
//          switch B = change setting by reading POT_1, POT_2 and POT_3
//          switch C = exit mode
//
//      Active pots
//          POT_1 : speed setting   0->60%
//          POT_2 : right wheel slow down factor 0->100% of speed
//          POT_3 : sample rate  10mS->500mS (units of 10mS)
//
//      PID parameters include sample time to save some arithmetic operations
//      Also, the values are scaled by TIME_SCALE_FACTOR to allow int arithmetic
//      Thus the gain values are larger than would be expected
//      


uint8_t run_follow_line_mode(void) {

uint8_t          i, line_L, line_R, L_threshold, R_threshold, LR_value, last_LR_value;
uint16_t         counter, WW_count, time16;   // count of consecutive White/White readings
line_follow_state_t     state;
vehicle_state_t  drive_mode;
//      variables for line follow PID loop
int16_t      last_error, integral, error;
int16_t       p_term, i_term, d_term, PID_correction, left_pwm, save_left_pwm, right_pwm, save_right_pwm;

    state = S_INIT;
    left_pwm = DEFAULT_LINE_FOLLOW_SPEED;
    right_pwm = DEFAULT_LINE_FOLLOW_SPEED;
    WW_count = 0;
    last_LR_value = LR_WW;   // assume robot to placed over the black tape line

    drive_mode = V_FORWARD;
    error = 0;
    last_error = 0;
    integral   = 0;
    
    set_LED(LED_A, FLASH_ON);
    set_LED(LED_B, FLASH_ON);
    set_LED(LED_C, FLASH_ON);
    clr_LED(LED_D);  
//
// main loop
//       
    FOREVER {
    	SET_LOG_PIN;
//
//  Start/stop mode
//
		if (switch_A == PRESSED) {
			if (state == S_INIT) {
				state = S_PID;
				WW_count = 0;
			} else {
				state = S_INIT;
				vehicle_stop();
			}
			WAIT_SWITCH_RELEASED(switch_A); 
			continue;
		}
//
// read pots to set system characteristics
//
        if (switch_B == PRESSED) {
            SOUND_READ_POTS;  
//            ad_value = get_adc(POT_3);
//            temp16 =  ((ad_value >> 3) & 0x1F);                   // convert to 0->31
//            temp16 = (temp16 * 20) / 0x1F;                        // convert to 0->50
//            sample_time = (uint8_t)temp16 * 10;                   // convert to 0->500mS
//            if (sample_time == 0) {              // ensure that sample time is not 0
//                sample_time = 10;
//            }   
            WAIT_SWITCH_RELEASED(switch_B);
        }
//
//  check for exit
//
        if (switch_C == PRESSED) {            //  exit mode
            vehicle_stop();
            SOUND_EXIT_SELECTION;
            WAIT_SWITCH_RELEASED(switch_C);
            return 0;
        } 
//
// Line following using analogue values from IR line sensors
// Generate analogue and digital values	
//
		line_L = get_adc(LINE_SENSOR_L);
		L_threshold = (line_L > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
		
		line_R = get_adc(LINE_SENSOR_R);
		R_threshold = (line_R > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
		
		last_LR_value = LR_value;
		LR_value = (L_threshold << 1) | R_threshold;
	
		error = (line_L - line_R); 
//
// Detect loss of line during tracking   PROBLEM!
// 
/*
		if (state == S_PID) {
			if (LR_value == LR_WW) {
				if ((last_LR_value == LR_BW) && (drive_mode == V_FORWARD_TURN_LEFT)) {
					state = S_FIND_LINE_FROM_RIGHT;
				} else {
					if ((last_LR_value == LR_WB) && (drive_mode == V_FORWARD_TURN_RIGHT)) {
						state = S_FIND_LINE_FROM_LEFT;
					}
				}	
			}
		}
*/

//
// Detect vehicle lost condition
// Keep coint of number of consecutive WHITE/WHITE sensor readings.
// If the number is greater than a set threshold then either
//	1. the robot is running so straight that it does not touch the black tape (unlikely)
//  2. the robot has lost the tape and is in no-mans land!
//
		if (state == S_PID ) {
			if ((line_L < LINE_NEAR_WHITE) && (line_R < LINE_NEAR_WHITE)) {
				WW_count++;
				if (WW_count > LINE_LOST_THRESHOLD) {
					state = S_FIND_LINE_FROM_LOST;
				}
			} else {	
				WW_count = 0;
			}
		}
//
// Run state machine
//

		switch (state) {
		
			case S_INIT :
				SET_MOTORS(0, 0);
				break;
					
			case S_FIND_LINE_FROM_LOST :
		// start spiral search pattern
				SET_MOTORS(60, 0);
				for (counter = 0 ; counter < 30 ; counter++) {
					SET_MOTORS(30, counter);
					line_L = get_adc(LINE_SENSOR_L);
					line_R = get_adc(LINE_SENSOR_R);
					if ((line_L > LINE_NEAR_BLACK) || (line_R > LINE_NEAR_BLACK)) {
						SET_MOTORS(0,0);
						state = S_PID;
						break;
					}
					if (state == S_PID) {
						break;
					} else {
						DelayMs(500);
					}	
				}
				break;
				
		// Drive forward until line detected or timeout occurs
				CLR_TIMER16;
				SET_MOTORS(DEFAULT_LINE_FOLLOW_SPEED, DEFAULT_LINE_FOLLOW_SPEED);  // move forward
				FOREVER {
					line_L = get_adc(LINE_SENSOR_L);
					line_R = get_adc(LINE_SENSOR_R);
					if ((line_L > LINE_NEAR_BLACK) || (line_R > LINE_NEAR_BLACK)) {
						break;    // exit loop when line is detected
					}
					GET_TIMER16(time16);
					if (time16 >= TIMEOUT_1 ) { // exit if no line detected after a number of seconds
						SET_MOTORS(0, 0);
						state = S_INIT;
						sys_error = TIME_OUT;
						break;
					}
				}
		//  Implement line hunt from LEFT
				if (line_R > LINE_NEAR_BLACK) { 
			// run multiple hunt sequeces coming from the LEFT
					for (i=0 ; i < FIND_LINE_LOOP_COUNT ; i++) {
				// arc LEFT
						CLR_TIMER16;
						SET_MOTORS(0, SLOW);     // arc LEFT
						FOREVER {
							line_R = get_adc(LINE_SENSOR_R);
							if (line_R < LINE_NEAR_WHITE) {
								break;    // exit loop when RIGHT sensor has moved away from line
							}
							GET_TIMER16(time16);
							if( time16 >= TIMEOUT_2) {
								SET_MOTORS(0, 0);
								state = S_INIT;
								sys_error = TIME_OUT;
								break;
							}									}
				// Slow forward
						CLR_TIMER16;
						SET_MOTORS(SLOW, SLOW);   // slow forward
						FOREVER {
							line_R = get_adc(LINE_SENSOR_R);
							if (line_R > LINE_NEAR_BLACK) {
								break;    // exit loop when RIGHT sensor has detected line
							}
							GET_TIMER16(time16);
							if( time16 >= TIMEOUT_2) {
								SET_MOTORS(0, 0);
								state = S_INIT;
								sys_error = TIME_OUT;
								break;
							}	
						}
					}
				}
		//  Implement line hunt from RIGHT				
				if (line_L > LINE_NEAR_BLACK) {   // comming from the RIGHT
			// run multiple hunt sequeces coming from the RIGHT
					for (i=0 ; i < FIND_LINE_LOOP_COUNT ; i++) {
				// arc RIGHT
						CLR_TIMER16;
						SET_MOTORS(SLOW, 0);     // arc RIGHT
						FOREVER {
							line_L = get_adc(LINE_SENSOR_L);
							if (line_L < LINE_NEAR_WHITE) {
								break;    // exit loop when RIGHT sensor has moved away from line
							}
							GET_TIMER16(time16);
							if( time16 >= TIMEOUT_2) {
								SET_MOTORS(0, 0);
								state = S_INIT;
								sys_error = TIME_OUT;
								break;
							}		
						}
				// Slow forward
						CLR_TIMER16;
						SET_MOTORS(SLOW, SLOW);   // slow forward
						FOREVER {
							line_L = get_adc(LINE_SENSOR_L);
							if (line_L > LINE_NEAR_BLACK) {
								break;    // exit loop when RIGHT sensor has detected line
							}
							GET_TIMER16(time16);
							if( time16 >= TIMEOUT_2) {
								SET_MOTORS(0, 0);
								state = S_INIT;
								sys_error = TIME_OUT;
								break;
							}	
						}
					}
				}
				state = S_PID;  // Return to PID mode
				break;
					
			case S_FIND_LINE_FROM_LEFT :
				DelayMs(CLEAR_LINE_TIME_MS);
				save_left_pwm = left_pwm;
				save_right_pwm = right_pwm;
				SET_MOTORS(+25, -25);
//				set_motor_speed_dir(LEFT_MOTOR,  +25);   // spin RIGHT to find line
//				set_motor_speed_dir(RIGHT_MOTOR, -25);
				counter = 0;
				for (counter = 0 ; counter < SPIN_TIMEOUT ; counter++) {
					line_R = get_adc(LINE_SENSOR_R);
					R_threshold = (line_R > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
					if (R_threshold == BLACK) {
						break;    // exit loop when RIGHT sensor has detected line
					}
				}
				if (counter >= SPIN_TIMEOUT) {
					state = S_FIND_LINE_FROM_LOST;
					break;  // go to line lost routine
				} 
				for (counter = 0 ; counter < SPIN_TIMEOUT ; counter++) {
					line_L = get_adc(LINE_SENSOR_L);
					L_threshold = (line_L > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
					if (L_threshold == BLACK) {
						break;    // exit loop when RIGHT sensor has detected line
					}
				}
				if (counter >= SPIN_TIMEOUT) {
					state = S_FIND_LINE_FROM_LOST;
					break;  // go to line lost routine
				} 
//				set_motor_speed_dir(LEFT_MOTOR,  save_left_pwm);   // reset to original
//				set_motor_speed_dir(RIGHT_MOTOR, save_right_pwm);  // pwm values
				SET_MOTORS(save_left_pwm, save_right_pwm);
				state = S_PID;
				DelayMs(LINE_FOLLOW_SAMPLE_TIME);
				break;
					
			case S_FIND_LINE_FROM_RIGHT :
				DelayMs(CLEAR_LINE_TIME_MS);
				save_left_pwm = left_pwm;
				save_right_pwm = right_pwm;
//				set_motor_speed_dir(LEFT_MOTOR,  -25);   // turn LEFT to find line
//				set_motor_speed_dir(RIGHT_MOTOR, +25);
				SET_MOTORS(-25, +25);
				for (counter = 0 ; counter > SPIN_TIMEOUT ; counter++) {
					line_L = get_adc(LINE_SENSOR_L);
					L_threshold = (line_L > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
					if (L_threshold == BLACK) {
						break;    // exit loop when RIGHT sensor has detected line
					}
					DelayMs(CLEAR_LINE_TIME_MS);
				}
				if (counter >= SPIN_TIMEOUT) {
					state = S_FIND_LINE_FROM_LOST;
					break;  // go to line lost routine
				} 
				for (counter = 0 ; counter > SPIN_TIMEOUT ; counter++) {
					line_R = get_adc(LINE_SENSOR_R);
					R_threshold = (line_R > LINE_VALUE_THRESHOLD) ? BLACK : WHITE;
					if (R_threshold == BLACK) {
						break;    // exit loop when RIGHT sensor has detected line
					}
					DelayMs(CLEAR_LINE_TIME_MS);
				}
				if (counter >= SPIN_TIMEOUT) {
					state = S_FIND_LINE_FROM_LOST;
					break;  // go to line lost routine
				} 
//				set_motor_speed_dir(LEFT_MOTOR,  save_left_pwm);   // reset to original
//				set_motor_speed_dir(RIGHT_MOTOR, save_right_pwm);  // pwm values
				SET_MOTORS(save_left_pwm, save_right_pwm);
				state = S_PID;
				DelayMs(LINE_FOLLOW_SAMPLE_TIME);
				break;
					
			case S_FOUND_L : // line found from LEFT 
				if (line_L > LINE_NEAR_BLACK) {
					state = S_PID;  // restart PID mode
				}
				break;
					
			case S_FOUND_R : // line found from RIGHT
				if (line_R > LINE_NEAR_BLACK) {
					state = S_PID;  // restart PID mode
				}
				break;
					
			case S_PID : //   Execute PID control loop
				p_term = error * LINE_KP_INT;
			//
				integral += error;
				if (integral > LINE_INTEGRAL_WINDUP_LIMIT) {
					integral = LINE_INTEGRAL_WINDUP_LIMIT;
				}
				if (integral < -LINE_INTEGRAL_WINDUP_LIMIT) {
					integral = -LINE_INTEGRAL_WINDUP_LIMIT;
				}
				i_term = integral * LINE_KI_INT; //   DELTA_T
			//
				d_term = ((error - last_error) * LINE_KD_INT);
			//
				PID_correction = (p_term + i_term + d_term) / TIME_SCALE_FACTOR;
			//
				last_error = error;	
			//
				left_pwm = DEFAULT_LINE_FOLLOW_SPEED - PID_correction;		
				right_pwm = DEFAULT_LINE_FOLLOW_SPEED + PID_correction;
			//
				if (left_pwm > 100) {
					left_pwm = 100;
				} else {
					if (left_pwm < -100) {
						left_pwm = -100;
					}
				}
				if (right_pwm > 100) {
					right_pwm = 100;
				} else {
					if (right_pwm < -100) {
						right_pwm = -100;
					}
				}
			//
//				set_motor_speed_dir(LEFT_MOTOR, (uint8_t)left_pwm);
//				set_motor_speed_dir(RIGHT_MOTOR, (uint8_t)right_pwm);
				SET_MOTORS(left_pwm, right_pwm);
			//
				if (abs((uint8_t)PID_correction) < PWM_DEADBAND ) {
					drive_mode = V_FORWARD;
				} else {
					if (left_pwm > right_pwm) {
						drive_mode = V_FORWARD_TURN_RIGHT;
					} else {
						drive_mode = V_FORWARD_TURN_LEFT;
					}
				}
			//
				CLEAR_LOG_PIN;
				DelayMs(LINE_FOLLOW_SAMPLE_TIME);   // only in S_PID state
				break;
				
			default :
				break;
				
			}  // end of switch construct
			CLEAR_LOG_PIN;
		}  // end of FOREVER loop
	}  // end of line follow function

//----------------------------------------------------------------------------
// run_follow_light_mode : 
// =====================
//
// Description
//      Vehicle uses two LDR sensors to detect incident light.  As the light
//      intensity increases, the value read from the sensor decreases.
//
// Notes
//
//      Active switches are 
//          switch A = go/stop button
//          switch B = change setting by reading POT_1, POT_2 and POT_3
//          switch C = exit mode
//
//      Active pots
//          POT_1 : ambient light setting   0->63
//          POT_2 : drag speed setting   (if switch B is pressed for less than 2 seconds)
//          POT_2 : deadband 0->15       (if switch B is pressed for more than 2 seconds)
//          POT_3 : sample rate   -  0->200mS
//

uint8_t run_follow_light_mode(void) {

uint8_t       L_light, R_light, L_ambient, R_ambient, ambient_diff;
uint8_t       sample_time, ambient;
int8_t        L_correction, R_correction;
mode_state_t  state;
int16_t       last_error, integral, error;
int16_t       p_term, i_term, d_term, PID_correction, left_pwm, right_pwm;

    state = MODE_INIT;
    left_speed  = BASE_LIGHT_FOLLOW_PWM;
    right_speed = BASE_LIGHT_FOLLOW_PWM;
    sample_time = DEFAULT_LIGHT_SAMPLE_TIME;
   
    ambient = DEFAULT_AMBIENT;

    set_LED(LED_A, FLASH_ON);
    set_LED(LED_B, FLASH_ON);
    set_LED(LED_C, FLASH_ON);
    clr_LED(LED_D);
//
// read sensors to get ambient light values and compute correction values
//
    L_ambient = ~get_adc(FRONT_SENSOR_L);  
    R_ambient = ~get_adc(FRONT_SENSOR_R);
    if (L_ambient > R_ambient) {   // Right sensor brighter
        ambient_diff = L_ambient - R_ambient;
    	L_correction = -(int8_t)(ambient_diff >> 1);
    	R_correction = +(ambient_diff >> 1);
    } else {
        ambient_diff = R_ambient - L_ambient;
    	L_correction = +(ambient_diff >> 1);
    	R_correction = -(int8_t)(ambient_diff >> 1);
    }
//
// main loop
//       
    FOREVER {
    	SET_LOG_PIN;
//
// read pots to set system characteristics
//    
        if (switch_B == PRESSED) {
/*
            CLR_TIMER16;
            SOUND_READ_POTS;
            WAIT_SWITCH_RELEASED(switch_B);     // wait until button returns to quiescent state 
            GET_TIMER16(ticks);                 // read 16-bit 8mS tick counter
            
            ad_value = get_adc(POT_1);                     
            if (ticks > (2 * TICKS_IN_ONE_SECOND)) {          // button press time > 2 seconds
                light_deadband =  ((ad_value >> 4) & 0x0F);       // convert to 0->15 
            } else {
                ambient =  ((ad_value >> 2) & 0x3F);              // convert to 0->63 
            }            
            ad_value = get_adc(POT_2);
            temp16 =  ((ad_value >> 3) & 0x1F);                   // convert to 0->31
            temp16 = (temp16 * left_speed) / 0x1F;                // convert to 0->left_speed
            drag_speed = left_speed - (uint8_t)temp16;            // convert to left_speed->0

            ad_value = get_adc(POT_3);
            temp16 =  ((ad_value >> 3) & 0x1F);                   // convert to 0->31
            temp16 = (temp16 * 20) / 0x1F;                        // convert to 0->20
            sample_time = (uint8_t)temp16 * 10;                   // convert to 0->200mS
            if (sample_time == 0) {              // ensure that sample time is not 0
                sample_time = 10;
            }
            //
            // recalculate ambient
            //
            L_ambient = get_adc(FRONT_SENSOR_L);  
            R_ambient = get_adc(FRONT_SENSOR_R);
            if (L_ambient > R_ambient) {
                ambient_diff = L_ambient - R_ambient;
            } else {
                ambient_diff = R_ambient - L_ambient;
            }
*/
        }
//
//  check for exit
//
        if (switch_C == PRESSED) {            //  exit mode
            vehicle_stop();
            SOUND_EXIT_SELECTION;
            WAIT_SWITCH_RELEASED(switch_C);
            return 0;
        }     
//
//  run simple state machine to define operating modes
//  There are two states : MODE_INIT and MODE_RUNNING
//
      if (state == MODE_INIT) {
            if (switch_A == PRESSED) {            //  go to RUN state              
                state = MODE_RUNNING;
                WAIT_SWITCH_RELEASED(switch_A); 
            } else {
                continue;                         // back to begining of FOREVER loop
            }
        }
        if (state == MODE_RUNNING) { 
            if (switch_A == PRESSED) {            // halt bump activity
                state = MODE_INIT;
                vehicle_stop();
                WAIT_SWITCH_RELEASED(switch_A);
                continue;
            }
        }  
       
//
// Read sensors and apply corrections for different sensor responses	
//
	L_light = (~get_adc(FRONT_SENSOR_L)) + L_correction;  
    R_light = (~get_adc(FRONT_SENSOR_R)) + R_correction;
	
	error = (R_light - L_light); // << LIGHT_ERROR_GAIN; // apply gain to error value (power of 2)
//
// P calculation : P = error * Kp
//
		p_term = error * LIGHT_KP_INT;
//
// I calculation : integral * Ki
//                 with integral windup protection
//
		integral += error;
		if (integral > LIGHT_INTEGRAL_WINDUP_LIMIT) {
			integral = LIGHT_INTEGRAL_WINDUP_LIMIT;
		}
		if (integral < -LIGHT_INTEGRAL_WINDUP_LIMIT) {
			integral = -LIGHT_INTEGRAL_WINDUP_LIMIT;
		}
		i_term = integral * LIGHT_KI_INT; // * DELTA_T;
//
// D calculation : D = (Current error - error) * Kd
//
		d_term = ((error - last_error) * LIGHT_KD_INT);   // / DELTA_T;
//
// Calculate PID correction and rescale
//
		PID_correction = (p_term + i_term + d_term) / TIME_SCALE_FACTOR;
		
		last_error = error;	
		
//
// Calculate pwm values and clamp to 0 -> 100%
//
		left_pwm = BASE_LIGHT_FOLLOW_PWM + PID_correction;		
		right_pwm = BASE_LIGHT_FOLLOW_PWM - PID_correction;
		if (left_pwm > 100) {
			left_pwm = 100;
		} else {
			if (left_pwm < -100) {
				left_pwm = -100;
			}
		}
		if (right_pwm > 100) {
			right_pwm = 100;
		} else {
			if (right_pwm < -100) {
				right_pwm = -100;
			}
		}
//
// set motors. Allow motors to reverse.
// 
		// set_motor_speed_dir(LEFT_MOTOR, (uint8_t)left_pwm);
		// set_motor_speed_dir(RIGHT_MOTOR, (uint8_t)right_pwm);  
		SET_MOTORS(left_pwm, right_pwm); 

        CLEAR_LOG_PIN;
        DelayMs(sample_time); 
                   
    }        // end of FOREVER loop
}
