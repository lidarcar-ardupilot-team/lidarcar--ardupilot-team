#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/app_config.h"
#include "app/app_types.h"
#include "app/core/app_math.h"
#include "app/hw/oled_display.h"
#include "app/hw/debug_uart.h"
#include "app/hw/keys.h"
#include "app/hw/gray_sensor.h"
#include "app/hw/encoder.h"
#include "app/hw/motor_driver.h"
#include "app/radar_protocol.h"
#include "app/wireless_protocol.h"
#include "app/wireless_diag.h"

#define RADAR_R1_NAV_TEST_ENABLE 1
#define RADAR_R1_MOTOR_OUTPUT_ENABLE 1
#define RADAR_R1_USE_BRIDGE_YAW_ONLY 1
#define RADAR_R1_PHASE8_OLED_DIAG 1
#define RADAR_R1_JY_YAW_RELATIVE_TO_RADAR_ORIGIN 0
#define RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE 1
#define RADAR_R1_A_FORBID_TEST_DISABLE_B 0
/* Trial 2026-07-06: add +15 PWM to forward and turn commands for v75 field test. */
#define RADAR_R1_PWM_TRIAL_DELTA 15
#define RADAR_R1_FWD_PWM(v)      ((v) + RADAR_R1_PWM_TRIAL_DELTA)
#define RADAR_R1_TURN_PWM(v)     ((v) + RADAR_R1_PWM_TRIAL_DELTA)
#define RADAR_R1_FORBID_PROTECT_DIST_CM 40
#define RADAR_R1_FORBID_PLAN_MARGIN_CM 5
#define RADAR_R1_FORBID_HARD_STOP_SHRINK_CM 10
#define RADAR_R1_FORBID_SOFT_COST_DIST_CM 50
#define RADAR_R1_FORBID_SOFT_COST_STEP_CM 5
#define RADAR_R1_FORBID_SOFT_COST_PER_STEP 40U
#define RADAR_R1_FORBID_SOFT_COST_CORRIDOR_PCT 75U
#define RADAR_R1_ASTAR_HOME_PLAN_MARGIN_CM 20
#define RADAR_R1_ASTAR_HOME_SOFT_COST_DIST_CM 80
#define RADAR_R1_ASTAR_HOME_SOFT_COST_PER_STEP 90U
#define RADAR_R1_FORBID_DANGER_DIST_CM 24
#define RADAR_R1_FORBID_DANGER_INNER_PWM 120
#define RADAR_R1_FORBID_DANGER_OUTER_PWM 225
#define RADAR_R1_TARGET_POCKET_REWARD_DIST_CM 60
#define RADAR_R1_TARGET_POCKET_REWARD_STEP_CM 10
#define RADAR_R1_TARGET_POCKET_REWARD_PER_STEP 30U
#define RADAR_R1_FORBID_PROTECT_SCALE_PCT 60
#define RADAR_R1_FORBID_PROTECT_ESCAPE_PWM 45
#define RADAR_R1_ASTAR_HOME_FORBID_PROTECT_ESCAPE_PWM \
    55
#define RADAR_R1_FORWARD_LEFT_BIAS_PWM 10
#define WIRELESS_TARGET_AUTO_START_ENABLE 1
#define RADAR_R1_A_FORBID_ORTHO_ROUTE_ENABLE 0
#define RADAR_R1_A_FORBID_ASTAR_ORTHO_NEIGHBOR_ENABLE 1
#define RADAR_R1_A_FORBID_WEAK_FOLLOW_ENABLE 1
#define RADAR_R1_A_FORBID_ALIGN_DEG 8
#define RADAR_R1_A_FORBID_DRIVE_DEG 25
#define RADAR_R1_A_FORBID_TURN_PWM RADAR_R1_TURN_PWM(125)
#define RADAR_R1_A_FORBID_WEAK_FORWARD_PWM RADAR_R1_FWD_PWM(250)
#define RADAR_R1_A_FORBID_WEAK_FORWARD_SLOW_PWM RADAR_R1_FWD_PWM(215)
#define RADAR_R1_A_FORBID_WEAK_TURN_GAIN 1
#define RADAR_R1_A_FORBID_WEAK_TURN_MAX_PWM RADAR_R1_TURN_PWM(65)
#define RADAR_R1_A_FORBID_YAW_SLEW_DEG 4
#define RADAR_R1_A_FORBID_BIG_YAW_DEG 45
#define RADAR_R1_A_FORBID_MID_YAW_DEG 25
#define RADAR_R1_A_FORBID_BIG_TURN_PWM RADAR_R1_TURN_PWM(125)
#define RADAR_R1_A_FORBID_MID_FORWARD_PWM RADAR_R1_FWD_PWM(190)
/*
 * Return-home A* uses the same path executor, but the first return corner can
 * create a >100deg temporary yaw error while the car is already moving away
 * from the QR.  Do not use the target-side hard spin threshold here; otherwise
 * the car only travels one short return segment and then keeps turning in
 * place at the next waypoint.  Keep it slower and gentler, but allow a larger
 * yaw window before declaring "spin only".
 */
#define RADAR_R1_ASTAR_HOME_BIG_YAW_DEG 90
#define RADAR_R1_ASTAR_HOME_MID_YAW_DEG 45
#define RADAR_R1_ASTAR_HOME_BIG_TURN_PWM RADAR_R1_TURN_PWM(105)
#define RADAR_R1_ASTAR_HOME_MID_FORWARD_PWM RADAR_R1_FWD_PWM(175)
#define RADAR_R1_ASTAR_HOME_WEAK_TURN_MAX_PWM RADAR_R1_TURN_PWM(55)
#define RADAR_R1_ASTAR_HOME_NEAR_FORBID_BIG_YAW_DEG 45
#define RADAR_R1_ASTAR_HOME_NEAR_FORBID_MID_YAW_DEG 25
#define RADAR_R1_ASTAR_HOME_NEAR_FORBID_FORWARD_PWM RADAR_R1_FWD_PWM(125)
#define RADAR_R1_ASTAR_HOME_NEAR_FORBID_MIN_PWM RADAR_R1_FWD_PWM(150)
#define RADAR_R1_ASTAR_HOME_NEAR_FORBID_TURN_MAX_PWM RADAR_R1_TURN_PWM(55)
#define RADAR_R1_ASTAR_HOME_STOP_X_CM 10
#define RADAR_R1_ASTAR_HOME_STOP_Y_CM 10
#define RADAR_R1_ASTAR_A_HOME_STOP_X_CM 8
#define RADAR_R1_ASTAR_A_HOME_STOP_Y_CM 8
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_DIST_CM 70
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_YAW_DEG 90
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_ALIGN_DEG 25
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_ALIGN_PWM RADAR_R1_TURN_PWM(85)
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_PWM RADAR_R1_FWD_PWM(120)
#define RADAR_R1_ASTAR_HOME_FINAL_REVERSE_TURN_MAX_PWM RADAR_R1_TURN_PWM(45)
#define RADAR_R1_A_FORBID_NEAR_SCALE_PCT 75
#define RADAR_R1_A_FORBID_LOOKAHEAD_CM 30
#define RADAR_R1_A_FORBID_LOOKAHEAD_NEAR_CM 18
#define RADAR_R1_A_FORBID_LOOKAHEAD_GOAL_CM 15
#define RADAR_R1_A_FORBID_GOAL_NEAR_CM 60
#define RADAR_R1_A_FORBID_ASTAR_TURN_45_COST 8
#define RADAR_R1_A_FORBID_ASTAR_TURN_90_COST 35
#define RADAR_R1_A_FORBID_ASTAR_TURN_135_COST 70
#define RADAR_R1_A_FORBID_ASTAR_TURN_180_COST 120
#define RADAR_R1_A_FORBID_ASTAR_START_REVERSE_COST 220
#define RADAR_R1_A_FORBID_AXIS_EXPAND_ENABLE 1
#define RADAR_R1_ASTAR_HOME_SHORTCUT_ENABLE 1
#define RADAR_R1_ASTAR_HOME_SHORTCUT_STEP_CM 5
#define RADAR_R1_A_FORBID_PLAN_CLEARANCE_CM 0
#define RADAR_R1_A_FORBID_AXIS_REACH_CM 10
#define RADAR_R1_A_FORBID_AXIS_CROSSTRACK_CM 55
#define RADAR_R1_ASTAR_HOME_AXIS_CROSSTRACK_CM 35
#define RADAR_R1_A_FORBID_TARGET_STOP_X_CM 18
#define RADAR_R1_A_FORBID_TARGET_STOP_Y_CM 18
#define RADAR_R1_A_FORBID_TARGET_BONUS_X_CM 10
#define RADAR_R1_A_FORBID_TARGET_BONUS_Y_CM 10
#define RADAR_R1_A_FORBID_A_TARGET_BONUS_X_CM 8
#define RADAR_R1_A_FORBID_A_TARGET_BONUS_Y_CM 8
#define RADAR_R1_A_FORBID_TARGET_ACCEPT_X_CM 15
#define RADAR_R1_A_FORBID_TARGET_ACCEPT_Y_CM 15
#define RADAR_R1_A_FORBID_TARGET_SLOW_DIST_CM 45
#define RADAR_R1_A_FORBID_TARGET_SLOW_SCALE_PCT 60
#define RADAR_R1_A_FORBID_TARGET_FINE_FORWARD_PWM RADAR_R1_FWD_PWM(100)
#define RADAR_R1_A_FORBID_TARGET_FINE_TURN_MAX_PWM RADAR_R1_TURN_PWM(25)
#define RADAR_R1_A_FORBID_TARGET_FINE_BIG_ERR_DEG 25
#define RADAR_R1_A_FORBID_TARGET_FINE_BIG_TURN_MAX_PWM RADAR_R1_TURN_PWM(45)
#define RADAR_R1_A_FORBID_TARGET_FINE_APPROACH_YAW_DEG 45
#define RADAR_R1_A_FORBID_TARGET_FINE_APPROACH_TURN_MAX_PWM RADAR_R1_TURN_PWM(20)
#define RADAR_R1_A_FORBID_TARGET_FINE_ALIGN_DEG 8
#define RADAR_R1_A_FORBID_TARGET_FINE_MAX_TRIES 2U
#define RADAR_R1_A_FORBID_TARGET_FINE_STABLE_MS 1200U
#define RADAR_R1_A_FORBID_TARGET_FINE_DRIVE_MS 800U
#define RADAR_R1_A_FORBID_TARGET_FINE_DRIVE_PWM \
    (RADAR_R1_A_FORBID_TARGET_FINE_FORWARD_PWM - 15)
/* A-code TO_TARGET only:
 * The latest A log still parsed target=(-85,265) correctly, but A* liked the
 * x≈0,y≈280 corridor beside the front forbidden block.  That leaves too much
 * right-side drift before the final left move.  Add a soft, non-blocking cost
 * after clearing the lower left/rear box so A shifts left before y≈260, while
 * keeping the final QR coordinate and all B/C/return behavior unchanged.
 */
#define RADAR_R1_A_ROUTE_CORRIDOR_Y_MIN_CM 245
#define RADAR_R1_A_ROUTE_CORRIDOR_Y_MAX_CM 330
#define RADAR_R1_A_ROUTE_CORRIDOR_MAX_X_CM (-40)
#define RADAR_R1_A_ROUTE_CORRIDOR_COST_STEP_CM 10
#define RADAR_R1_A_ROUTE_CORRIDOR_COST_PER_STEP 160U
#define RADAR_R1_A_ROUTE_CORRIDOR_COST_MAX 1000U
#define RADAR_R1_A_ROUTE_LOW_LEFT_Y_MIN_CM 190
#define RADAR_R1_A_ROUTE_LOW_LEFT_Y_MAX_CM 245
#define RADAR_R1_A_ROUTE_LOW_LEFT_MIN_X_CM (-35)
#define RADAR_R1_A_ROUTE_LOW_LEFT_COST_PER_STEP 120U
#define RADAR_R1_A_ROUTE_LOW_LEFT_COST_MAX 720U
/* The measured left/rear forbidden zone is slanted, but represented as an
 * axis-aligned AABB.  In A-code TO_TARGET only, this AABB can falsely classify
 * the visually open top-right passage as hard forbidden around x≈-50,y≈228.
 * Keep normal protect/slow behavior there, but do not convert it into a
 * mission-ending hard stop.
 */
#define RADAR_R1_A_OPEN_PASSAGE_X_MIN_CM (-80)
#define RADAR_R1_A_OPEN_PASSAGE_X_MAX_CM (-40)
#define RADAR_R1_A_OPEN_PASSAGE_Y_MIN_CM 210
#define RADAR_R1_A_OPEN_PASSAGE_Y_MAX_CM 235
#define RADAR_R1_B_TARGET_SAFE_ACCEPT_X_CM 10
#define RADAR_R1_B_TARGET_SAFE_ACCEPT_Y_CM 18
#define RADAR_R1_B_TARGET_SAFE_MAX_X_CM 18
#define RADAR_R1_B_TARGET_FINE_TIMEOUT_MS 2500U
#define RADAR_R1_TARGET_TO_HOME_SETTLE_MS 1500U
#define RADAR_R1_PRE_RETURN_FINE_ENABLE 1
#define RADAR_R1_C_ASTAR_TARGET_X_CM 148
#define RADAR_R1_C_ASTAR_TARGET_Y_CM 165
#define RADAR_R1_WIRELESS_A_TARGET_Y_BIAS_CM 5
#define RADAR_R1_WIRELESS_B_TARGET_X_BIAS_CM (-6)
#define RADAR_R1_A_FORBID_CORNER_Y1_CM 50
#define RADAR_R1_A_FORBID_CORNER_X_CM (-60)
#define RADAR_R1_A_FORBID_CORNER_Y2_CM 250
#define RADAR_R1_ROUTE_A 0
#define RADAR_R1_ROUTE_B 1
#define RADAR_R1_ROUTE_C 2
/* 2026-05-25: physical A QR code known-good route.
 * The verified y260/x-90 geometry is now owned by ROUTE_A; do not
 * switch back to ROUTE_C for this target unless the route is revalidated.
 */
#define RADAR_R1_FIXED_ROUTE_TARGET RADAR_R1_ROUTE_A
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_B
#define RADAR_R1_TUNE_TAG        "r1_b_y280_right0_x25_drift18"
#elif RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
#define RADAR_R1_TUNE_TAG        "phase9_a_highleft_bx9_bsafe"
#else
#define RADAR_R1_TUNE_TAG        "phase9_a_home23_turnsoft3_pwm5"
#endif
#elif RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_C
#define RADAR_R1_TUNE_TAG        "phase9_c_unvalidated"
#else
#define RADAR_R1_TUNE_TAG        "phase8_bridge_xyz_no_motor_jy_log"
#endif
#define RADAR_R1_MAX_PWM         380
#define RADAR_R1_MIN_PWM         RADAR_R1_FWD_PWM(160)
#define RADAR_R1_TURN_MIN_PWM    RADAR_R1_TURN_PWM(95)
#define RADAR_R1_TURN_IN_PLACE_PWM RADAR_R1_TURN_PWM(127)
#define RADAR_R1_TURN_SLOW_PWM   RADAR_R1_TURN_PWM(109)
#define RADAR_R1_TURN_SLOWDOWN_DEG 35
#define RADAR_R1_HEADING_CORRECT_PWM RADAR_R1_TURN_PWM(130)
#define RADAR_R1_STRAIGHT_BASE_L RADAR_R1_FWD_PWM(330)
#define RADAR_R1_STRAIGHT_BASE_R RADAR_R1_FWD_PWM(300)
#define RADAR_R1_STRAIGHT_YAW_GAIN 3
#define RADAR_R1_STRAIGHT_CORR_MAX_PWM RADAR_R1_TURN_PWM(45)
#define RADAR_R1_STRAIGHT_HEADING_CORRECT_DEG 20
#define RADAR_R1_STRAIGHT_RECOVERY_FORWARD_PWM RADAR_R1_FWD_PWM(220)
#define RADAR_R1_STRAIGHT_RECOVERY_TURN_PWM RADAR_R1_TURN_PWM(55)
#define RADAR_R1_CROSSTRACK_ENABLE_YAW_DEG 3
#define RADAR_R1_CROSSTRACK_GAIN_DIV 5
#define RADAR_R1_CROSSTRACK_MAX_DEG 6
#define RADAR_R1_RETURN_X_CROSSTRACK_GAIN_DIV 4
#define RADAR_R1_RETURN_X_CROSSTRACK_MAX_DEG 8
#define RADAR_R1_RETURN_HOME_CROSSTRACK_GAIN_DIV 3
#define RADAR_R1_RETURN_HOME_CROSSTRACK_MAX_DEG 10
#define RADAR_R1_LINEERR_FILTER_DIV 1
#define RADAR_R1_LINEERR_ACC_DEADBAND_CM 8
#define RADAR_R1_LINEERR_ACC_START_CM 18
#define RADAR_R1_LINEERR_ACC_INPUT_DIV 3
#define RADAR_R1_LINEERR_ACC_SAMPLE_MS 100U
#define RADAR_R1_LINEERR_ACC_LEAK_NUM 7
#define RADAR_R1_LINEERR_ACC_LEAK_DEN 8
#define RADAR_R1_LINEERR_ACC_LIMIT 80
#define RADAR_R1_LINEERR_ACC_PWM_DIV 16
#define RADAR_R1_LINEERR_ACC_MAX_PWM 6
#define RADAR_R1_LINEERR_ACC_YAW_MAX_DEG 18
#define RADAR_R1_TURN_GAIN       3
#define RADAR_R1_TURN_MAX_PWM    RADAR_R1_TURN_PWM(170)
#define RADAR_R1_LEFT_TURN_RIGHT_SCALE_PCT 85
#define RADAR_R1_SPIN_GAIN       2
#define RADAR_R1_SPIN_MAX_PWM    RADAR_R1_TURN_PWM(160)
#define RADAR_R1_CMD_SLEW_PER_UPDATE 60
#define RADAR_R1_YAW_FILTER_DIV  4
#define RADAR_R1_PI_YAW_JUMP_DEG 30
#define RADAR_R1_PI_YAW_FALLBACK_MS 800U
#define RADAR_R1_LOG_UNARMED_TAIL_MS 5000U
#define RADAR_R1_FIXED_ROUTE_TEST 1
#define RADAR_R1_FIXED_FIRST_X_CM 0
#define RADAR_R1_A_FIXED_FINAL_Y_CM 265
#define RADAR_R1_A_FIXED_EXTRA_Y_CM 0
#define RADAR_R1_A_FIXED_Y_STOP_DELTA_CM 235
#define RADAR_R1_A_FIXED_Y_SEGMENT_CM 260
#define RADAR_R1_A_FIXED_FINAL_X_CM 85
#define RADAR_R1_A_FIXED_FINAL_X_SIGN (-1)
#define RADAR_R1_A_FIXED_FINAL_TURN_YAW_DEG 180
#define RADAR_R1_A_SLOWDOWN_DIST_CM 20
#define RADAR_R1_A_SLOW_SCALE_PCT 75
#define RADAR_R1_A_FINAL_GATE_CM 5
#define RADAR_R1_A_RETURN_CORNER60_ENABLE 1
#define RADAR_R1_A_RETURN_TURN1_YAW_DEG 90
#define RADAR_R1_A_RETURN_TURN2_YAW_DEG 0
#define RADAR_R1_A_RETURN_X_CM 98
#define RADAR_R1_A_RETURN_X_SIGN 1
#define RADAR_R1_A_RETURN_HOME_TURN_YAW_DEG (-90)
#define RADAR_R1_A_RETURN_HOME_ENCODER_CAP_CM 320
#define RADAR_R1_A_RETURN_X_LINEERR_CLAMP_CM 15
#define RADAR_R1_A_RETURN_PRETURN_STOP_MS 1500U
#define RADAR_R1_A_RETURN_TURN_GAP_MS 800U
#define RADAR_R1_A_RETURN_X_RELEASE_MS 800U
#define RADAR_R1_A_RETURN_HOME_RELEASE_MS 800U
#define RADAR_R1_A_RETURN_HOME_REACH_CM 28
#define RADAR_R1_B_FIXED_FINAL_Y_CM 280
#define RADAR_R1_B_FIXED_EXTRA_Y_CM 20
#define RADAR_R1_B_FIXED_Y_STOP_DELTA_CM 280
#define RADAR_R1_B_FIXED_Y_SEGMENT_CM 280
#define RADAR_R1_B_FIXED_FINAL_X_CM 25
#define RADAR_R1_B_FIXED_FINAL_X_SIGN 1
#define RADAR_R1_B_FIXED_FINAL_TURN_YAW_DEG 0
#define RADAR_R1_B_RUNTIME_FINAL_Y_CM 285
#define RADAR_R1_B_RUNTIME_Y_STOP_GATE_CM 260
#define RADAR_R1_B_RUNTIME_FINAL_X_CM 9
#define RADAR_R1_B_RUNTIME_X_SLOWDOWN_DIST_CM 20
#define RADAR_R1_B_RUNTIME_X_ENCODER_CAP_CM 30
#define RADAR_R1_B_RUNTIME_X_STOP_GATE_CM 8
#define RADAR_R1_B_RUNTIME_X_SLOW_SCALE_PCT 25
#define RADAR_R1_C_FIXED_FINAL_Y_CM 147
#define RADAR_R1_C_FIXED_EXTRA_Y_CM 0
#define RADAR_R1_C_FIXED_Y_STOP_DELTA_CM 147
#define RADAR_R1_C_FIXED_Y_SEGMENT_CM 147
#define RADAR_R1_C_FIXED_FINAL_X_CM 138
#define RADAR_R1_C_FIXED_FINAL_X_SIGN 1
#define RADAR_R1_C_FIXED_FINAL_TURN_YAW_DEG 0
#define RADAR_R1_C_RETURN_CORNER60_ENABLE 1
#define RADAR_R1_C_RETURN_TURN1_YAW_DEG 90
#define RADAR_R1_C_RETURN_TURN2_YAW_DEG 0
#define RADAR_R1_C_RETURN_X_CM 75
#define RADAR_R1_C_RETURN_X_SIGN 1
#define RADAR_R1_C_RETURN_HOME_TURN_YAW_DEG (-90)
#define RADAR_R1_C_RETURN_HOME_ENCODER_CAP_CM 320
#define RADAR_R1_C_RETURN_X_LINEERR_CLAMP_CM 15
#define RADAR_R1_C_RETURN_PRETURN_STOP_MS 3000U
#define RADAR_R1_C_RETURN_TURN_GAP_MS 800U
#define RADAR_R1_C_RETURN_X_RELEASE_MS 800U
#define RADAR_R1_C_RETURN_HOME_RELEASE_MS 800U
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_B
#define RADAR_R1_FIXED_FINAL_Y_CM RADAR_R1_B_FIXED_FINAL_Y_CM
#define RADAR_R1_FIXED_EXTRA_Y_CM RADAR_R1_B_FIXED_EXTRA_Y_CM
#define RADAR_R1_FIXED_Y_STOP_DELTA_CM RADAR_R1_B_FIXED_Y_STOP_DELTA_CM
#define RADAR_R1_FIXED_Y_SEGMENT_CM RADAR_R1_B_FIXED_Y_SEGMENT_CM
#define RADAR_R1_FIXED_FINAL_X_CM RADAR_R1_B_FIXED_FINAL_X_CM
#define RADAR_R1_FIXED_FINAL_X_SIGN RADAR_R1_B_FIXED_FINAL_X_SIGN
#define RADAR_R1_FIXED_FINAL_TURN_YAW_DEG RADAR_R1_B_FIXED_FINAL_TURN_YAW_DEG
#elif RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
#define RADAR_R1_FIXED_FINAL_Y_CM RADAR_R1_A_FIXED_FINAL_Y_CM
#define RADAR_R1_FIXED_EXTRA_Y_CM RADAR_R1_A_FIXED_EXTRA_Y_CM
#define RADAR_R1_FIXED_Y_STOP_DELTA_CM RADAR_R1_A_FIXED_Y_STOP_DELTA_CM
#define RADAR_R1_FIXED_Y_SEGMENT_CM RADAR_R1_A_FIXED_Y_SEGMENT_CM
#define RADAR_R1_FIXED_FINAL_X_CM RADAR_R1_A_FIXED_FINAL_X_CM
#define RADAR_R1_FIXED_FINAL_X_SIGN RADAR_R1_A_FIXED_FINAL_X_SIGN
#define RADAR_R1_FIXED_FINAL_TURN_YAW_DEG RADAR_R1_A_FIXED_FINAL_TURN_YAW_DEG
#elif RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_C
#define RADAR_R1_FIXED_FINAL_Y_CM RADAR_R1_C_FIXED_FINAL_Y_CM
#define RADAR_R1_FIXED_EXTRA_Y_CM RADAR_R1_C_FIXED_EXTRA_Y_CM
#define RADAR_R1_FIXED_Y_STOP_DELTA_CM RADAR_R1_C_FIXED_Y_STOP_DELTA_CM
#define RADAR_R1_FIXED_Y_SEGMENT_CM RADAR_R1_C_FIXED_Y_SEGMENT_CM
#define RADAR_R1_FIXED_FINAL_X_CM RADAR_R1_C_FIXED_FINAL_X_CM
#define RADAR_R1_FIXED_FINAL_X_SIGN RADAR_R1_C_FIXED_FINAL_X_SIGN
#define RADAR_R1_FIXED_FINAL_TURN_YAW_DEG RADAR_R1_C_FIXED_FINAL_TURN_YAW_DEG
#else
#define RADAR_R1_FIXED_FINAL_Y_CM RADAR_R1_A_FIXED_FINAL_Y_CM
#define RADAR_R1_FIXED_EXTRA_Y_CM RADAR_R1_A_FIXED_EXTRA_Y_CM
#define RADAR_R1_FIXED_Y_STOP_DELTA_CM RADAR_R1_A_FIXED_Y_STOP_DELTA_CM
#define RADAR_R1_FIXED_Y_SEGMENT_CM RADAR_R1_A_FIXED_Y_SEGMENT_CM
#define RADAR_R1_FIXED_FINAL_X_CM RADAR_R1_A_FIXED_FINAL_X_CM
#define RADAR_R1_FIXED_FINAL_X_SIGN RADAR_R1_A_FIXED_FINAL_X_SIGN
#define RADAR_R1_FIXED_FINAL_TURN_YAW_DEG RADAR_R1_A_FIXED_FINAL_TURN_YAW_DEG
#endif
#define RADAR_R1_FIXED_Y_TARGET_CM     (RADAR_R1_FIXED_FINAL_Y_CM + RADAR_R1_FIXED_EXTRA_Y_CM)
#define RADAR_R1_FIXED_FINAL_X_DELTA_CM \
    (RADAR_R1_FIXED_FINAL_X_SIGN * RADAR_R1_FIXED_FINAL_X_CM)
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
#define RADAR_R1_RETURN_CORNER60_ENABLE RADAR_R1_A_RETURN_CORNER60_ENABLE
#define RADAR_R1_RETURN_TURN1_YAW_DEG RADAR_R1_A_RETURN_TURN1_YAW_DEG
#define RADAR_R1_RETURN_TURN2_YAW_DEG RADAR_R1_A_RETURN_TURN2_YAW_DEG
#define RADAR_R1_RETURN_X_DELTA_CM \
    (RADAR_R1_A_RETURN_X_SIGN * RADAR_R1_A_RETURN_X_CM)
#define RADAR_R1_RETURN_HOME_TURN_YAW_DEG RADAR_R1_A_RETURN_HOME_TURN_YAW_DEG
#define RADAR_R1_RETURN_HOME_ENCODER_CAP_COUNTS \
    (RADAR_R1_A_RETURN_HOME_ENCODER_CAP_CM * RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM)
#define RADAR_R1_RETURN_X_LINEERR_CLAMP_CM RADAR_R1_A_RETURN_X_LINEERR_CLAMP_CM
#define RADAR_R1_RETURN_PRETURN_STOP_MS RADAR_R1_A_RETURN_PRETURN_STOP_MS
#define RADAR_R1_RETURN_TURN_GAP_MS RADAR_R1_A_RETURN_TURN_GAP_MS
#define RADAR_R1_RETURN_X_RELEASE_MS RADAR_R1_A_RETURN_X_RELEASE_MS
#define RADAR_R1_RETURN_HOME_RELEASE_MS RADAR_R1_A_RETURN_HOME_RELEASE_MS
#elif RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_C
#define RADAR_R1_RETURN_CORNER60_ENABLE RADAR_R1_C_RETURN_CORNER60_ENABLE
#define RADAR_R1_RETURN_TURN1_YAW_DEG RADAR_R1_C_RETURN_TURN1_YAW_DEG
#define RADAR_R1_RETURN_TURN2_YAW_DEG RADAR_R1_C_RETURN_TURN2_YAW_DEG
#define RADAR_R1_RETURN_X_DELTA_CM \
    (RADAR_R1_C_RETURN_X_SIGN * RADAR_R1_C_RETURN_X_CM)
#define RADAR_R1_RETURN_HOME_TURN_YAW_DEG RADAR_R1_C_RETURN_HOME_TURN_YAW_DEG
#define RADAR_R1_RETURN_HOME_ENCODER_CAP_COUNTS \
    (RADAR_R1_C_RETURN_HOME_ENCODER_CAP_CM * RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM)
#define RADAR_R1_RETURN_X_LINEERR_CLAMP_CM RADAR_R1_C_RETURN_X_LINEERR_CLAMP_CM
#define RADAR_R1_RETURN_PRETURN_STOP_MS RADAR_R1_C_RETURN_PRETURN_STOP_MS
#define RADAR_R1_RETURN_TURN_GAP_MS RADAR_R1_C_RETURN_TURN_GAP_MS
#define RADAR_R1_RETURN_X_RELEASE_MS RADAR_R1_C_RETURN_X_RELEASE_MS
#define RADAR_R1_RETURN_HOME_RELEASE_MS RADAR_R1_C_RETURN_HOME_RELEASE_MS
#else
#define RADAR_R1_RETURN_CORNER60_ENABLE 0
#define RADAR_R1_RETURN_TURN1_YAW_DEG 90
#define RADAR_R1_RETURN_TURN2_YAW_DEG 0
#define RADAR_R1_RETURN_X_DELTA_CM 0
#define RADAR_R1_RETURN_HOME_TURN_YAW_DEG (-90)
#define RADAR_R1_RETURN_HOME_ENCODER_CAP_COUNTS RADAR_R1_GUARD_ENCODER_CAP_COUNTS
#define RADAR_R1_RETURN_X_LINEERR_CLAMP_CM RADAR_R1_RIGHT_LEG_LINEERR_CLAMP_CM
#define RADAR_R1_RETURN_PRETURN_STOP_MS 0U
#define RADAR_R1_RETURN_TURN_GAP_MS 400U
#define RADAR_R1_RETURN_X_RELEASE_MS RADAR_R1_FINAL_X_RELEASE_MS
#define RADAR_R1_RETURN_HOME_RELEASE_MS RADAR_R1_FINAL_X_RELEASE_MS
#endif
#define RADAR_R1_FIXED_FIRST_REACH_CM 5
#define RADAR_R1_FIXED_Y_REACH_CM 6
#define RADAR_R1_FIXED_FINAL_REACH_CM 5
#define RADAR_R1_FIXED_FINAL_TURN_REACH_DEG 5
#define RADAR_R1_FINAL_TURN_Y_DRIFT_LIMIT_CM 18
#define RADAR_R1_RIGHT_LEG_LINEERR_CLAMP_CM 5
#define RADAR_R1_FINAL_TURN_PRESTOP_MS 900U
#define RADAR_R1_FINAL_TURN_PWM 115
#define RADAR_R1_FINAL_TURN_SLOW_PWM 90
#define RADAR_R1_FINAL_TURN_PRE_POSE_STABLE_CM 5
#define RADAR_R1_FINAL_TURN_PRE_STABLE_MS 400U
#define RADAR_R1_SEGMENT_SETTLE_MS 400U
#define RADAR_R1_FINAL_X_RELEASE_MS 2000U
#define RADAR_R1_TURN_SETTLE_DEG 5
#define RADAR_R1_TURN_SETTLE_RECHECK_DEG 8
#define RADAR_R1_TURN_SETTLE_MS  2000U
#define RADAR_R1_FIRST_TURN_SETTLE_MS 600U
#define RADAR_R1_RIGHT_TURN_SETTLE_MS 1200U
#define RADAR_R1_TURN_POSE_STABLE_CM 5
#define RADAR_R1_TURN_POSE_STABLE_MS 400U
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
/*
 * Phase 8C A-route dry-run showed the first y+ leg stuck in turn-align:
 * yaw=89~90, desired_yaw=94, yaw_err=4~5, turn_done=0, cmd_l/cmd_r
 * opposite-sign.  A-route field calibration expects this first leg to be
 * a clean y+ straight segment, so keep its initial heading at 90 deg and
 * let the dry-run expose same-direction forward commands before motors
 * are enabled.
 */
#define RADAR_R1_FIRST_TURN_EXTRA_DEG 0
#else
#define RADAR_R1_FIRST_TURN_EXTRA_DEG 4
#endif
#define RADAR_R1_TURN_FRESH_MAX_MS 300U
#define RADAR_R1_POST_TURN_RAMP_MS 900U
#define RADAR_R1_POST_TURN_YAW_ONLY_MS 2200U
#define RADAR_R1_POST_TURN_SCALE_START_PCT 60
#define RADAR_R1_POST_TURN_CORR_MAX_PWM 18
#define RADAR_R1_POST_TURN_LEFT_BOOST_PWM 42
#define RADAR_R1_POST_TURN_START_MIN_L 230
#define RADAR_R1_POST_TURN_START_MIN_R 180
#define RADAR_R1_LEFT_TURN_BUFFER_MS 1600U
#define RADAR_R1_LEFT_TURN_BUFFER_YAW_GAIN 2
#define RADAR_R1_LEFT_TURN_BUFFER_CORR_MAX_PWM 25
#define RADAR_R1_POSE_JUMP_REJECT_CM 250
#define RADAR_R1_ORIGIN_STABLE_MIN_FRAMES 5U
#define RADAR_R1_ORIGIN_STABLE_MIN_MS 500U
#define RADAR_R1_ORIGIN_STABLE_RANGE_CM 5
#define RADAR_R1_ORIGIN_STABLE_YAW_DEG 3
#define RADAR_R1_GUARD_NEG_Y_CM  (-20)
#define RADAR_R1_GUARD_MIN_ENC_COUNTS 80
#define RADAR_R1_GUARD_STAGNANT_ENC_COUNTS 12000
#define RADAR_R1_GUARD_STAGNANT_RADAR_CM 60
#define RADAR_R1_GUARD_STAGNANT_ABORT_ENABLE 0
#define RADAR_R1_GUARD_JY_YAW_MAX_DEG 45
#define RADAR_R1_GUARD_ENCODER_CAP_CM 252
#define RADAR_R1_GUARD_Y_SEGMENT_CAP_CM 250
#define RADAR_R1_GUARD_A_X_SEGMENT_CAP_CM 115
#define RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM 67
#define RADAR_R1_GUARD_ENCODER_CAP_COUNTS \
    (RADAR_R1_GUARD_ENCODER_CAP_CM * RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM)
#define RADAR_R1_GUARD_FINAL_X_CAP_CM RADAR_R1_FIXED_FINAL_X_CM
#define RADAR_R1_GUARD_FINAL_X_CAP_COUNTS \
    (RADAR_R1_GUARD_FINAL_X_CAP_CM * RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM)
#define RADAR_R1_GUARD_FINAL_X_STAGNANT_ENC_COUNTS \
    (8 * RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM)
#define RADAR_R1_GUARD_FINAL_X_STAGNANT_RADAR_CM 3
#define RADAR_R1_GUARD_FINAL_X_STAGNANT_ABORT_ENABLE 0
#define NAV_SPEED_LOOP_ENABLE    1
#define NAV_SPEED_LOOP_SAMPLE_MS 120U
#define NAV_SPEED_LOOP_KP_DIV    90
#define NAV_SPEED_LOOP_CORRECTION_LIMIT 10
#define NAV_SPEED_LOOP_TARGET_L_NUM 9
#define NAV_SPEED_LOOP_TARGET_L_DEN 1
#define NAV_SPEED_LOOP_TARGET_R_NUM 7
#define NAV_SPEED_LOOP_TARGET_R_DEN 1
#define NAV_SPEED_LOOP_DISTANCE_SYNC_DIV 0
#define NAV_SPEED_LOOP_SYNC_LIMIT 0
#define NAV_SPEED_LOOP_PWM_STEP_LIMIT 45
#define RADAR_R1_ALIGN_DEG       5
#define RADAR_R1_DRIVE_DEG       20
#define RADAR_R1_LOG_MS          200U
#define RADAR_R1_STREAM_LOG_ENABLE 0
#define RADAR_R1_OFFLINE_LOG_MS  500U
#define RADAR_R1_LOG_CAPACITY    128U
#define RADAR_R1_OLED_MS         500U
#define RADAR_R1_OLED_SLICE_MS   80U
/*
 * OLED writes are synchronous I2C transactions.  Do not clear/redraw the
 * whole screen during motion.  R1 refreshes one 16-char line per slice, so
 * display work is spread across frames and the 20 ms control loop is not held
 * by a long I2C burst.
 */
#define RADAR_R1_OLED_WHILE_RUNNING_ENABLE 1
#define RADAR_R1_ZERO_AT_FIRST_POSE 1
#define RADAR_R0_TEST_ENABLE     0
#define RADAR_R0_LOG_MS          1000U
#define RADAR_R0_OLED_MS         500U
#define JY901_ENABLE             1
#define JY901_USE_YAW_FOR_R1     1
#define JY901_FRAME_LEN          11U
#define JY901_HEADER             0x55U
#define JY901_ANGLE_FRAME        0x53U
#define JY901_TIMEOUT_MS         500U
#define JY901_YAW_SIGN           1
#define WIRELESS_W0_TEST_ENABLE  0
#define WIRELESS_W0_OLED_MS      300U
#define WIRELESS_W0_LOG_MS       1000U
#define WIRELESS_W0_AUTO_TX_MS   1000U
#define WIRELESS_W0_AUTO_TX_ENABLE 0
#define WIRELESS_W0_LEGACY_PING_ENABLE 0
#define MANUAL_TARGET_INPUT_ENABLE 1
#define WIRELESS_W0_DEBUG_LOG_ENABLE 1
#define WIRELESS_W0_RX_ENABLE    1
#define WIRELESS_W0_TX_TIMEOUT_MS 5U
#define WIRELESS_W0_TX_SPIN_LIMIT 80000U
#define WIRELESS_UART_IBRD_32MHZ_115200 17U
#define WIRELESS_UART_FBRD_32MHZ_115200 23U
#define DEBUG_UART_IBRD_32MHZ_115200    17U
#define DEBUG_UART_FBRD_32MHZ_115200    23U
#define WIRELESS_W0_TARGET_ID    DEV_ADDR_BROADCAST

typedef enum {
    UI_MODE_HOME = 0,
    UI_MODE_MENU,
    UI_MODE_COORD,
    UI_MODE_MAP,
    UI_MODE_ZONE,
    UI_MODE_TASK,
    UI_MODE_DEBUG
} UiMode;

typedef enum {
    UI_MENU_COORD = 0,
    UI_MENU_MAP,
    UI_MENU_ZONE,
    UI_MENU_TASK,
    UI_MENU_DEBUG,
    UI_MENU_COUNT
} UiMenuItem;

typedef enum {
    COORD_FIELD_QR = 0,
    COORD_FIELD_X,
    COORD_FIELD_Y,
    COORD_FIELD_STEP,
    COORD_FIELD_APPLY,
    COORD_FIELD_BACK,
    COORD_FIELD_COUNT
} CoordField;

typedef enum {
    MAP_FIELD_X_MIN = 0,
    MAP_FIELD_X_MAX,
    MAP_FIELD_Y_MIN,
    MAP_FIELD_Y_MAX,
    MAP_FIELD_BOUNDARY,
    MAP_FIELD_OBSTACLE,
    MAP_FIELD_HOME_X,
    MAP_FIELD_HOME_Y,
    MAP_FIELD_APPLY,
    MAP_FIELD_BACK,
    MAP_FIELD_COUNT
} MapField;

typedef enum {
    ZONE_FIELD_INDEX = 0,
    ZONE_FIELD_COUNT_SET,
    ZONE_FIELD_X_MIN,
    ZONE_FIELD_Y_MIN,
    ZONE_FIELD_X_MAX,
    ZONE_FIELD_Y_MAX,
    ZONE_FIELD_APPLY,
    ZONE_FIELD_BACK,
    ZONE_FIELD_COUNT
} ZoneField;

typedef enum {
    TASK_FIELD_TO_TARGET = 0,
    TASK_FIELD_HOME,
    TASK_FIELD_STOP,
    TASK_FIELD_CLEAR_LOG,
    TASK_FIELD_BACK,
    TASK_FIELD_COUNT
} TaskField;

static volatile uint32_t g_tickMs = 0U;

static volatile uint32_t g_radarRxByteCount = 0U;
static volatile uint32_t g_radarFrameOkCount = 0U;
static volatile uint32_t g_radarFrameErrCount = 0U;
static volatile uint32_t g_jy901RxByteCount = 0U;
static volatile uint32_t g_jy901FrameOkCount = 0U;
static volatile uint32_t g_jy901FrameErrCount = 0U;
static volatile uint8_t g_jy901RxIndex = 0U;
static volatile uint8_t g_jy901RxBuf[JY901_FRAME_LEN] = {0};
static volatile int16_t g_jy901YawRawDeg = 0;
static volatile int16_t g_jy901YawDeg = 0;
static volatile int16_t g_jy901YawOriginDeg = 0;
static volatile uint32_t g_jy901StampMs = 0U;
static volatile bool g_jy901YawOriginLocked = false;
static volatile bool g_jy901Valid = false;
static volatile uint32_t g_wirelessFrameOkCount = 0U;
static volatile uint32_t g_wirelessFrameErrCount = 0U;
static volatile uint32_t g_wirelessRxByteCount = 0U;
static volatile uint32_t g_wirelessTxFrameCount = 0U;
static volatile uint32_t g_wirelessTxByteDropCount = 0U;
static volatile uint32_t g_wirelessChecksumErrCount = 0U;
static volatile uint32_t g_wirelessLenErrCount = 0U;
static volatile uint32_t g_wirelessTypeErrCount = 0U;
static volatile uint32_t g_wirelessTailErrCount = 0U;
static volatile uint32_t g_wirelessDuplicateCount = 0U;
static volatile uint32_t g_wirelessIgnoredCount = 0U;
static volatile uint32_t g_wirelessTargetUpdateCount = 0U;
static volatile uint8_t g_wirelessHaveSeq = 0U;
static volatile uint8_t g_wirelessLastSeq = 0U;
static volatile uint8_t g_wirelessTxSeq = 0U;
static volatile uint8_t g_wirelessLastEventCode = 0U;
static volatile uint8_t g_wirelessLastParseStatus = WIRELESS_PARSE_NONE;
static volatile uint32_t g_wirelessLastEventMs = 0U;
static volatile bool g_wirelessEventPending = false;
static volatile WirelessFrame_t g_wirelessLastFrame = {0};
static volatile uint8_t g_wirelessRxWindow[WIRELESS_FRAME_LEN] = {0};
static volatile uint8_t g_wirelessLastRawFrame[WIRELESS_FRAME_LEN] = {0};
static volatile uint8_t g_wirelessLastRawFrameValid = 0U;
static uint32_t g_wirelessW0LastOledMs = 0U;
static uint32_t g_wirelessW0LastLogMs = 0U;
static uint32_t g_wirelessW0LastAutoTxMs = 0U;
static uint32_t g_radarR0LastLogMs = 0U;
static uint32_t g_radarR0LastOledMs = 0U;
static uint32_t g_radarR1LastLogMs = 0U;
static uint32_t g_radarR1LastOfflineLogMs = 0U;
static uint32_t g_radarR1LastOledMs = 0U;
static bool g_radarR1EverArmed = false;
static uint32_t g_radarR1LastArmedMs = 0U;
static bool g_radarR1LogFrozen = false;
static bool g_radarR1RouteBActive = false;
#if MANUAL_TARGET_INPUT_ENABLE
#define UI_MENU_LONG_PRESS_MS 800U
#define UI_RUN_LONG_PRESS_MS 1200U
static UiMode g_uiMode = UI_MODE_HOME;
static UiMenuItem g_uiMenuItem = UI_MENU_COORD;
static CoordField g_coordField = COORD_FIELD_X;
static MapField g_mapField = MAP_FIELD_HOME_X;
static ZoneField g_zoneField = ZONE_FIELD_INDEX;
static TaskField g_taskField = TASK_FIELD_TO_TARGET;
static bool g_coordEditing = false;
static bool g_mapEditing = false;
static bool g_zoneEditing = false;
static bool g_uiCarRunEnabled = false;
static bool g_manualRouteRunActive = false;
static bool g_uiKey1LongHandled = false;
static uint32_t g_uiKey1PressStartMs = 0U;
static bool g_uiKey2LongHandled = false;
static uint32_t g_uiKey2PressStartMs = 0U;
static uint8_t g_manualTargetQr = QR_ID_A;
/*
 * R1 map-planning test target:
 * The intended field route is "car-head forward, then left into map y+".
 * With the current field pose convention that is approximated by a safe
 * QR-center test point around x=130,y=200 cm; y=200 is valid on the real
 * 380 cm x 350 cm map, while x=130 keeps the goal out of the left white block.
 */
static int16_t g_manualTargetX =
    RADAR_R1_FIXED_FIRST_X_CM + RADAR_R1_FIXED_FINAL_X_DELTA_CM;
static int16_t g_manualTargetY = RADAR_R1_FIXED_FINAL_Y_CM;
static int16_t g_manualTargetStep = 10;
static uint32_t g_manualTargetApplyCount = 0U;
static int16_t g_menuStep = 10;
static uint8_t g_zoneEditIndex = 0U;
#endif

static volatile Pose2D g_poseRaw = {0};
static volatile Pose2D g_poseMap = {0};
static volatile TargetPoint g_targetPoint = {0};
static MissionContext g_mission = {0};
static bool g_p7MissionModeEnable = false;
static uint32_t g_arrivedTargetMs = 0U;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
static bool g_radarR1OriginLocked = false;
static Point2D g_radarR1OriginMap = {0};
static int16_t g_radarR1OriginYawDeg = 0;
static bool g_radarR1YawFilterLocked = false;
static int16_t g_radarR1YawFilteredDeg = 0;
static bool g_radarR1PiYawHaveLast = false;
static int16_t g_radarR1PiYawLastRawDeg = 0;
static uint32_t g_radarR1PiYawFallbackUntilMs = 0U;
static int16_t g_radarR1SlewLeft = 0;
static int16_t g_radarR1SlewRight = 0;
static bool g_radarR1TurnSettleActive = false;
static bool g_radarR1TurnSettleDone = false;
static bool g_radarR1PostTurnRampActive = false;
static bool g_radarR1SegmentSettleActive = false;
static bool g_radarR1RouteDone = false;
static volatile bool g_radarR1UrgentYStopPending = false;
static volatile int16_t g_radarR1UrgentYStopPoseY = 0;
static volatile uint32_t g_radarR1UrgentYStopMs = 0U;
static uint32_t g_radarR1TurnSettleUntilMs = 0U;
static uint32_t g_radarR1TurnSettleStableSinceMs = 0U;
static uint32_t g_radarR1PostTurnRampStartMs = 0U;
static uint32_t g_radarR1SegmentSettleUntilMs = 0U;
static int16_t g_radarR1TurnSettleLastX = 0;
static int16_t g_radarR1TurnSettleLastY = 0;
static bool g_radarR1LineErrFilterLocked = false;
static uint8_t g_radarR1LineErrFilterWp = 0U;
static int16_t g_radarR1LineErrFilteredCm = 0;
static bool g_radarR1LineErrAccumLocked = false;
static uint8_t g_radarR1LineErrAccumWp = 0U;
static int16_t g_radarR1LineErrAccumScore = 0;
static int16_t g_radarR1LineErrAccumBiasPwm = 0;
static uint32_t g_radarR1LineErrAccumLastMs = 0U;
static uint8_t g_radarR1OriginStableFrames = 0U;
static uint8_t g_radarR1OriginState = 0U;
static uint32_t g_radarR1OriginStableStartMs = 0U;
static int16_t g_radarR1OriginStableXMin = 0;
static int16_t g_radarR1OriginStableXMax = 0;
static int16_t g_radarR1OriginStableYMin = 0;
static int16_t g_radarR1OriginStableYMax = 0;
static int16_t g_radarR1OriginStableRefYawDeg = 0;
static bool g_radarR1GuardEncoderActive = false;
static int16_t g_radarR1GuardBaseLeftCount = 0;
static int16_t g_radarR1GuardBaseRightCount = 0;
static int16_t g_radarR1GuardForwardCounts = 0;
static uint8_t g_radarR1StopSource = 0U;
static uint8_t g_radarR1ProtectFault = 0U;
#endif
static SpeedLoopStatus g_navSpeedLoop = {0};
static bool g_navSpeedLoopReady = false;
static uint32_t g_navSpeedLoopLastMs = 0U;
static int16_t g_navSpeedLoopPrevLeft = 0;
static int16_t g_navSpeedLoopPrevRight = 0;

typedef struct {
    uint32_t time_ms;
    int32_t raw_x_cm;
    int32_t raw_y_cm;
    int32_t pose_x_cm;
    int32_t pose_y_cm;
    int32_t target_x_cm;
    int32_t target_y_cm;
    int16_t raw_yaw_deg;
    int16_t yaw_deg;
    int16_t desired_yaw_deg;
    int16_t wp_x_cm;
    int16_t wp_y_cm;
    int16_t err_dist_cm;
    int16_t err_yaw_deg;
    int16_t line_err_cm;
    int16_t line_err_acc;
    int16_t line_bias_pwm;
    int16_t cmd_left;
    int16_t cmd_right;
    uint16_t rx_count;
    uint16_t ok_count;
    uint16_t err_count;
    uint16_t pose_age_ms;
    uint16_t gyro_rx_count;
    uint16_t gyro_ok_count;
    uint16_t gyro_err_count;
    uint16_t gyro_age_ms;
    uint32_t turn_settle_remaining_ms;
    uint32_t turn_stable_ms;
    int16_t turn_pose_dx_cm;
    int16_t turn_pose_dy_cm;
    uint8_t turn_settle_active;
    uint8_t turn_settle_done;
    uint8_t post_turn_active;
    uint8_t start_gate;
    int16_t radar_dy_cm;
    int16_t enc_forward_counts;
    int16_t jy_yaw_deg;
    uint8_t stop_source;
    uint8_t protect_fault;
    uint8_t origin_state;
    uint8_t origin_frames;
    int16_t err_x_cm;
    int16_t err_y_cm;
    int16_t speed_target_l;
    int16_t speed_target_r;
    int16_t speed_l;
    int16_t speed_r;
    int16_t speed_sync;
    uint8_t state;
    uint8_t fault;
    uint8_t plan_ok;
    uint8_t plan_fail;
    uint8_t path_len;
    uint8_t waypoint_index;
    uint8_t ctrl_stage;
    uint8_t flags;
    uint8_t mission_leg;
    uint8_t target_source;
    uint8_t forbidden_count;
} RadarR1LogEntry;

static RadarR1LogEntry g_radarR1Logs[RADAR_R1_LOG_CAPACITY] = {0};
static uint8_t g_radarR1LogIndex = 0U;
static uint8_t g_radarR1LogCount = 0U;
static uint16_t g_radarR1LogDropped = 0U;

#if WIRELESS_W0_TEST_ENABLE
static DevStage g_devStage = DEV_STAGE_P7_FULL_MISSION;
#else
static DevStage g_devStage = (MOTOR_DIR_TEST_ENABLE != 0) ?
    DEV_STAGE_C0_MOTOR_DIR : DEV_STAGE_P7_FULL_MISSION;
#endif
static NavState g_navState = NAV_WAIT_TARGET;
static FaultCode g_faultCode = FAULT_NONE;
static uint8_t g_planFailCode = PLAN_FAIL_NONE;
static PathBuffer g_path = {0};
static PlanResult g_lastPlan = {0};
static NavLogEntry g_navLogs[NAV_LOG_CAPACITY] = {0};
static uint8_t g_navLogWriteIndex = 0U;

static uint8_t g_grid[MAP_NODE_COUNT] = {0};
static uint16_t g_cost[MAP_NODE_COUNT] = {0};
static int16_t g_parent[MAP_NODE_COUNT] = {0};
static uint8_t g_nodeState[MAP_NODE_COUNT] = {0};
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
static bool g_radarR1AstarYawSlewLocked = false;
static uint8_t g_radarR1AstarYawSlewWp = 0U;
static int16_t g_radarR1AstarYawSlewDeg = 0;
static uint8_t g_radarR1AstarFineTryCount = 0U;
static uint8_t g_radarR1AstarFinePhase = 0U; /* 0=align, 1=drive */
static uint32_t g_radarR1AstarFineStableSinceMs = 0U;
static uint32_t g_radarR1AstarFineDriveStartMs = 0U;
static uint32_t g_radarR1BTargetFineSinceMs = 0U;
static bool g_radarR1PreReturnFineUsed = false;
#endif

#define SPEED_C4_TEST_ENABLE      1
#define SPEED_C4_OPEN_LOOP_DIAG   0
#define SPEED_C4_PULSE_PWM        800
#define SPEED_C4_PULSE_BASE_L     595
#define SPEED_C4_PULSE_BASE_R     800
#define SPEED_C4_PULSE_RIGHT_MORE_L 590
#define SPEED_C4_PULSE_RIGHT_MORE_R 800
#define SPEED_C4_PULSE_RIGHT_LESS_L 600
#define SPEED_C4_PULSE_RIGHT_LESS_R 800
#define SPEED_C4_PULSE_MS         6000U
#define SPEED_C4_PULSE_GAP_MS     0U
#define SPEED_C4_PULSE_PAIR_COUNT 1U
#define SPEED_C4_TARGET_CPS       800
#define SPEED_C4_TARGET_LOW_CPS   800
#define SPEED_C4_TARGET_HIGH_CPS  900
#define SPEED_C4_BASE_PWM_L       600
#define SPEED_C4_BASE_PWM_R       800
#define SPEED_C4_HIGH_BASE_PWM_L  600
#define SPEED_C4_HIGH_BASE_PWM_R  800
#define SPEED_C4_PWM_MIN          250
#define SPEED_C4_PWM_MAX          1000
#define SPEED_C4_KP_DIV           70
#define SPEED_C4_KI_DIV           0
#define SPEED_C4_INTEGRAL_LIMIT   2400000
#define SPEED_C4_CORRECTION_LIMIT 35
#define SPEED_C4_SPEED_SYNC_DIV   0
#define SPEED_C4_DISTANCE_SYNC_DIV 50
#define SPEED_C4_SYNC_LIMIT       0
#define SPEED_C4_TOTAL_SYNC_LIMIT 25
#define SPEED_C4_FF_SYNC_WEAK_DIV 80
#define SPEED_C4_FF_SYNC_WEAK_LIMIT 25
#define SPEED_C4_FF_SYNC_STRONG_DIV 40
#define SPEED_C4_FF_SYNC_STRONG_LIMIT 45
#define SPEED_C4_ENCODER_R_SCALE_NUM 640
#define SPEED_C4_ENCODER_R_SCALE_DEN 1000
#define SPEED_C4_PWM_STEP_LIMIT   12
#define SPEED_C4_DURATION_MS      2000U
#define SPEED_C4_SAMPLE_MS        200U
#define SPEED_C4_OLED_RUN_MS      2000U
#define SPEED_C4_OLED_STOP_MS     500U
#define SPEED_C5_DISTURB_BASE_MS  1000U
#define SPEED_C5_DISTURB_HOLD_MS  600U
#define SPEED_C5_DISTURB_RECOVER_MS 2400U
#define SPEED_C5_DISTURB_TOTAL_MS (SPEED_C5_DISTURB_BASE_MS + \
                                   SPEED_C5_DISTURB_HOLD_MS + \
                                   SPEED_C5_DISTURB_RECOVER_MS)
#define SPEED_C5_DISTURB_LOW_TARGET_CPS 450
#define SPEED_C5_DISTURB_LOW_PWM_L 340
#define SPEED_C5_DISTURB_LOW_PWM_R 450
#define ENCODER_DIR_L             (-1)
#define ENCODER_DIR_R             (-1)
#define ENCODER_USE_GPIO_IRQ      1
#define ENCODER_GPIOA_PINS        (GPIO_lift_bmg6_C0_PIN | GPIO_lift_bmg6_C1_PIN | \
                                   GPIO_right_bmg7_C0_PIN | GPIO_right_bmg7_C1_PIN)
#define ENCODER_GPIOA_POLARITY    (DL_GPIO_PIN_26_EDGE_RISE_FALL | DL_GPIO_PIN_27_EDGE_RISE_FALL | \
                                   DL_GPIO_PIN_29_EDGE_RISE_FALL | DL_GPIO_PIN_30_EDGE_RISE_FALL)

static bool g_key4LastPressed = false;
static bool g_key3LastPressed = false;
static bool g_key2LastPressed = false;
static bool g_key1LastPressed = false;
static int16_t g_motorTestPwm = 300;
#if !SPEED_C4_TEST_ENABLE
static int16_t g_motionTestTrim = 0;
#endif
#if !SPEED_C4_TEST_ENABLE
static uint32_t g_motorLogLastMs = 0U;
static uint32_t g_motorLogLastStep = 0xFFFFFFFFU;
static bool g_motionTestPulse = false;
static uint32_t g_motionTestStartMs = 0U;
#endif
#if !SPEED_C4_TEST_ENABLE
static uint32_t g_motionTestRunId = 0U;
#endif

#define MOTION_C35_BASE_PWM       300
#define MOTION_C35_TRIM_STEP      20
#define MOTION_C35_TRIM_MIN       (-200)
#define MOTION_C35_TRIM_MAX       200
#define MOTION_C35_DURATION_MS    1000U
#define MOTION_C35_LOG_CAPACITY   192U

typedef enum {
    MOTION_C35_EVENT_BOOT = 0,
    MOTION_C35_EVENT_START,
    MOTION_C35_EVENT_STOP,
    MOTION_C35_EVENT_TRIM,
    MOTION_C35_EVENT_DIR,
    MOTION_C35_EVENT_RUN,
    MOTION_C35_EVENT_EXPORT
} MotionC35Event;

typedef struct {
    uint32_t time_ms;
    uint32_t run_id;
    int16_t base_pwm;
    int16_t trim;
    int16_t pwm_l;
    int16_t pwm_r;
    uint16_t duration_ms;
    int16_t enc_l_delta;
    int16_t enc_r_delta;
    int16_t speed_l;
    int16_t speed_r;
    uint8_t left_ab;
    uint8_t right_ab;
    MotionC35Event event;
} MotionC35LogEntry;

#if !SPEED_C4_TEST_ENABLE
static MotionC35LogEntry g_motionC35Logs[MOTION_C35_LOG_CAPACITY] = {0};
static uint8_t g_motionC35LogIndex = 0U;
static uint8_t g_motionC35LogCount = 0U;
static uint16_t g_motionC35LogDropped = 0U;
#endif

typedef enum {
    SPEED_C4_MODE_NONE = 0,
    SPEED_C4_MODE_LEFT,
    SPEED_C4_MODE_RIGHT,
    SPEED_C4_MODE_BOTH
} SpeedC4Mode;

typedef enum {
    SPEED_C4_EVENT_BOOT = 0,
    SPEED_C4_EVENT_TARGET,
    SPEED_C4_EVENT_START,
    SPEED_C4_EVENT_PHASE,
    SPEED_C4_EVENT_RUN,
    SPEED_C4_EVENT_STOP,
    SPEED_C4_EVENT_EXPORT
} SpeedC4Event;

typedef enum {
    SPEED_C4_SEQUENCE_NONE = 0,
    SPEED_C4_SEQUENCE_BASE_PAIR,
    SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR,
    SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR
} SpeedC4Sequence;

typedef enum {
    SPEED_C4_CONTROL_OPEN = 0,
    SPEED_C4_CONTROL_SPEED_P,
    SPEED_C4_CONTROL_SPEED_P_DIST,
    SPEED_C4_CONTROL_DISTURB_LEFT,
    SPEED_C4_CONTROL_DISTURB_RIGHT
} SpeedC4Control;

typedef enum {
    SPEED_C4_PHASE_IDLE = 0,
    SPEED_C4_PHASE_BASE,
    SPEED_C4_PHASE_DISTURB,
    SPEED_C4_PHASE_RECOVER
} SpeedC4Phase;

typedef struct {
    uint32_t time_ms;
    uint32_t run_id;
    uint8_t pulse_index;
    uint8_t pulse_total;
    uint16_t elapsed_ms;
    SpeedC4Mode mode;
    SpeedC4Control control;
    SpeedC4Phase phase;
    int16_t target_l;
    int16_t target_r;
    int16_t speed_l;
    int16_t speed_r;
    int16_t err_l;
    int16_t err_r;
    int16_t pwm_l;
    int16_t pwm_r;
    int16_t count_l;
    int16_t count_r;
    int16_t sync;
    int16_t norm_err;
    int16_t yaw_delta;
    int16_t i_l;
    int16_t i_r;
    uint8_t sat_l;
    uint8_t sat_r;
    uint8_t left_ab;
    uint8_t right_ab;
    SpeedC4Event event;
} SpeedC4LogEntry;

static SpeedC4LogEntry g_speedC4Logs[MOTION_C35_LOG_CAPACITY] = {0};
static uint8_t g_speedC4LogIndex = 0U;
static uint8_t g_speedC4LogCount = 0U;
static uint16_t g_speedC4LogDropped = 0U;
static bool g_speedC4Run = false;
static SpeedC4Mode g_speedC4Mode = SPEED_C4_MODE_NONE;
static uint32_t g_speedC4StartMs = 0U;
static uint32_t g_speedC4LastSampleMs = 0U;
static uint32_t g_speedC4LastOledMs = 0U;
static uint32_t g_speedC4RunId = 0U;
static int16_t g_speedC4PrevLeft = 0;
static int16_t g_speedC4PrevRight = 0;
static int16_t g_speedC4PwmLeft = 0;
static int16_t g_speedC4PwmRight = 0;
static int16_t g_speedC4TargetCps = SPEED_C4_TARGET_LOW_CPS;
static SpeedC4Control g_speedC4Control = SPEED_C4_CONTROL_SPEED_P_DIST;
static SpeedC4Phase g_speedC4Phase = SPEED_C4_PHASE_IDLE;
static int16_t g_speedC4LastSyncCorrection = 0;
static int16_t g_speedC4YawStartDeg = 0;
static int32_t g_speedC4IntegralLeft = 0;
static int32_t g_speedC4IntegralRight = 0;
#if SPEED_C4_OPEN_LOOP_DIAG
static int16_t g_speedC4OpenPwmLeft = SPEED_C4_BASE_PWM_L;
static int16_t g_speedC4OpenPwmRight = SPEED_C4_BASE_PWM_R;
static SpeedC4Sequence g_speedC4Sequence = SPEED_C4_SEQUENCE_NONE;
static bool g_speedC4PulseGap = false;
static uint8_t g_speedC4PulseIndex = 0U;
static uint8_t g_speedC4PulseTotal = 0U;
static uint32_t g_speedC4PulseGapStartMs = 0U;
#endif

#if RADAR_R1_NAV_TEST_ENABLE
static void DebugCommand_Update(uint32_t nowMs);
#endif
static char SpeedC4_ModeChar(SpeedC4Mode mode);
static char SpeedC4_ControlChar(SpeedC4Control control);
static char SpeedC4_PhaseChar(SpeedC4Phase phase);
static void Motor_SetOutput(int16_t left, int16_t right);
static void Motor_SetOutputDry(int16_t left, int16_t right);
static void Motor_Stop(void);
static void NavSpeedLoop_Reset(uint32_t nowMs);
static void NavSpeedLoop_SetTargetFromPwm(int16_t leftCmd, int16_t rightCmd);
static void NavSpeedLoop_Update(uint32_t nowMs, bool motorEnable);
static const SpeedLoopStatus *NavSpeedLoop_GetStatus(void);
static void AppendLog(uint32_t nowMs, int16_t errDistCm, int16_t errYawDeg);
static void WirelessW0_Update(uint32_t nowMs);
static void WirelessW0_PrintHeader(void);
static void WirelessW0_ShowOled(uint32_t nowMs);
static void WirelessW0_SendSample(uint32_t nowMs, uint8_t qrId, int16_t xCm, int16_t yCm);
static void RadarR0_Update(uint32_t nowMs);
static void RadarR0_PrintHeader(void);
static void RadarR0_ShowOled(uint32_t nowMs);
static void RadarR1_Update(uint32_t nowMs);
static void RadarR1_PrintHeader(void);
static void RadarR1_PrintRawLog(uint32_t nowMs);
static void RadarR1_ShowOled(uint32_t nowMs, bool forceAll);
static void RadarR1_OfflineLogAppend(uint32_t nowMs);
static void RadarR1_OfflineLogReset(void);
static void RadarR1_OfflineLogExport(uint32_t nowMs);
static void RadarR1_ResetAutoOrigin(void);
static void Navigation_ResetPlan(void);
static uint8_t WirelessW0_NormalizeQrId(uint8_t qrId);
static bool RadarR1_BuildFixedRoute(PathBuffer *path);
static void Navigation_Update(uint32_t nowMs);
static void Mission_InitDefaults(uint32_t nowMs);
static void Mission_UpdatePoseTargetError(void);
static bool Mission_ConfigIsValid(void);
static Point2D Mission_CurrentGoal(void);
#if MANUAL_TARGET_INPUT_ENABLE
static void Ui_Update(uint32_t nowMs, bool key4Pressed, bool key3Pressed,
                      bool key2Pressed, bool key1Pressed);
static void Ui_ShowOled(uint32_t nowMs);
static void ManualTarget_Apply(uint32_t nowMs, bool sendFrame);
#endif
static void WirelessUart_Force115200(void);
static void RadarUart_Force115200(void);
static void Jy901_ResetYawOriginToCurrent(void);

static const RectObstacle g_obstacles[] = {
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    /*
     * 2026-05-27 measured field forbidden zones.
     *
     * The current planner stores axis-aligned RectObstacle, so each measured
     * slanted rectangle is represented by the tight bounding box of its measured
     * edge endpoints.  A* expands each measured box by 8 cm so the planned path
     * prefers the safe corridor between forbidden zones instead of touching the
     * measured edge; the protect layer then slows near that planned boundary.
     *
     * Three-zone real-field A-code test.
     * Hard no-go margin is RADAR_R1_FORBID_PLAN_MARGIN_CM=5cm; the outer
     * RADAR_R1_FORBID_SOFT_COST_DIST_CM=50cm band remains passable but costly.
     * The soft-cost function discounts overlap/corridor areas affected by
     * multiple zones, so the middle passage is not punished more than a single
     * obstacle's outside edge.
     *
     * Front/right:
     *   width edge:  (+119,-29) -> (+213,-22)
     *   length edge: (+218,-23) -> (+212,+132)
     *   AABB: x=+119..+218, y=-29..+132
     *
     * Left/rear:
     *   length edge: (-30,+77) -> (-38,+239)
     *   width edge:  (-44,+242) -> (-146,+232)
     *   AABB: x=-146..-30, y=+77..+242
     *
     * Left/front:
     *   width edge:  (+43,+247) -> (+37,+347)
     *   length edge: (+33,+354) -> (+184,+356)
     *   AABB: x=+33..+184, y=+247..+356
     */
    {119, -29, 218, 132},
    {-146, 77, -30, 242},
    {33, 247, 184, 356},
#else
    {-20, 80, 90, 220},
    {160, 210, 330, 305},
    {220, -20, 330, 120},
#endif
};

static void Mission_InitDefaults(uint32_t nowMs)
{
    uint8_t i;
    uint8_t staticCount = (uint8_t) (sizeof(g_obstacles) / sizeof(g_obstacles[0]));

    memset(&g_mission, 0, sizeof(g_mission));
    g_mission.leg = MISSION_LEG_NONE;
    g_mission.target_source = TARGET_SOURCE_NONE;
    g_mission.map.x_min_cm = MAP_X_MIN_CM;
    g_mission.map.y_min_cm = MAP_Y_MIN_CM;
    g_mission.map.x_max_cm = MAP_X_MAX_CM;
    g_mission.map.y_max_cm = MAP_Y_MAX_CM;
    g_mission.map.boundary_margin_cm = MAP_BOUNDARY_MARGIN_CM;
    #if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    /* Keep a 5 cm planning clearance around the measured forbidden boxes.
     * This makes A* avoid the real forbidden edge instead of merely slowing
     * after it is already close.  RADAR_R1_FORBID_PROTECT_DIST_CM then adds
     * a low-speed band outside this planned no-go clearance. */
    g_mission.map.obstacle_margin_cm = RADAR_R1_FORBID_PLAN_MARGIN_CM;
#else
    g_mission.map.obstacle_margin_cm = MAP_OBSTACLE_MARGIN_CM;
#endif
    g_mission.map.home_map_frame.x_cm = MAP_HOME_X_CM;
    g_mission.map.home_map_frame.y_cm = MAP_HOME_Y_CM;
    g_mission.map.home_yaw_deg = MAP_HOME_YAW_DEG;
    g_mission.map.valid = (MAP_CONFIG_DEFAULT_VALID != 0);
    g_mission.map.stamp_ms = nowMs;
    g_mission.home_valid = true;

    g_mission.forbidden_count =
        (staticCount > MAP_FORBIDDEN_MAX) ? MAP_FORBIDDEN_MAX : staticCount;
    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        g_mission.forbidden[i].rect = g_obstacles[i];
        g_mission.forbidden[i].source = 0U; /* 0 = compiled default */
        g_mission.forbidden[i].valid = true;
    }
}

static void Mission_UpdatePoseTargetError(void)
{
    g_mission.pose_actual_map.x_cm = g_poseMap.x_cm;
    g_mission.pose_actual_map.y_cm = g_poseMap.y_cm;
    g_mission.target_expected_map = g_targetPoint.target_map_frame;
    g_mission.control_error_cm.x_cm =
        g_mission.target_expected_map.x_cm - g_mission.pose_actual_map.x_cm;
    g_mission.control_error_cm.y_cm =
        g_mission.target_expected_map.y_cm - g_mission.pose_actual_map.y_cm;
}

static bool Mission_ConfigIsValid(void)
{
    if (!g_mission.map.valid || !g_mission.home_valid) {
        return false;
    }
    if ((g_mission.map.x_min_cm >= g_mission.map.x_max_cm) ||
        (g_mission.map.y_min_cm >= g_mission.map.y_max_cm)) {
        return false;
    }
    if ((g_mission.map.boundary_margin_cm < 0) ||
        (g_mission.map.obstacle_margin_cm < 0)) {
        return false;
    }
    if ((g_mission.map.home_map_frame.x_cm < g_mission.map.x_min_cm) ||
        (g_mission.map.home_map_frame.x_cm > g_mission.map.x_max_cm) ||
        (g_mission.map.home_map_frame.y_cm < g_mission.map.y_min_cm) ||
        (g_mission.map.home_map_frame.y_cm > g_mission.map.y_max_cm)) {
        return false;
    }
    return true;
}

static Point2D Mission_CurrentGoal(void)
{
    if (g_mission.leg == MISSION_LEG_TO_HOME) {
        return g_mission.map.home_map_frame;
    }
    return g_targetPoint.target_map_frame;
}

static int16_t Jy901_ReadInt16Le(const volatile uint8_t *buf, uint8_t lowIndex)
{
    uint16_t value = (uint16_t) buf[lowIndex] |
                     ((uint16_t) buf[(uint8_t) (lowIndex + 1U)] << 8);
    return (int16_t) value;
}

static int16_t Jy901_RawAngleToDeg(int16_t raw)
{
    return NormalizeDeg((int16_t) (((int32_t) raw * 180L) / 32768L));
}

static void Jy901_ResetYawOriginToCurrent(void)
{
#if JY901_ENABLE
    if (g_jy901Valid) {
        g_jy901YawOriginDeg = g_jy901YawRawDeg;
        g_jy901YawOriginLocked = true;
    } else {
        /*
         * PB7 can be pressed before the first JY901 frame.  Still force the
         * control/display yaw to zero now; the first valid frame will lock the
         * raw yaw origin and continue from the same PB7-relative convention.
         */
        g_jy901YawOriginDeg = g_jy901YawRawDeg;
        g_jy901YawOriginLocked = false;
    }
    g_jy901YawDeg = 0;
    g_poseMap.yaw_deg = 0;
#endif
}

static bool Jy901_IsFresh(uint32_t nowMs)
{
#if JY901_ENABLE
    return g_jy901Valid && ((g_jy901StampMs != 0U) &&
                            ((nowMs - g_jy901StampMs) <= JY901_TIMEOUT_MS));
#else
    (void) nowMs;
    return true;
#endif
}

static void Jy901_ProcessAngleFrame(void)
{
#if JY901_ENABLE
    int16_t rawYaw = Jy901_ReadInt16Le(g_jy901RxBuf, 6U);
    int16_t yawAbs = Jy901_RawAngleToDeg(rawYaw);
    int16_t yawRel;

    g_jy901YawRawDeg = yawAbs;
    if (!g_jy901YawOriginLocked) {
        g_jy901YawOriginDeg = yawAbs;
        g_jy901YawOriginLocked = true;
    }
    yawRel = NormalizeDeg((int16_t) ((yawAbs - g_jy901YawOriginDeg) *
                                     JY901_YAW_SIGN));
    g_jy901YawDeg = yawRel;
    g_jy901Valid = true;
    g_jy901StampMs = g_tickMs;

    /*
     * Do not write g_poseRaw here.  g_poseRaw is now reserved for the exact
     * Raspberry Pi radar UART output shown on OLED: raw x, raw y and raw yaw.
     * JY901 yaw may still be used for g_poseMap/control below.
     */
#if JY901_USE_YAW_FOR_R1 && !RADAR_R1_USE_BRIDGE_YAW_ONLY
    if (g_poseMap.valid) {
#if RADAR_R1_JY_YAW_RELATIVE_TO_RADAR_ORIGIN
        if (g_radarR1OriginLocked) {
            g_poseMap.yaw_deg = NormalizeDeg((int16_t)
                (yawRel - g_radarR1OriginYawDeg));
        } else {
            g_poseMap.yaw_deg = yawRel;
        }
#else
        g_poseMap.yaw_deg = yawRel;
#endif
    }
#endif
#endif
}

static void Jy901_ParseByte(uint8_t byte)
{
#if JY901_ENABLE
    uint8_t i;
    uint8_t sum = 0U;

    if (g_jy901RxIndex == 0U) {
        if (byte != JY901_HEADER) {
            return;
        }
        g_jy901RxBuf[0] = byte;
        g_jy901RxIndex = 1U;
        return;
    }

    if ((g_jy901RxIndex == 1U) && (byte == JY901_HEADER)) {
        g_jy901RxBuf[0] = byte;
        return;
    }

    g_jy901RxBuf[g_jy901RxIndex++] = byte;
    if (g_jy901RxIndex < JY901_FRAME_LEN) {
        return;
    }

    for (i = 0U; i < (JY901_FRAME_LEN - 1U); ++i) {
        sum = (uint8_t) (sum + g_jy901RxBuf[i]);
    }
    if (sum == g_jy901RxBuf[JY901_FRAME_LEN - 1U]) {
        if (g_jy901RxBuf[1] == JY901_ANGLE_FRAME) {
            ++g_jy901FrameOkCount;
            Jy901_ProcessAngleFrame();
        }
    } else {
        ++g_jy901FrameErrCount;
    }
    g_jy901RxIndex = 0U;
#else
    (void) byte;
#endif
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
static bool RadarR1_FixedRouteNeedsTurnSettle(uint8_t waypointIndex);
static bool RadarR1_FixedRouteIsRightLeg(uint8_t waypointIndex);
static bool RadarR1_FixedRouteIsReturnXLeg(uint8_t waypointIndex);
static bool RadarR1_FixedRouteIsReturnHomeLeg(uint8_t waypointIndex);
static int16_t RadarR1_FixedRouteAxisDistance(uint8_t waypointIndex,
                                              int32_t dx,
                                              int32_t dy);

static int16_t RadarR1_FixedRouteLineError(uint8_t waypointIndex,
                                           int32_t dx,
                                           int32_t dy)
{
    if (RadarR1_FixedRouteIsRightLeg(waypointIndex)) {
        int16_t clampCm = RadarR1_FixedRouteIsReturnXLeg(waypointIndex) ?
            RADAR_R1_RETURN_X_LINEERR_CLAMP_CM :
            RADAR_R1_RIGHT_LEG_LINEERR_CLAMP_CM;
        /*
         * Final X leg: drive along X from the post-turn pose.  The
         * latest field log shows radar Y can jump about 50 cm during the
         * final rotation while physical Y has not moved that much.  Keep
         * the X leg from steering hard on that yaw-dependent radar-Y jump;
         * only allow a small +/-5 cm line error for gentle correction.
         */
        return (int16_t) Clamp32(dy, -clampCm, clampCm);
    }
    if (waypointIndex >= 1U) {
        /*
         * +Y leg: waypoint.x is the post-turn baseline.  dx is therefore the
         * cross-track error to keep within about +/-5 cm while dy decreases.
         */
        return (int16_t) Clamp32(dx, -32768, 32767);
    }
    return 0;
}

static bool RadarR1_PostTurnYawOnlyActive(uint32_t nowMs)
{
    return (g_radarR1PostTurnRampActive &&
            ((nowMs - g_radarR1PostTurnRampStartMs) <
             RADAR_R1_POST_TURN_YAW_ONLY_MS));
}

static int16_t RadarR1_FilterLineErrForControl(uint8_t waypointIndex,
                                               int16_t lineErrCm,
                                               uint32_t nowMs)
{
    int32_t delta;
    int32_t step;

    if ((waypointIndex < 2U) ||
        !g_radarR1TurnSettleDone ||
        RadarR1_PostTurnYawOnlyActive(nowMs) ||
        !g_radarR1LineErrFilterLocked ||
        (g_radarR1LineErrFilterWp != waypointIndex)) {
        g_radarR1LineErrFilterLocked = true;
        g_radarR1LineErrFilterWp = waypointIndex;
        g_radarR1LineErrFilteredCm = lineErrCm;
        return lineErrCm;
    }

    delta = (int32_t) lineErrCm - g_radarR1LineErrFilteredCm;
    step = delta / RADAR_R1_LINEERR_FILTER_DIV;
    if ((step == 0) && (delta != 0)) {
        step = (delta > 0) ? 1 : -1;
    }
    g_radarR1LineErrFilteredCm = (int16_t) Clamp32(
        (int32_t) g_radarR1LineErrFilteredCm + step, -32768, 32767);
    return g_radarR1LineErrFilteredCm;
}

static void RadarR1_ResetLineErrAccum(void)
{
    g_radarR1LineErrAccumLocked = false;
    g_radarR1LineErrAccumWp = 0U;
    g_radarR1LineErrAccumScore = 0;
    g_radarR1LineErrAccumBiasPwm = 0;
    g_radarR1LineErrAccumLastMs = 0U;
}

static int16_t RadarR1_LineErrAccumSpeedBiasPwm(uint8_t waypointIndex,
                                                int16_t lineErrCm,
                                                int16_t errYawDeg,
                                                uint32_t nowMs)
{
    int16_t absLine;
    int32_t score;
    int32_t bias;

    if ((waypointIndex < 2U) ||
        !g_radarR1TurnSettleDone ||
        RadarR1_PostTurnYawOnlyActive(nowMs)) {
        RadarR1_ResetLineErrAccum();
        return 0;
    }

    if (!g_radarR1LineErrAccumLocked ||
        (g_radarR1LineErrAccumWp != waypointIndex)) {
        g_radarR1LineErrAccumLocked = true;
        g_radarR1LineErrAccumWp = waypointIndex;
        g_radarR1LineErrAccumScore = 0;
        g_radarR1LineErrAccumBiasPwm = 0;
        g_radarR1LineErrAccumLastMs = nowMs;
        return 0;
    }

    if ((nowMs - g_radarR1LineErrAccumLastMs) <
        RADAR_R1_LINEERR_ACC_SAMPLE_MS) {
        return g_radarR1LineErrAccumBiasPwm;
    }
    g_radarR1LineErrAccumLastMs = nowMs;

    absLine = Abs16(lineErrCm);
    score = ((int32_t) g_radarR1LineErrAccumScore *
             RADAR_R1_LINEERR_ACC_LEAK_NUM) /
            RADAR_R1_LINEERR_ACC_LEAK_DEN;

    /*
     * This is intentionally not a normal integral.  Small errors only leak the
     * stored score and output 0; only a persistent, same-direction line error
     * while yaw is already close to the segment heading may add a few PWM of
     * wheel-speed split.  The clamp makes it self-limited and prevents the
     * S-curve from becoming integral windup.
     */
    if ((absLine >= RADAR_R1_LINEERR_ACC_START_CM) &&
        (Abs16(errYawDeg) <= RADAR_R1_LINEERR_ACC_YAW_MAX_DEG)) {
        int32_t input = (int32_t) lineErrCm / RADAR_R1_LINEERR_ACC_INPUT_DIV;
        if ((input == 0) && (lineErrCm != 0)) {
            input = (lineErrCm > 0) ? 1 : -1;
        }
        score += input;
    }

    score = Clamp32(score,
                    -RADAR_R1_LINEERR_ACC_LIMIT,
                    RADAR_R1_LINEERR_ACC_LIMIT);
    g_radarR1LineErrAccumScore = (int16_t) score;

    if (absLine < RADAR_R1_LINEERR_ACC_DEADBAND_CM) {
        g_radarR1LineErrAccumBiasPwm = 0;
        return 0;
    }

    bias = score / RADAR_R1_LINEERR_ACC_PWM_DIV;
    bias = Clamp32(bias,
                   -RADAR_R1_LINEERR_ACC_MAX_PWM,
                   RADAR_R1_LINEERR_ACC_MAX_PWM);
    g_radarR1LineErrAccumBiasPwm = (int16_t) bias;
    return g_radarR1LineErrAccumBiasPwm;
}

static bool RadarR1_FixedRouteNeedsTurnSettle(uint8_t waypointIndex)
{
    return (waypointIndex == 1U);
}

static bool RadarR1_RuntimeBActive(void)
{
    return g_radarR1RouteBActive;
}

static bool RadarR1_FixedRouteIsFinalTurn(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive()) {
        return (waypointIndex == 2U);
    }
    return (waypointIndex == 2U) ||
           ((RADAR_R1_RETURN_CORNER60_ENABLE != 0) &&
            ((waypointIndex == 4U) || (waypointIndex == 5U) ||
             (waypointIndex == 7U)));
}

static bool RadarR1_FixedRouteIsReturnXLeg(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive()) {
        return false;
    }
    return (RADAR_R1_RETURN_CORNER60_ENABLE != 0) && (waypointIndex == 6U);
}

static bool RadarR1_FixedRouteIsReturnTurn(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive()) {
        return false;
    }
    return (RADAR_R1_RETURN_CORNER60_ENABLE != 0) &&
           ((waypointIndex == 4U) || (waypointIndex == 5U) ||
            (waypointIndex == 7U));
}

static bool RadarR1_FixedRouteIsReturnHomeLeg(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive()) {
        return false;
    }
    return (RADAR_R1_RETURN_CORNER60_ENABLE != 0) && (waypointIndex == 8U);
}

static bool RadarR1_FixedRouteIsRightLeg(uint8_t waypointIndex)
{
    return (waypointIndex == 3U) || RadarR1_FixedRouteIsReturnXLeg(waypointIndex);
}

static int16_t RadarR1_FixedRouteTurnYaw(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive() && (waypointIndex == 2U)) {
        return RADAR_R1_B_FIXED_FINAL_TURN_YAW_DEG;
    }
    if ((RADAR_R1_RETURN_CORNER60_ENABLE != 0) && (waypointIndex == 4U)) {
        return RADAR_R1_RETURN_TURN1_YAW_DEG;
    }
    if ((RADAR_R1_RETURN_CORNER60_ENABLE != 0) && (waypointIndex == 5U)) {
        return RADAR_R1_RETURN_TURN2_YAW_DEG;
    }
    if ((RADAR_R1_RETURN_CORNER60_ENABLE != 0) && (waypointIndex == 7U)) {
        return RADAR_R1_RETURN_HOME_TURN_YAW_DEG;
    }
    return RADAR_R1_FIXED_FINAL_TURN_YAW_DEG;
}

static int16_t RadarR1_FixedRouteXDeltaCm(uint8_t waypointIndex)
{
    if (RadarR1_RuntimeBActive() && (waypointIndex == 3U)) {
        return (int16_t) (RADAR_R1_B_FIXED_FINAL_X_SIGN *
                          RADAR_R1_B_RUNTIME_X_ENCODER_CAP_CM);
    }
    if (RadarR1_FixedRouteIsReturnXLeg(waypointIndex)) {
        return RADAR_R1_RETURN_X_DELTA_CM;
    }
    return RADAR_R1_FIXED_FINAL_X_DELTA_CM;
}

static int16_t RadarR1_FixedRouteStraightYaw(uint8_t waypointIndex)
{
    if (RadarR1_FixedRouteIsReturnXLeg(waypointIndex)) {
        return RADAR_R1_RETURN_TURN2_YAW_DEG;
    }
    if (RadarR1_FixedRouteIsReturnHomeLeg(waypointIndex)) {
        return RADAR_R1_RETURN_HOME_TURN_YAW_DEG;
    }
    if (RadarR1_FixedRouteIsRightLeg(waypointIndex)) {
        if (RadarR1_RuntimeBActive()) {
            return RADAR_R1_B_FIXED_FINAL_TURN_YAW_DEG;
        }
        return RADAR_R1_FIXED_FINAL_TURN_YAW_DEG;
    }
    if (waypointIndex == 1U) {
        return (int16_t) (90 + RADAR_R1_FIRST_TURN_EXTRA_DEG);
    }
    return 90;
}

static int16_t RadarR1_FixedRouteAxisDistance(uint8_t waypointIndex,
                                              int32_t dx,
                                              int32_t dy)
{
    if (RadarR1_FixedRouteIsFinalTurn(waypointIndex)) {
        (void) dx;
        (void) dy;
        return 0;
    }
    if (RadarR1_FixedRouteIsRightLeg(waypointIndex)) {
        (void) dy;
        return (int16_t) Abs32(dx);
    }
    if (waypointIndex >= 1U) {
        return (int16_t) Abs32(dy);
    }
    return ApproxDistanceCm(dx, dy);
}

static int16_t RadarR1_FixedRouteDesiredYaw(uint8_t waypointIndex,
                                            int16_t lineErrCm,
                                            int16_t currentYawDeg,
                                            uint32_t nowMs)
{
    if (RadarR1_FixedRouteIsFinalTurn(waypointIndex)) {
        (void) lineErrCm;
        (void) currentYawDeg;
        (void) nowMs;
        return RadarR1_FixedRouteTurnYaw(waypointIndex);
    }
    if (waypointIndex >= 1U) {
        int16_t baseYaw = RadarR1_FixedRouteStraightYaw(waypointIndex);
        int16_t baseYawErr = NormalizeDeg((int16_t) (baseYaw - currentYawDeg));
        /*
         * Before the mandatory post-turn settle is complete, align to the pure
         * segment heading.  Do not let line_err pull the heading while the
         * radar/map is still settling after a turn.
         */
        if (!g_radarR1TurnSettleDone ||
            RadarR1_PostTurnYawOnlyActive(nowMs) ||
            (Abs16(baseYawErr) > RADAR_R1_CROSSTRACK_ENABLE_YAW_DEG)) {
            return baseYaw;
        }
        /*
         * Once the straight leg has actually started, add a small cross-track
         * heading bias.  On y+ legs, positive line_err means the target line is
         * on x+ side, so bias below 90 deg.  On +X and -Y return legs the sign
         * is reversed: positive line_err needs a heading above the base yaw.
         */
        int32_t gainDiv = RADAR_R1_CROSSTRACK_GAIN_DIV;
        int32_t maxCorrDeg = RADAR_R1_CROSSTRACK_MAX_DEG;
        int32_t lineCorrDeg;
        if (RadarR1_FixedRouteIsReturnHomeLeg(waypointIndex)) {
            gainDiv = RADAR_R1_RETURN_HOME_CROSSTRACK_GAIN_DIV;
            maxCorrDeg = RADAR_R1_RETURN_HOME_CROSSTRACK_MAX_DEG;
        } else if (RadarR1_FixedRouteIsReturnXLeg(waypointIndex)) {
            gainDiv = RADAR_R1_RETURN_X_CROSSTRACK_GAIN_DIV;
            maxCorrDeg = RADAR_R1_RETURN_X_CROSSTRACK_MAX_DEG;
        }
        lineCorrDeg = Clamp32((int32_t) lineErrCm / gainDiv,
                              -maxCorrDeg, maxCorrDeg);
        if ((baseYaw == 0) || (baseYaw == -90)) {
            return NormalizeDeg((int16_t) (baseYaw + lineCorrDeg));
        }
        return NormalizeDeg((int16_t) (baseYaw - lineCorrDeg));
    }
    return 0;
}
#endif

#if RADAR_R1_NAV_TEST_ENABLE
static uint8_t RadarR1_CtrlStage(int16_t errDistCm, int16_t errYawDeg)
{
#if RADAR_R1_FIXED_ROUTE_TEST
    if (g_radarR1TurnSettleActive) {
        return 4U; /* turn aligned; stopped for map/pose settle */
    }
    if (g_radarR1SegmentSettleActive) {
        return 5U; /* reached a 30 cm y segment; stopped for sensor snapshot */
    }
    if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
        (g_path.current_index >= 1U) &&
        g_radarR1TurnSettleDone &&
        (Abs16(errYawDeg) > RADAR_R1_STRAIGHT_HEADING_CORRECT_DEG)) {
        return 1U; /* heading correction during the y+ straight leg */
    }
#endif
    if (errDistCm <= WAYPOINT_REACH_CM) {
        return 3U; /* reached / switching waypoint */
    }
    if (Abs16(errYawDeg) > RADAR_R1_ALIGN_DEG) {
        return 1U; /* slow turn in place before the next straight segment */
    }
    return 2U;     /* straight/arc drive toward current waypoint */
}
#endif

#if RADAR_R1_NAV_TEST_ENABLE
static void DebugCommand_Update(uint32_t nowMs)
{
    uint8_t ch;
    uint8_t handled = 0U;

    while ((handled < 8U) && DebugUart_TryReadChar(&ch)) {
        handled++;
        /*
         * Field fallback: PA11 debug RX has no historical command parser.
         * Accept one-byte commands so the log can be exported even if PB6
         * release/menu state is inconvenient during map testing.
         *
         *   E/e : export nav_r1 offline log
         *   D/d : dump/export nav_r1 offline log
         *   ?   : print command help
         */
        if ((ch == 'E') || (ch == 'e') || (ch == 'D') || (ch == 'd')) {
            if (g_uiCarRunEnabled) {
                DebugUart_WriteString("nav_r1.cmd busy running\r\n");
            } else {
                DebugUart_WriteString("nav_r1.cmd export\r\n");
                RadarR1_OfflineLogExport(nowMs);
            }
        } else if (ch == '?') {
            DebugUart_WriteString("nav_r1.cmd help: send E or D to export offline log; PB6 short-home also exports\r\n");
        }
    }
}
#endif

static Point2D TransformPoint(Point2D rawPoint, int32_t swapAxes, int32_t xSign,
                              int32_t ySign, int32_t xOffsetCm, int32_t yOffsetCm)
{
    Point2D point = rawPoint;
    int32_t tmp;

    if (swapAxes != 0) {
        tmp = point.x_cm;
        point.x_cm = point.y_cm;
        point.y_cm = tmp;
    }

    point.x_cm = point.x_cm * xSign + xOffsetCm;
    point.y_cm = point.y_cm * ySign + yOffsetCm;
    return point;
}

static Point2D GetQrFallbackPoint(uint8_t qrType)
{
    Point2D point = {85, 0};

    switch (qrType) {
    case QR_TYPE_A:
        point.x_cm = (int16_t) (RADAR_R1_FIXED_FIRST_X_CM +
            RADAR_R1_A_FIXED_FINAL_X_SIGN * RADAR_R1_A_FIXED_FINAL_X_CM);
        point.y_cm = RADAR_R1_A_FIXED_FINAL_Y_CM;
        break;
    case QR_TYPE_B:
        point.x_cm = (int16_t) (RADAR_R1_FIXED_FIRST_X_CM +
            RADAR_R1_B_FIXED_FINAL_X_SIGN * RADAR_R1_B_RUNTIME_FINAL_X_CM);
        point.y_cm = RADAR_R1_B_RUNTIME_FINAL_Y_CM;
        break;
    case QR_TYPE_C:
        point.x_cm = RADAR_R1_C_ASTAR_TARGET_X_CM;
        point.y_cm = RADAR_R1_C_ASTAR_TARGET_Y_CM;
        break;
    default:
        break;
    }
    return point;
}

static Point2D WirelessTargetMapPoint(uint8_t qrId, Point2D rawPoint)
{
    Point2D mapPoint = TransformPoint(rawPoint, TARGET_SWAP_AXES, TARGET_X_SIGN,
                                      TARGET_Y_SIGN, TARGET_X_OFFSET_CM,
                                      TARGET_Y_OFFSET_CM);

    if ((rawPoint.x_cm == 0) && (rawPoint.y_cm == 0)) {
        return GetQrFallbackPoint(qrId);
    }
    if (qrId == QR_ID_A) {
        /*
         * Keep the airplane-origin offset conversion first, then apply the
         * same field tuning as the manual A route: y260 -> y265 to start the
         * return a little farther from the forbidden band.
         */
        mapPoint.y_cm += RADAR_R1_WIRELESS_A_TARGET_Y_BIAS_CM;
    } else if (qrId == QR_ID_B) {
        /*
         * Keep the transformed airplane target, but apply the B QR x tuning:
         * nominal x15 -> x12, reducing the chance of being guided into the
         * nearby forbidden zone.
         */
        mapPoint.x_cm += RADAR_R1_WIRELESS_B_TARGET_X_BIAS_CM;
    }
    return mapPoint;
}

static bool IsPointFresh(uint32_t stampMs, uint32_t timeoutMs, uint32_t nowMs)
{
    return ((stampMs != 0U) && ((nowMs - stampMs) <= timeoutMs));
}

static bool PoseIsFresh(uint32_t nowMs)
{
#if RADAR_R1_USE_BRIDGE_YAW_ONLY
    return g_poseMap.valid &&
           IsPointFresh(g_poseMap.stamp_ms, POSE_TIMEOUT_MS, nowMs);
#else
    return g_poseMap.valid &&
           IsPointFresh(g_poseMap.stamp_ms, POSE_TIMEOUT_MS, nowMs) &&
           Jy901_IsFresh(nowMs);
#endif
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
static int32_t RadarR1_FixedYStopGateDeltaCm(void)
{
    if (RadarR1_RuntimeBActive()) {
        return RADAR_R1_B_RUNTIME_Y_STOP_GATE_CM;
    }
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
    return (int32_t) RADAR_R1_FIXED_FINAL_Y_CM -
           (int32_t) RADAR_R1_A_SLOWDOWN_DIST_CM;
#else
    return RADAR_R1_FIXED_Y_STOP_DELTA_CM;
#endif
}

static bool RadarR1_FixedYStopGateReached(void)
{
    /*
     * Validation gate: go back to "record actual radar Y at origin, then stop
     * after a fixed Y growth".  Sensor-role test: radar has highest
     * priority for the configured Y stop gate; encoder is logged/guarded as an
     * independent distance sanity check, and JY901B owns yaw accuracy.
     */
    return g_uiCarRunEnabled &&
           g_poseMap.valid &&
           (g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
           (g_path.current_index == 1U) &&
           (((int32_t) g_poseMap.y_cm - g_path.points[0].y_cm) >=
            RadarR1_FixedYStopGateDeltaCm());
}

static void RadarR1_RequestUrgentYStopFromPose(uint32_t nowMs)
{
    if (RadarR1_FixedYStopGateReached()) {
        g_radarR1UrgentYStopPoseY =
            (int16_t) Clamp32(g_poseMap.y_cm, -32768, 32767);
        g_radarR1UrgentYStopMs = nowMs;
        g_radarR1UrgentYStopPending = true;
    }
}

static bool RadarR1_TakeUrgentYStop(void)
{
    bool pending;

    __disable_irq();
    pending = g_radarR1UrgentYStopPending;
    g_radarR1UrgentYStopPending = false;
    __enable_irq();
    return pending;
}

static void RadarR1_OriginStableRestart(Point2D mapPoint, int16_t yawDeg,
                                        uint32_t nowMs)
{
    g_radarR1OriginState = 1U;
    g_radarR1OriginStableFrames = 1U;
    g_radarR1OriginStableStartMs = nowMs;
    g_radarR1OriginStableXMin = mapPoint.x_cm;
    g_radarR1OriginStableXMax = mapPoint.x_cm;
    g_radarR1OriginStableYMin = mapPoint.y_cm;
    g_radarR1OriginStableYMax = mapPoint.y_cm;
    g_radarR1OriginStableRefYawDeg = yawDeg;
}

static bool RadarR1_OriginStableFeed(Point2D mapPoint, int16_t yawDeg,
                                     uint32_t nowMs)
{
    int16_t yawDelta;

    if (g_radarR1OriginStableFrames == 0U) {
        RadarR1_OriginStableRestart(mapPoint, yawDeg, nowMs);
        return false;
    }

    if (mapPoint.x_cm < g_radarR1OriginStableXMin) {
        g_radarR1OriginStableXMin = mapPoint.x_cm;
    }
    if (mapPoint.x_cm > g_radarR1OriginStableXMax) {
        g_radarR1OriginStableXMax = mapPoint.x_cm;
    }
    if (mapPoint.y_cm < g_radarR1OriginStableYMin) {
        g_radarR1OriginStableYMin = mapPoint.y_cm;
    }
    if (mapPoint.y_cm > g_radarR1OriginStableYMax) {
        g_radarR1OriginStableYMax = mapPoint.y_cm;
    }

    yawDelta = NormalizeDeg((int16_t) (yawDeg - g_radarR1OriginStableRefYawDeg));
    if (((g_radarR1OriginStableXMax - g_radarR1OriginStableXMin) >
         RADAR_R1_ORIGIN_STABLE_RANGE_CM) ||
        ((g_radarR1OriginStableYMax - g_radarR1OriginStableYMin) >
         RADAR_R1_ORIGIN_STABLE_RANGE_CM) ||
        (Abs16(yawDelta) > RADAR_R1_ORIGIN_STABLE_YAW_DEG)) {
        RadarR1_OriginStableRestart(mapPoint, yawDeg, nowMs);
        return false;
    }

    if (g_radarR1OriginStableFrames < 255U) {
        ++g_radarR1OriginStableFrames;
    }

    if ((g_radarR1OriginStableFrames >= RADAR_R1_ORIGIN_STABLE_MIN_FRAMES) &&
        ((nowMs - g_radarR1OriginStableStartMs) >=
         RADAR_R1_ORIGIN_STABLE_MIN_MS)) {
        g_radarR1OriginMap.x_cm = (int16_t)
            (((int32_t) g_radarR1OriginStableXMin +
              g_radarR1OriginStableXMax) / 2);
        g_radarR1OriginMap.y_cm = (int16_t)
            (((int32_t) g_radarR1OriginStableYMin +
              g_radarR1OriginStableYMax) / 2);
        g_radarR1OriginYawDeg = g_radarR1OriginStableRefYawDeg;
        g_radarR1OriginLocked = true;
        g_radarR1OriginState = 2U;
#if JY901_ENABLE && JY901_USE_YAW_FOR_R1 && !RADAR_R1_USE_BRIDGE_YAW_ONLY
        /*
         * Stage-8B uses bridge/slam_toolbox for X/Y but JY901B for control
         * yaw.  PB7 can be pressed before the bridge origin is actually
         * locked, so re-zero JY901B at the exact same stable-origin moment.
         * This makes OLED Y(JY) and R(bridge raw_yaw) share the same local
         * zero; any remaining difference is real sensor drift/noise, not a
         * start-time offset.
         */
        Jy901_ResetYawOriginToCurrent();
#endif
        return true;
    }

    return false;
}

static int16_t RadarR1_GuardNormalizedForwardCount(int16_t leftCount,
                                                   int16_t rightCount)
{
    int32_t rightNorm = ((int32_t) rightCount * SPEED_C4_ENCODER_R_SCALE_NUM) /
        SPEED_C4_ENCODER_R_SCALE_DEN;
    int32_t avg = ((int32_t) leftCount + rightNorm) / 2;
    return (int16_t) Clamp32(avg, -32768, 32767);
}

static void RadarR1_GuardReset(void)
{
    g_radarR1GuardEncoderActive = false;
    g_radarR1GuardBaseLeftCount = 0;
    g_radarR1GuardBaseRightCount = 0;
    g_radarR1GuardForwardCounts = 0;
    g_radarR1StopSource = 0U;
    g_radarR1ProtectFault = 0U;
}

static void RadarR1_GuardUpdateEncoder(void)
{
    int16_t leftCount;
    int16_t rightCount;
    int16_t base;
    int16_t now;

    Encoder_GetCounts(&leftCount, &rightCount);
    if (!g_radarR1GuardEncoderActive) {
        g_radarR1GuardBaseLeftCount = leftCount;
        g_radarR1GuardBaseRightCount = rightCount;
        g_radarR1GuardForwardCounts = 0;
        g_radarR1GuardEncoderActive = true;
        return;
    }

    base = RadarR1_GuardNormalizedForwardCount(g_radarR1GuardBaseLeftCount,
                                               g_radarR1GuardBaseRightCount);
    now = RadarR1_GuardNormalizedForwardCount(leftCount, rightCount);
    g_radarR1GuardForwardCounts =
        (int16_t) Clamp32((int32_t) now - base, -32768, 32767);
}

static int32_t RadarR1_GuardEncoderCapCountsForWaypoint(uint8_t waypointIndex)
{
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
    if (waypointIndex == 1U) {
        return (int32_t) RADAR_R1_GUARD_Y_SEGMENT_CAP_CM *
               RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM;
    }
    if (waypointIndex == 3U) {
        return (int32_t) RADAR_R1_GUARD_A_X_SEGMENT_CAP_CM *
               RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM;
    }
#endif
    return RADAR_R1_GUARD_ENCODER_CAP_COUNTS;
}

static bool RadarR1_GuardEncoderCapReached(uint8_t waypointIndex)
{
    int16_t forwardCounts = g_radarR1GuardForwardCounts;

    if (forwardCounts < 0) {
        forwardCounts = (int16_t) -forwardCounts;
    }

    return ((int32_t) forwardCounts >=
            RadarR1_GuardEncoderCapCountsForWaypoint(waypointIndex));
}

static bool RadarR1_GuardXCapReached(uint8_t waypointIndex)
{
    int16_t forwardCounts = g_radarR1GuardForwardCounts;
    int32_t capCounts = (int32_t) Abs16(RadarR1_FixedRouteXDeltaCm(waypointIndex)) *
                        RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM;

#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
    if (waypointIndex == 3U) {
        capCounts = (int32_t) RADAR_R1_GUARD_A_X_SEGMENT_CAP_CM *
                    RADAR_R1_GUARD_ENCODER_COUNTS_PER_CM;
    }
#endif
    if (forwardCounts < 0) {
        forwardCounts = (int16_t) -forwardCounts;
    }

    return ((int32_t) forwardCounts >= capCounts);
}

static bool RadarR1_GuardReturnHomeCapReached(void)
{
    int16_t forwardCounts = g_radarR1GuardForwardCounts;

    if (forwardCounts < 0) {
        forwardCounts = (int16_t) -forwardCounts;
    }

    return (forwardCounts >= RADAR_R1_RETURN_HOME_ENCODER_CAP_COUNTS);
}

static bool RadarR1_GuardShouldAbortStraight(int16_t radarDyCm)
{
    int16_t forwardCounts = g_radarR1GuardForwardCounts;

    if (forwardCounts < 0) {
        forwardCounts = (int16_t) -forwardCounts;
    }

    if ((forwardCounts >= RADAR_R1_GUARD_MIN_ENC_COUNTS) &&
        (radarDyCm <= RADAR_R1_GUARD_NEG_Y_CM)) {
        g_radarR1ProtectFault = 1U;
        g_radarR1StopSource = 3U;
        return true;
    }

    if ((forwardCounts >= RADAR_R1_GUARD_STAGNANT_ENC_COUNTS) &&
        (radarDyCm < RADAR_R1_GUARD_STAGNANT_RADAR_CM)) {
        g_radarR1ProtectFault = 2U;
#if RADAR_R1_GUARD_STAGNANT_ABORT_ENABLE
        g_radarR1StopSource = 3U;
        return true;
#endif
    }

#if JY901_ENABLE && !RADAR_R1_USE_BRIDGE_YAW_ONLY
    if (Jy901_IsFresh(g_tickMs) &&
        (forwardCounts >= RADAR_R1_GUARD_MIN_ENC_COUNTS) &&
        (Abs16(NormalizeDeg((int16_t) (90 - g_jy901YawDeg))) >
         RADAR_R1_GUARD_JY_YAW_MAX_DEG)) {
        g_radarR1ProtectFault = 3U;
        g_radarR1StopSource = 3U;
        return true;
    }
#endif

    return false;
}

static bool RadarR1_GuardShouldAbortRightLeg(int16_t xProgressCm)
{
    int16_t forwardCounts = g_radarR1GuardForwardCounts;

    if (forwardCounts < 0) {
        forwardCounts = (int16_t) -forwardCounts;
    }

    /*
     * The final X leg is a short encoder-bounded move.  If the encoders already
     * report several centimeters of wheel travel but radar X has not changed
     * for even a few centimeters, the pose stream is effectively locked; stop
     * instead of driving blind.
     */
    if ((forwardCounts >= RADAR_R1_GUARD_FINAL_X_STAGNANT_ENC_COUNTS) &&
        (xProgressCm < RADAR_R1_GUARD_FINAL_X_STAGNANT_RADAR_CM)) {
        g_radarR1ProtectFault = 4U;
#if RADAR_R1_GUARD_FINAL_X_STAGNANT_ABORT_ENABLE
        g_radarR1StopSource = 3U;
        return true;
#endif
    }

    return false;
}
#endif

static void RadarR1_ResetAutoOrigin(void)
{
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
    /*
     * PB7 is the run reference, not power-on.  The next valid radar frame after
     * PB7 becomes local (0,0,0) for the fixed +90deg/+Y test.  This coordinates
     * with the Pi-side origin: Pi may already output its own stable local pose,
     * while MCU still defines the experiment origin at the actual start button.
     */
    g_radarR1OriginLocked = false;
    g_radarR1OriginMap.x_cm = 0;
    g_radarR1OriginMap.y_cm = 0;
    g_radarR1OriginYawDeg = 0;
    g_radarR1YawFilterLocked = false;
    g_radarR1YawFilteredDeg = 0;
    g_radarR1PiYawHaveLast = false;
    g_radarR1PiYawLastRawDeg = 0;
    g_radarR1PiYawFallbackUntilMs = 0U;
    g_radarR1OriginStableFrames = 0U;
    g_radarR1OriginState = 1U;
    g_radarR1OriginStableStartMs = 0U;
    g_radarR1OriginStableXMin = 0;
    g_radarR1OriginStableXMax = 0;
    g_radarR1OriginStableYMin = 0;
    g_radarR1OriginStableYMax = 0;
    g_radarR1OriginStableRefYawDeg = 0;
    g_poseMap.x_cm = 0;
    g_poseMap.y_cm = 0;
    g_poseMap.yaw_deg = 0;
    g_poseMap.valid = false;
    g_poseMap.stamp_ms = 0U;
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    g_radarR1UrgentYStopPending = false;
    g_radarR1UrgentYStopPoseY = 0;
    g_radarR1UrgentYStopMs = 0U;
    RadarR1_GuardReset();
#endif
}

static bool TargetIsFresh(uint32_t nowMs)
{
    if (!g_targetPoint.valid) {
        return false;
    }
    /*
     * Airplane integration sends a one-shot target frame.  Once that frame has
     * started a mission, keep the latched target valid until the target/home
     * leg explicitly clears it; otherwise a long C/B run can reach the QR,
     * stop, and then fall back to WAIT_TARGET before the return-home leg starts.
     */
    if (g_uiCarRunEnabled && (g_mission.leg != MISSION_LEG_NONE)) {
        return true;
    }
    return IsPointFresh(g_targetPoint.stamp_ms, TARGET_TIMEOUT_MS, nowMs);
}

static bool Mission_ShouldReturnAfterTarget(void)
{
    return (g_mission.leg == MISSION_LEG_TO_TARGET) &&
           (g_p7MissionModeEnable ||
            (g_mission.target_source == TARGET_SOURCE_WIRELESS) ||
            (g_mission.target_source == TARGET_SOURCE_MANUAL));
}

static bool WorldToCell(int32_t xCm, int32_t yCm, int16_t *cellX, int16_t *cellY)
{
    int32_t gridX;
    int32_t gridY;

    if ((xCm < g_mission.map.x_min_cm) || (xCm > g_mission.map.x_max_cm) ||
        (yCm < g_mission.map.y_min_cm) || (yCm > g_mission.map.y_max_cm)) {
        return false;
    }

    gridX = (xCm - g_mission.map.x_min_cm) / MAP_CELL_CM;
    gridY = (yCm - g_mission.map.y_min_cm) / MAP_CELL_CM;
    if ((gridX < 0) || (gridX >= MAP_WIDTH_CELLS) ||
        (gridY < 0) || (gridY >= MAP_HEIGHT_CELLS)) {
        return false;
    }

    *cellX = (int16_t) gridX;
    *cellY = (int16_t) gridY;
    return true;
}

static Point2D CellToWorld(int16_t cellX, int16_t cellY)
{
    Point2D point;

    point.x_cm = g_mission.map.x_min_cm + cellX * MAP_CELL_CM + MAP_CELL_CM / 2;
    point.y_cm = g_mission.map.y_min_cm + cellY * MAP_CELL_CM + MAP_CELL_CM / 2;
    return point;
}

static int16_t CellIndex(int16_t cellX, int16_t cellY)
{
    return (int16_t) (cellY * MAP_WIDTH_CELLS + cellX);
}

static bool CellOccupied(int16_t cellX, int16_t cellY)
{
    if ((cellX < 0) || (cellX >= MAP_WIDTH_CELLS) ||
        (cellY < 0) || (cellY >= MAP_HEIGHT_CELLS)) {
        return true;
    }
    return (g_grid[CellIndex(cellX, cellY)] != 0U);
}

static void MarkInflatedRect(RectObstacle rect, int32_t marginCm)
{
    int16_t x;
    int16_t y;

    rect.x_min_cm = (int16_t) (rect.x_min_cm - marginCm);
    rect.y_min_cm = (int16_t) (rect.y_min_cm - marginCm);
    rect.x_max_cm = (int16_t) (rect.x_max_cm + marginCm);
    rect.y_max_cm = (int16_t) (rect.y_max_cm + marginCm);

    for (y = 0; y < MAP_HEIGHT_CELLS; ++y) {
        for (x = 0; x < MAP_WIDTH_CELLS; ++x) {
            Point2D cellCenter = CellToWorld(x, y);
            int16_t index = CellIndex(x, y);
            if ((cellCenter.x_cm >= rect.x_min_cm) && (cellCenter.x_cm <= rect.x_max_cm) &&
                (cellCenter.y_cm >= rect.y_min_cm) && (cellCenter.y_cm <= rect.y_max_cm)) {
                g_grid[index] = 1U;
            }
        }
    }
}

static void GridMap_Rebuild(void)
{
    uint32_t i;
    int16_t x;
    int16_t y;

    memset(g_grid, 0, sizeof(g_grid));

    for (y = 0; y < MAP_HEIGHT_CELLS; ++y) {
        for (x = 0; x < MAP_WIDTH_CELLS; ++x) {
            Point2D cellCenter = CellToWorld(x, y);
            int16_t index = CellIndex(x, y);
            if ((cellCenter.x_cm <= (g_mission.map.x_min_cm + g_mission.map.boundary_margin_cm)) ||
                (cellCenter.x_cm >= (g_mission.map.x_max_cm - g_mission.map.boundary_margin_cm)) ||
                (cellCenter.y_cm <= (g_mission.map.y_min_cm + g_mission.map.boundary_margin_cm)) ||
                (cellCenter.y_cm >= (g_mission.map.y_max_cm - g_mission.map.boundary_margin_cm))) {
                g_grid[index] = 1U;
            }
        }
    }

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid) {
            MarkInflatedRect(g_mission.forbidden[i].rect,
                             g_mission.map.obstacle_margin_cm);
        }
    }
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
static int32_t DistanceToRectWithMarginCm(RectObstacle rect, int32_t xCm,
                                          int32_t yCm, int32_t marginCm)
{
    int32_t dx = 0;
    int32_t dy = 0;

    rect.x_min_cm = (int16_t) (rect.x_min_cm - marginCm);
    rect.y_min_cm = (int16_t) (rect.y_min_cm - marginCm);
    rect.x_max_cm = (int16_t) (rect.x_max_cm + marginCm);
    rect.y_max_cm = (int16_t) (rect.y_max_cm + marginCm);

    if (xCm < rect.x_min_cm) {
        dx = (int32_t) rect.x_min_cm - xCm;
    } else if (xCm > rect.x_max_cm) {
        dx = xCm - (int32_t) rect.x_max_cm;
    }
    if (yCm < rect.y_min_cm) {
        dy = (int32_t) rect.y_min_cm - yCm;
    } else if (yCm > rect.y_max_cm) {
        dy = yCm - (int32_t) rect.y_max_cm;
    }
    return ApproxDistanceCm(dx, dy);
}

static int32_t DistanceToInflatedRectCm(RectObstacle rect, int32_t xCm, int32_t yCm)
{
    return DistanceToRectWithMarginCm(rect, xCm, yCm,
                                      g_mission.map.obstacle_margin_cm);
}

static bool RadarR1_AStarPointTooCloseToForbidden(int32_t xCm, int32_t yCm)
{
    uint8_t i;

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid &&
            (DistanceToInflatedRectCm(g_mission.forbidden[i].rect,
                                      xCm,
                                      yCm) <=
             RADAR_R1_A_FORBID_PLAN_CLEARANCE_CM)) {
            return true;
        }
    }
    return false;
}

static uint16_t RadarR1_AStarATargetCorridorCost(Point2D point)
{
    int32_t overCm;
    uint32_t steps;
    uint32_t cost;
    uint32_t lowLeftCost = 0U;

    if (!g_targetPoint.valid ||
        (g_targetPoint.qr_type != QR_ID_A) ||
        (g_mission.leg != MISSION_LEG_TO_TARGET)) {
        return 0U;
    }

    cost = 0U;
    if ((point.y_cm >= RADAR_R1_A_ROUTE_CORRIDOR_Y_MIN_CM) &&
        (point.y_cm <= RADAR_R1_A_ROUTE_CORRIDOR_Y_MAX_CM) &&
        (point.x_cm > RADAR_R1_A_ROUTE_CORRIDOR_MAX_X_CM)) {
        overCm = (int32_t) point.x_cm - RADAR_R1_A_ROUTE_CORRIDOR_MAX_X_CM;
        steps = (uint32_t)
            ((overCm + RADAR_R1_A_ROUTE_CORRIDOR_COST_STEP_CM - 1) /
             RADAR_R1_A_ROUTE_CORRIDOR_COST_STEP_CM);
        cost = steps * RADAR_R1_A_ROUTE_CORRIDOR_COST_PER_STEP;
        if (cost > RADAR_R1_A_ROUTE_CORRIDOR_COST_MAX) {
            cost = RADAR_R1_A_ROUTE_CORRIDOR_COST_MAX;
        }
    }

    if ((point.y_cm >= RADAR_R1_A_ROUTE_LOW_LEFT_Y_MIN_CM) &&
        (point.y_cm <= RADAR_R1_A_ROUTE_LOW_LEFT_Y_MAX_CM) &&
        (point.x_cm < RADAR_R1_A_ROUTE_LOW_LEFT_MIN_X_CM)) {
        int32_t leftCm = (int32_t) RADAR_R1_A_ROUTE_LOW_LEFT_MIN_X_CM -
            point.x_cm;
        uint32_t leftSteps = (uint32_t)
            ((leftCm + RADAR_R1_A_ROUTE_CORRIDOR_COST_STEP_CM - 1) /
             RADAR_R1_A_ROUTE_CORRIDOR_COST_STEP_CM);
        lowLeftCost = leftSteps * RADAR_R1_A_ROUTE_LOW_LEFT_COST_PER_STEP;
        if (lowLeftCost > RADAR_R1_A_ROUTE_LOW_LEFT_COST_MAX) {
            lowLeftCost = RADAR_R1_A_ROUTE_LOW_LEFT_COST_MAX;
        }
    }

    cost += lowLeftCost;
    if (cost > 1000U) {
        cost = 1000U;
    }
    return (uint16_t) cost;
}

static uint16_t RadarR1_AStarForbiddenSoftCost(int16_t cellX, int16_t cellY)
{
    uint8_t i;
    uint8_t nearCount = 0U;
    Point2D point;
    uint32_t worstCost = 0U;
    int32_t softDistCm = RADAR_R1_FORBID_SOFT_COST_DIST_CM;
    uint32_t costPerStep = RADAR_R1_FORBID_SOFT_COST_PER_STEP;

    if ((cellX < 0) || (cellX >= MAP_WIDTH_CELLS) ||
        (cellY < 0) || (cellY >= MAP_HEIGHT_CELLS)) {
        return 0U;
    }

    if (g_mission.leg == MISSION_LEG_TO_HOME) {
        softDistCm = RADAR_R1_ASTAR_HOME_SOFT_COST_DIST_CM;
        costPerStep = RADAR_R1_ASTAR_HOME_SOFT_COST_PER_STEP;
    }

    point = CellToWorld(cellX, cellY);
    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid) {
            int32_t distCm = DistanceToInflatedRectCm(g_mission.forbidden[i].rect,
                                                      point.x_cm,
                                                      point.y_cm);
            if ((distCm > 0) &&
                (distCm < softDistCm)) {
                uint32_t steps = (uint32_t)
                    ((softDistCm - distCm +
                      RADAR_R1_FORBID_SOFT_COST_STEP_CM - 1) /
                     RADAR_R1_FORBID_SOFT_COST_STEP_CM);
                uint32_t cost = steps * costPerStep;
                nearCount++;
                if (cost > worstCost) {
                    worstCost = cost;
                }
            }
        }
    }

    /* If a cell is in the passage between multiple forbidden zones, do not
     * stack penalties.  Discount the max single-zone penalty so the corridor
     * stays passable while still preferring the center of the gap. */
    if (nearCount >= 2U) {
        worstCost = (worstCost * RADAR_R1_FORBID_SOFT_COST_CORRIDOR_PCT) / 100U;
    }
    worstCost += RadarR1_AStarATargetCorridorCost(point);
    if (worstCost > 1000U) {
        worstCost = 1000U;
    }
    return (uint16_t) worstCost;
}

static uint16_t RadarR1_AStarTargetPocketReward(int16_t cellX, int16_t cellY)
{
    Point2D point;
    int32_t dx;
    int32_t dy;
    int32_t distCm;
    uint32_t steps;
    uint32_t reward;

    if (!g_targetPoint.valid ||
        (g_mission.leg != MISSION_LEG_TO_TARGET) ||
        (cellX < 0) || (cellX >= MAP_WIDTH_CELLS) ||
        (cellY < 0) || (cellY >= MAP_HEIGHT_CELLS)) {
        return 0U;
    }

    point = CellToWorld(cellX, cellY);
    dx = (int32_t) g_targetPoint.target_map_frame.x_cm - point.x_cm;
    dy = (int32_t) g_targetPoint.target_map_frame.y_cm - point.y_cm;
    distCm = ApproxDistanceCm(dx, dy);
    if ((distCm <= 0) || (distCm >= RADAR_R1_TARGET_POCKET_REWARD_DIST_CM)) {
        return 0U;
    }

    steps = (uint32_t)
        ((RADAR_R1_TARGET_POCKET_REWARD_DIST_CM - distCm +
          RADAR_R1_TARGET_POCKET_REWARD_STEP_CM - 1) /
         RADAR_R1_TARGET_POCKET_REWARD_STEP_CM);
    reward = steps * RADAR_R1_TARGET_POCKET_REWARD_PER_STEP;
    if (reward > 215U) {
        reward = 215U;
    }
    return (uint16_t) reward;
}

static bool RadarR1_AStarCellTooCloseToForbidden(int16_t cellX, int16_t cellY)
{
    Point2D point;

    if (CellOccupied(cellX, cellY)) {
        return true;
    }
    point = CellToWorld(cellX, cellY);
    return RadarR1_AStarPointTooCloseToForbidden(point.x_cm, point.y_cm);
}

static bool RadarR1_AStarAxisWaypointReached(const PathBuffer *path,
                                             int16_t poseX,
                                             int16_t poseY)
{
    PathPoint previous;
    PathPoint waypoint;
    int16_t dx;
    int16_t dy;
    int16_t crossTrackLimitCm;
    int32_t crossTrackCm;

    if (((path->source != PATH_SRC_ASTAR) &&
         (path->source != PATH_SRC_ASTAR_HOME)) ||
        (path->current_index == 0U) ||
        (path->current_index >= path->len)) {
        return false;
    }

    previous = path->points[path->current_index - 1U];
    waypoint = path->points[path->current_index];
    dx = (int16_t) (waypoint.x_cm - previous.x_cm);
    dy = (int16_t) (waypoint.y_cm - previous.y_cm);
    crossTrackLimitCm = (path->source == PATH_SRC_ASTAR_HOME) ?
        RADAR_R1_ASTAR_HOME_AXIS_CROSSTRACK_CM :
        RADAR_R1_A_FORBID_AXIS_CROSSTRACK_CM;

    if ((dx != 0) && (Abs16(dx) >= Abs16(dy))) {
        crossTrackCm = (int32_t) poseY - waypoint.y_cm;
        if (Abs32(crossTrackCm) > crossTrackLimitCm) {
            return false;
        }
        return (dx > 0) ?
            (poseX >= (waypoint.x_cm - RADAR_R1_A_FORBID_AXIS_REACH_CM)) :
            (poseX <= (waypoint.x_cm + RADAR_R1_A_FORBID_AXIS_REACH_CM));
    }
    if (dy != 0) {
        crossTrackCm = (int32_t) poseX - waypoint.x_cm;
        if (Abs32(crossTrackCm) > crossTrackLimitCm) {
            return false;
        }
        return (dy > 0) ?
            (poseY >= (waypoint.y_cm - RADAR_R1_A_FORBID_AXIS_REACH_CM)) :
            (poseY <= (waypoint.y_cm + RADAR_R1_A_FORBID_AXIS_REACH_CM));
    }

    return (ApproxDistanceCm((int32_t) waypoint.x_cm - poseX,
                             (int32_t) waypoint.y_cm - poseY) <=
            RADAR_R1_A_FORBID_AXIS_REACH_CM);
}

static int16_t RadarR1_AStarTargetDistanceCm(void)
{
    if (!g_targetPoint.valid) {
        return 32767;
    }
    return ApproxDistanceCm(
        (int32_t) g_targetPoint.target_map_frame.x_cm - g_poseMap.x_cm,
        (int32_t) g_targetPoint.target_map_frame.y_cm - g_poseMap.y_cm);
}

static int16_t RadarR1_AStarHomeDistanceCm(void)
{
    return ApproxDistanceCm(
        (int32_t) g_mission.map.home_map_frame.x_cm - g_poseMap.x_cm,
        (int32_t) g_mission.map.home_map_frame.y_cm - g_poseMap.y_cm);
}

static bool RadarR1_AStarHomeStopZoneReached(void)
{
    int32_t dx;
    int32_t dy;
    int32_t stopX = RADAR_R1_ASTAR_HOME_STOP_X_CM;
    int32_t stopY = RADAR_R1_ASTAR_HOME_STOP_Y_CM;

    if ((g_path.source != PATH_SRC_ASTAR_HOME) ||
        (g_mission.leg != MISSION_LEG_TO_HOME)) {
        return false;
    }

    if (g_targetPoint.valid && (g_targetPoint.qr_type == QR_ID_A)) {
        stopX = RADAR_R1_ASTAR_A_HOME_STOP_X_CM;
        stopY = RADAR_R1_ASTAR_A_HOME_STOP_Y_CM;
    }

    dx = (int32_t) g_poseMap.x_cm - g_mission.map.home_map_frame.x_cm;
    dy = (int32_t) g_poseMap.y_cm - g_mission.map.home_map_frame.y_cm;
    return (Abs32(dx) <= stopX) && (Abs32(dy) <= stopY);
}

static bool RadarR1_AStarTargetZoneReached(void)
{
    int32_t dx;
    int32_t dy;

    if (!g_targetPoint.valid ||
        (g_path.source != PATH_SRC_ASTAR)) {
        return false;
    }

    dx = (int32_t) g_poseMap.x_cm - g_targetPoint.target_map_frame.x_cm;
    dy = (int32_t) g_poseMap.y_cm - g_targetPoint.target_map_frame.y_cm;
    return (Abs32(dx) <= RADAR_R1_A_FORBID_TARGET_STOP_X_CM) &&
           (Abs32(dy) <= RADAR_R1_A_FORBID_TARGET_STOP_Y_CM);
}

static bool RadarR1_AStarTargetBonusZoneReached(void)
{
    int32_t dx;
    int32_t dy;
    int32_t bonusX = RADAR_R1_A_FORBID_TARGET_BONUS_X_CM;
    int32_t bonusY = RADAR_R1_A_FORBID_TARGET_BONUS_Y_CM;

    if (!g_targetPoint.valid ||
        (g_path.source != PATH_SRC_ASTAR)) {
        return false;
    }

    if (g_targetPoint.qr_type == QR_ID_A) {
        bonusX = RADAR_R1_A_FORBID_A_TARGET_BONUS_X_CM;
        bonusY = RADAR_R1_A_FORBID_A_TARGET_BONUS_Y_CM;
    }

    dx = (int32_t) g_poseMap.x_cm - g_targetPoint.target_map_frame.x_cm;
    dy = (int32_t) g_poseMap.y_cm - g_targetPoint.target_map_frame.y_cm;
    return (Abs32(dx) <= bonusX) && (Abs32(dy) <= bonusY);
}

static bool RadarR1_AStarTargetAcceptZoneReached(void)
{
    int32_t dx;
    int32_t dy;

    if (!g_targetPoint.valid ||
        (g_path.source != PATH_SRC_ASTAR)) {
        return false;
    }

    dx = (int32_t) g_poseMap.x_cm - g_targetPoint.target_map_frame.x_cm;
    dy = (int32_t) g_poseMap.y_cm - g_targetPoint.target_map_frame.y_cm;
    return (Abs32(dx) <= RADAR_R1_A_FORBID_TARGET_ACCEPT_X_CM) &&
           (Abs32(dy) <= RADAR_R1_A_FORBID_TARGET_ACCEPT_Y_CM);
}

static bool RadarR1_BTargetSafeAcceptReached(void)
{
    int32_t dx;
    int32_t dy;

    if (!g_targetPoint.valid ||
        (g_targetPoint.qr_type != QR_ID_B) ||
        (g_path.source != PATH_SRC_ASTAR) ||
        (g_mission.leg != MISSION_LEG_TO_TARGET)) {
        return false;
    }

    dx = (int32_t) g_poseMap.x_cm - g_targetPoint.target_map_frame.x_cm;
    dy = (int32_t) g_poseMap.y_cm - g_targetPoint.target_map_frame.y_cm;
    return (Abs32(dx) <= RADAR_R1_B_TARGET_SAFE_ACCEPT_X_CM) &&
           (Abs32(dy) <= RADAR_R1_B_TARGET_SAFE_ACCEPT_Y_CM) &&
           ((int32_t) g_poseMap.x_cm <= RADAR_R1_B_TARGET_SAFE_MAX_X_CM);
}

static bool RadarR1_AStarNearForbiddenBoundary(void)
{
    uint8_t i;

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid &&
            (DistanceToInflatedRectCm(g_mission.forbidden[i].rect,
                                      g_poseMap.x_cm,
                                      g_poseMap.y_cm) <=
             RADAR_R1_FORBID_PROTECT_DIST_CM)) {
            return true;
        }
    }
    return false;
}

static int32_t RadarR1_AStarNearestForbiddenDistanceCm(void)
{
    uint8_t i;
    bool found = false;
    int32_t bestDistCm = 32767;

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid) {
            int32_t distCm = DistanceToInflatedRectCm(g_mission.forbidden[i].rect,
                                                      g_poseMap.x_cm,
                                                      g_poseMap.y_cm);
            if (!found || (distCm < bestDistCm)) {
                found = true;
                bestDistCm = distCm;
            }
        }
    }

    return found ? bestDistCm : 32767;
}

static int8_t RadarR1_AStarNearestForbiddenSide(void)
{
    uint8_t i;
    bool found = false;
    int32_t bestDistCm = 32767;
    int32_t bestVx = 0;
    int32_t bestVy = 0;
    int16_t yaw;
    int32_t lateral;

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid) {
            RectObstacle rect = g_mission.forbidden[i].rect;
            int32_t margin = g_mission.map.obstacle_margin_cm;
            int32_t xMin = (int32_t) rect.x_min_cm - margin;
            int32_t xMax = (int32_t) rect.x_max_cm + margin;
            int32_t yMin = (int32_t) rect.y_min_cm - margin;
            int32_t yMax = (int32_t) rect.y_max_cm + margin;
            int32_t nearX = Clamp32(g_poseMap.x_cm, xMin, xMax);
            int32_t nearY = Clamp32(g_poseMap.y_cm, yMin, yMax);
            int32_t vx = nearX - g_poseMap.x_cm;
            int32_t vy = nearY - g_poseMap.y_cm;
            int32_t distCm = ApproxDistanceCm(vx, vy);

            if (!found || (distCm < bestDistCm)) {
                found = true;
                bestDistCm = distCm;
                bestVx = vx;
                bestVy = vy;
            }
        }
    }

    if (!found || ((bestVx == 0) && (bestVy == 0))) {
        return 0;
    }

    yaw = NormalizeDeg(g_poseMap.yaw_deg);
    if ((yaw >= -45) && (yaw <= 45)) {
        /* Facing x+: left is y+. */
        lateral = bestVy;
    } else if ((yaw > 45) && (yaw < 135)) {
        /* Facing y+: left is x-. */
        lateral = -bestVx;
    } else if ((yaw <= -45) && (yaw > -135)) {
        /* Facing y-: left is x+. */
        lateral = bestVx;
    } else {
        /* Facing x-: left is y-. */
        lateral = -bestVy;
    }

    if (lateral > 0) {
        return 1;
    }
    if (lateral < 0) {
        return -1;
    }
    return 0;
}

static bool RadarR1_AStarAOpenPassageHardStopBypass(uint8_t forbiddenIndex)
{
    if (!g_targetPoint.valid ||
        (g_targetPoint.qr_type != QR_ID_A) ||
        (g_mission.leg != MISSION_LEG_TO_TARGET) ||
        (g_path.source != PATH_SRC_ASTAR)) {
        return false;
    }

    /* Index 1 is the measured left/rear slanted forbidden zone stored as an
     * AABB.  Only bypass its visually open top-right corner; all other zones
     * and all other parts of this zone still hard-stop normally.
     */
    if (forbiddenIndex != 1U) {
        return false;
    }

    return ((g_poseMap.x_cm >= RADAR_R1_A_OPEN_PASSAGE_X_MIN_CM) &&
            (g_poseMap.x_cm <= RADAR_R1_A_OPEN_PASSAGE_X_MAX_CM) &&
            (g_poseMap.y_cm >= RADAR_R1_A_OPEN_PASSAGE_Y_MIN_CM) &&
            (g_poseMap.y_cm <= RADAR_R1_A_OPEN_PASSAGE_Y_MAX_CM));
}

static bool RadarR1_AStarInsideHardForbidden(void)
{
    uint8_t i;

    for (i = 0U; i < g_mission.forbidden_count; ++i) {
        if (g_mission.forbidden[i].valid &&
            (DistanceToRectWithMarginCm(g_mission.forbidden[i].rect,
                                        g_poseMap.x_cm,
                                        g_poseMap.y_cm,
                                        -RADAR_R1_FORBID_HARD_STOP_SHRINK_CM) == 0)) {
            if (RadarR1_AStarAOpenPassageHardStopBypass(i)) {
                continue;
            }
            return true;
        }
    }
    return false;
}

static Point2D RadarR1_AStarLookaheadTarget(const PathBuffer *path,
                                            int16_t currentX,
                                            int16_t currentY)
{
    Point2D target;
    int32_t dx;
    int32_t dy;
    int16_t dist;
    int16_t lookaheadCm = RADAR_R1_A_FORBID_LOOKAHEAD_CM;
    uint8_t index = path->current_index;

    if ((path->source != PATH_SRC_ASTAR) &&
        (path->source != PATH_SRC_ASTAR_HOME)) {
        target.x_cm = path->points[index].x_cm;
        target.y_cm = path->points[index].y_cm;
        return target;
    }

    if (RadarR1_AStarNearForbiddenBoundary()) {
        lookaheadCm = RADAR_R1_A_FORBID_LOOKAHEAD_NEAR_CM;
    }

    if (path->len > 0U) {
        int32_t goalDx = (int32_t) path->points[path->len - 1U].x_cm - currentX;
        int32_t goalDy = (int32_t) path->points[path->len - 1U].y_cm - currentY;
        if (ApproxDistanceCm(goalDx, goalDy) <= RADAR_R1_A_FORBID_GOAL_NEAR_CM) {
            lookaheadCm = RADAR_R1_A_FORBID_LOOKAHEAD_GOAL_CM;
        }
    }

    dx = (int32_t) path->points[index].x_cm - currentX;
    dy = (int32_t) path->points[index].y_cm - currentY;
    dist = ApproxDistanceCm(dx, dy);

    if ((dist > lookaheadCm) && (dist > 0)) {
        target.x_cm = currentX + (int16_t) ((dx * lookaheadCm) / dist);
        target.y_cm = currentY + (int16_t) ((dy * lookaheadCm) / dist);
        return target;
    }
    if (((uint8_t) (index + 1U) < path->len) && (dist < lookaheadCm)) {
        target.x_cm = path->points[index + 1U].x_cm;
        target.y_cm = path->points[index + 1U].y_cm;
        return target;
    }

    target.x_cm = path->points[index].x_cm;
    target.y_cm = path->points[index].y_cm;
    return target;
}

#if RADAR_R1_A_FORBID_ORTHO_ROUTE_ENABLE
static bool RadarR1_BuildAForbidOrthoRoute(Point2D startWorld,
                                            Point2D goalWorld,
                                            PathBuffer *path)
{
    int16_t startX = (int16_t) Clamp32(startWorld.x_cm, -32768, 32767);
    int16_t startY = (int16_t) Clamp32(startWorld.y_cm, -32768, 32767);
    int16_t goalX = (int16_t) Clamp32(goalWorld.x_cm, -32768, 32767);
    int16_t goalY = (int16_t) Clamp32(goalWorld.y_cm, -32768, 32767);
    bool goHome = (Abs16(goalX) <= FINAL_REACH_CM) &&
                  (Abs16(goalY) <= FINAL_REACH_CM);

    path->len = 0U;
    path->current_index = 0U;
    path->source = goHome ? PATH_SRC_ASTAR_HOME : PATH_SRC_ASTAR;
    path->points[path->len++] = (PathPoint){startX, startY};
    if (goHome) {
        path->points[path->len++] = (PathPoint){startX, RADAR_R1_A_FORBID_CORNER_Y2_CM};
        path->points[path->len++] = (PathPoint){RADAR_R1_A_FORBID_CORNER_X_CM,
                                                RADAR_R1_A_FORBID_CORNER_Y2_CM};
        path->points[path->len++] = (PathPoint){RADAR_R1_A_FORBID_CORNER_X_CM,
                                                RADAR_R1_A_FORBID_CORNER_Y1_CM};
        path->points[path->len++] = (PathPoint){goalX, RADAR_R1_A_FORBID_CORNER_Y1_CM};
    } else {
        path->points[path->len++] = (PathPoint){startX, RADAR_R1_A_FORBID_CORNER_Y1_CM};
        path->points[path->len++] = (PathPoint){RADAR_R1_A_FORBID_CORNER_X_CM,
                                                RADAR_R1_A_FORBID_CORNER_Y1_CM};
        path->points[path->len++] = (PathPoint){RADAR_R1_A_FORBID_CORNER_X_CM,
                                                RADAR_R1_A_FORBID_CORNER_Y2_CM};
        path->points[path->len++] = (PathPoint){goalX, RADAR_R1_A_FORBID_CORNER_Y2_CM};
    }
    path->points[path->len++] = (PathPoint){goalX, goalY};
    return true;
}
#endif
#endif

static uint16_t HeuristicCost(int16_t fromX, int16_t fromY, int16_t toX, int16_t toY)
{
    int16_t dx = Abs16((int16_t) (toX - fromX));
    int16_t dy = Abs16((int16_t) (toY - fromY));
    int16_t diag = (dx < dy) ? dx : dy;
    int16_t straight = (dx > dy) ? (dx - dy) : (dy - dx);
    return (uint16_t) (diag * 14 + straight * 10);
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
static void RadarR1_AStarDirFromYaw(int16_t yawDeg, int16_t *dirX, int16_t *dirY)
{
    int16_t yaw = NormalizeDeg(yawDeg);

    if ((yaw >= -22) && (yaw <= 22)) {
        *dirX = 1;
        *dirY = 0;
    } else if ((yaw > 22) && (yaw <= 67)) {
        *dirX = 1;
        *dirY = 1;
    } else if ((yaw > 67) && (yaw <= 112)) {
        *dirX = 0;
        *dirY = 1;
    } else if ((yaw > 112) && (yaw <= 157)) {
        *dirX = -1;
        *dirY = 1;
    } else if ((yaw > 157) || (yaw <= -158)) {
        *dirX = -1;
        *dirY = 0;
    } else if ((yaw > -158) && (yaw <= -113)) {
        *dirX = -1;
        *dirY = -1;
    } else if ((yaw > -113) && (yaw <= -68)) {
        *dirX = 0;
        *dirY = -1;
    } else {
        *dirX = 1;
        *dirY = -1;
    }
}

static uint16_t RadarR1_AStarTurnPenalty(int16_t prevDx,
                                         int16_t prevDy,
                                         int16_t nextDx,
                                         int16_t nextDy,
                                         bool fromStart)
{
    int16_t dot;

    if ((prevDx == nextDx) && (prevDy == nextDy)) {
        return 0U;
    }

    dot = (int16_t) ((prevDx * nextDx) + (prevDy * nextDy));

    if (dot < 0) {
        /*
         * A reverse first step is exactly the field failure we just saw:
         * A* selected (-80,0), forcing the car to turn toward 180 deg before
         * moving.  Penalize that heavily while still allowing it if no other
         * route exists.
         */
        if (fromStart) {
            return RADAR_R1_A_FORBID_ASTAR_START_REVERSE_COST;
        }
        return RADAR_R1_A_FORBID_ASTAR_TURN_180_COST;
    }
    if (dot == 0) {
        return RADAR_R1_A_FORBID_ASTAR_TURN_90_COST;
    }
    if ((prevDx == 0) || (prevDy == 0) || (nextDx == 0) || (nextDy == 0)) {
        return RADAR_R1_A_FORBID_ASTAR_TURN_45_COST;
    }
    return RADAR_R1_A_FORBID_ASTAR_TURN_135_COST;
}
#endif

static bool BuildPathFromParents(int16_t startIndex, int16_t goalIndex, PathBuffer *path)
{
    int16_t reverseCells[MAX_RECON_CELLS];
    uint8_t reverseLen = 0U;
    int16_t node = goalIndex;
    int16_t previousX = -1;
    int16_t previousY = -1;
    int16_t dirX = 0;
    int16_t dirY = 0;
    bool haveDir = false;
    int16_t i;

    while ((node >= 0) && (reverseLen < MAX_RECON_CELLS)) {
        reverseCells[reverseLen++] = node;
        if (node == startIndex) {
            break;
        }
        node = g_parent[node];
    }

    if (reverseLen == MAX_RECON_CELLS && reverseCells[reverseLen - 1U] != startIndex) {
        return false;
    }
    if (reverseCells[reverseLen - 1U] != startIndex) {
        return false;
    }

    path->len = 0U;
    path->current_index = 0U;

    for (i = (int16_t) (reverseLen - 1U); i >= 0; --i) {
        int16_t index = reverseCells[i];
        int16_t cellX = index % MAP_WIDTH_CELLS;
        int16_t cellY = index / MAP_WIDTH_CELLS;

        if (path->len == 0U) {
            Point2D world = CellToWorld(cellX, cellY);
            path->points[path->len].x_cm = (int16_t) world.x_cm;
            path->points[path->len].y_cm = (int16_t) world.y_cm;
            path->len++;
        } else {
            int16_t stepX = (int16_t) (cellX - previousX);
            int16_t stepY = (int16_t) (cellY - previousY);
            if (!haveDir) {
                dirX = stepX;
                dirY = stepY;
                haveDir = true;
            } else if ((stepX != dirX) || (stepY != dirY)) {
                Point2D world = CellToWorld(previousX, previousY);
                if (path->len >= MAX_PATH_POINTS) {
                    return false;
                }
                path->points[path->len].x_cm = (int16_t) world.x_cm;
                path->points[path->len].y_cm = (int16_t) world.y_cm;
                path->len++;
                dirX = stepX;
                dirY = stepY;
            }
        }

        previousX = cellX;
        previousY = cellY;
    }

    if (path->len >= MAX_PATH_POINTS) {
        return false;
    }
    if ((path->points[path->len - 1U].x_cm != path->points[0U].x_cm) ||
        (path->points[path->len - 1U].y_cm != path->points[0U].y_cm)) {
        Point2D world = CellToWorld(goalIndex % MAP_WIDTH_CELLS, goalIndex / MAP_WIDTH_CELLS);
        path->points[path->len].x_cm = (int16_t) world.x_cm;
        path->points[path->len].y_cm = (int16_t) world.y_cm;
        path->len++;
    }

    return (path->len > 0U);
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE && \
    RADAR_R1_A_FORBID_AXIS_EXPAND_ENABLE
static bool RadarR1_AStarAxisSegmentClear(int16_t x1,
                                          int16_t y1,
                                          int16_t x2,
                                          int16_t y2)
{
    int32_t dx = (int32_t) x2 - x1;
    int32_t dy = (int32_t) y2 - y1;
    int32_t len = Abs32(dx) + Abs32(dy);
    int32_t steps;
    int32_t i;

    if ((dx != 0) && (dy != 0)) {
        return false;
    }
    steps = (len / MAP_CELL_CM) + 1;
    if (steps < 1) {
        steps = 1;
    }
    for (i = 0; i <= steps; ++i) {
        int16_t cx;
        int16_t cy;
        int16_t x = (int16_t) (x1 + ((dx * i) / steps));
        int16_t y = (int16_t) (y1 + ((dy * i) / steps));
        if (!WorldToCell(x, y, &cx, &cy) ||
            RadarR1_AStarCellTooCloseToForbidden(cx, cy)) {
            return false;
        }
    }
    return true;
}

static bool RadarR1_AStarAppendAxisPoint(PathBuffer *path, int16_t x, int16_t y)
{
    if (path->len > 0U) {
        PathPoint *last = &path->points[path->len - 1U];
        if ((last->x_cm == x) && (last->y_cm == y)) {
            return true;
        }
    }
    if (path->len >= MAX_PATH_POINTS) {
        return false;
    }
    path->points[path->len].x_cm = x;
    path->points[path->len].y_cm = y;
    path->len++;
    return true;
}

static bool RadarR1_AStarAxisExpandPath(PathBuffer *path, int16_t startYawDeg)
{
    PathPoint original[MAX_PATH_POINTS];
    uint8_t originalLen = path->len;
    uint8_t i;
    bool preferXFirst;
    int16_t startDirX = 1;
    int16_t startDirY = 0;

    if (path->len < 2U) {
        return true;
    }

    memcpy(original, path->points, sizeof(original));
    RadarR1_AStarDirFromYaw(startYawDeg, &startDirX, &startDirY);
    preferXFirst = (Abs16(startDirX) >= Abs16(startDirY));

    path->len = 0U;
    path->current_index = 0U;
    if (!RadarR1_AStarAppendAxisPoint(path,
                                      original[0].x_cm,
                                      original[0].y_cm)) {
        return false;
    }

    for (i = 1U; i < originalLen; ++i) {
        PathPoint *last = &path->points[path->len - 1U];
        int16_t fromX = last->x_cm;
        int16_t fromY = last->y_cm;
        int16_t toX = original[i].x_cm;
        int16_t toY = original[i].y_cm;
        int16_t dx = (int16_t) (toX - fromX);
        int16_t dy = (int16_t) (toY - fromY);
        bool xFirst = preferXFirst;
        bool xFirstOk = true;
        bool yFirstOk = true;
        int16_t midXFirstX = toX;
        int16_t midXFirstY = fromY;
        int16_t midYFirstX = fromX;
        int16_t midYFirstY = toY;

        if ((dx == 0) || (dy == 0)) {
            if (RadarR1_AStarAxisSegmentClear(fromX, fromY, toX, toY)) {
                if (!RadarR1_AStarAppendAxisPoint(path, toX, toY)) {
                    return false;
                }
                preferXFirst = (dx != 0);
            } else {
                if (!RadarR1_AStarAppendAxisPoint(path, toX, toY)) {
                    return false;
                }
            }
            continue;
        }

        xFirstOk = RadarR1_AStarAxisSegmentClear(fromX, fromY,
                                                 midXFirstX, midXFirstY) &&
                   RadarR1_AStarAxisSegmentClear(midXFirstX, midXFirstY,
                                                 toX, toY);
        yFirstOk = RadarR1_AStarAxisSegmentClear(fromX, fromY,
                                                 midYFirstX, midYFirstY) &&
                   RadarR1_AStarAxisSegmentClear(midYFirstX, midYFirstY,
                                                 toX, toY);

        if (xFirst && !xFirstOk && yFirstOk) {
            xFirst = false;
        } else if (!xFirst && !yFirstOk && xFirstOk) {
            xFirst = true;
        } else if (!xFirstOk && !yFirstOk) {
            /*
             * If neither Manhattan split is clear, keep the original A* corner
             * rather than inventing a blocked axis point.  This preserves
             * planner completeness for odd grid cases, and the log will show
             * the remaining diagonal via wp_x/wp_y.
             */
            if (!RadarR1_AStarAppendAxisPoint(path, toX, toY)) {
                return false;
            }
            preferXFirst = (Abs16(dx) >= Abs16(dy));
            continue;
        }

        if (xFirst) {
            if (!RadarR1_AStarAppendAxisPoint(path, midXFirstX, midXFirstY) ||
                !RadarR1_AStarAppendAxisPoint(path, toX, toY)) {
                return false;
            }
        } else {
            if (!RadarR1_AStarAppendAxisPoint(path, midYFirstX, midYFirstY) ||
                !RadarR1_AStarAppendAxisPoint(path, toX, toY)) {
                return false;
            }
        }
        preferXFirst = !xFirst;
    }

    return (path->len > 0U);
}

#if RADAR_R1_ASTAR_HOME_SHORTCUT_ENABLE
static bool RadarR1_AStarSegmentClear(int16_t x1,
                                      int16_t y1,
                                      int16_t x2,
                                      int16_t y2)
{
    int32_t dx = (int32_t) x2 - x1;
    int32_t dy = (int32_t) y2 - y1;
    int32_t maxDelta = Abs32(dx);
    int32_t steps;
    int32_t i;

    if (Abs32(dy) > maxDelta) {
        maxDelta = Abs32(dy);
    }
    steps = (maxDelta / RADAR_R1_ASTAR_HOME_SHORTCUT_STEP_CM) + 1;
    if (steps < 1) {
        steps = 1;
    }

    for (i = 0; i <= steps; ++i) {
        int16_t cellX;
        int16_t cellY;
        int16_t x = (int16_t) (x1 + ((dx * i) / steps));
        int16_t y = (int16_t) (y1 + ((dy * i) / steps));
        if (!WorldToCell(x, y, &cellX, &cellY) ||
            RadarR1_AStarCellTooCloseToForbidden(cellX, cellY)) {
            return false;
        }
    }
    return true;
}

static bool RadarR1_AStarHomeShortcutPath(PathBuffer *path)
{
    PathPoint original[MAX_PATH_POINTS];
    uint8_t originalLen = path->len;
    uint8_t srcIndex = 0U;

    if ((g_mission.leg != MISSION_LEG_TO_HOME) ||
        (path->len < 3U)) {
        return true;
    }

    memcpy(original, path->points, sizeof(original));
    path->len = 0U;
    path->current_index = 0U;
    if (!RadarR1_AStarAppendAxisPoint(path,
                                      original[0U].x_cm,
                                      original[0U].y_cm)) {
        return false;
    }

    while (srcIndex < (uint8_t) (originalLen - 1U)) {
        uint8_t nextIndex = (uint8_t) (srcIndex + 1U);
        uint8_t probeIndex;

        for (probeIndex = (uint8_t) (originalLen - 1U);
             probeIndex > (uint8_t) (srcIndex + 1U);
             --probeIndex) {
            if (RadarR1_AStarSegmentClear(original[srcIndex].x_cm,
                                          original[srcIndex].y_cm,
                                          original[probeIndex].x_cm,
                                          original[probeIndex].y_cm)) {
                nextIndex = probeIndex;
                break;
            }
        }

        if (!RadarR1_AStarAppendAxisPoint(path,
                                          original[nextIndex].x_cm,
                                          original[nextIndex].y_cm)) {
            return false;
        }
        srcIndex = nextIndex;
    }

    return (path->len > 0U);
}
#endif
#endif

static PlanResult PlanPathAStar(Point2D startWorld, Point2D goalWorld, PathBuffer *path)
{
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE && \
    RADAR_R1_A_FORBID_ASTAR_ORTHO_NEIGHBOR_ENABLE
    static const int8_t offsetX[4] = {0, -1, 1, 0};
    static const int8_t offsetY[4] = {1, 0, 0, -1};
    static const uint8_t moveCost[4] = {10, 10, 10, 10};
#else
    static const int8_t offsetX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int8_t offsetY[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const uint8_t moveCost[8] = {10, 14, 10, 14, 10, 14, 10, 14};
#endif
    PlanResult result = {false, PLAN_FAIL_NO_PATH, 0U};
    int16_t startX;
    int16_t startY;
    int16_t goalX;
    int16_t goalY;
    int16_t startIndex;
    int16_t goalIndex;
    uint16_t openCount = 0U;
    uint16_t i;

    if (!WorldToCell(startWorld.x_cm, startWorld.y_cm, &startX, &startY)) {
        result.fail_code = PLAN_FAIL_START_OUT;
        return result;
    }
    if (!WorldToCell(goalWorld.x_cm, goalWorld.y_cm, &goalX, &goalY)) {
        result.fail_code = PLAN_FAIL_GOAL_OUT;
        return result;
    }
    if (CellOccupied(startX, startY)) {
        result.fail_code = PLAN_FAIL_START_BLOCKED;
        return result;
    }
    if (CellOccupied(goalX, goalY)) {
        result.fail_code = PLAN_FAIL_GOAL_BLOCKED;
        return result;
    }

    memset(g_nodeState, 0, sizeof(g_nodeState));
    for (i = 0U; i < MAP_NODE_COUNT; ++i) {
        g_cost[i] = 0xFFFFU;
        g_parent[i] = -1;
    }

    startIndex = CellIndex(startX, startY);
    goalIndex = CellIndex(goalX, goalY);
    g_cost[startIndex] = 0U;
    g_nodeState[startIndex] = 1U;
    openCount = 1U;

    while (openCount > 0U) {
        int16_t current = -1;
        uint16_t bestCost = 0xFFFFU;
        int16_t currentX;
        int16_t currentY;
        uint8_t n;

        for (i = 0U; i < MAP_NODE_COUNT; ++i) {
            if ((g_nodeState[i] & 0x01U) != 0U) {
                int16_t nodeX = (int16_t) (i % MAP_WIDTH_CELLS);
                int16_t nodeY = (int16_t) (i / MAP_WIDTH_CELLS);
                uint16_t cost = (uint16_t) (g_cost[i] + HeuristicCost(nodeX, nodeY, goalX, goalY));
                if (cost < bestCost) {
                    bestCost = cost;
                    current = (int16_t) i;
                }
            }
        }

        if (current < 0) {
            break;
        }
        if (current == goalIndex) {
            if (!BuildPathFromParents(startIndex, goalIndex, path)) {
                result.fail_code = PLAN_FAIL_PATH_OVERFLOW;
                return result;
            }
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE && \
    RADAR_R1_A_FORBID_AXIS_EXPAND_ENABLE
            /*
             * A* may legitimately return short diagonal hops around the
             * inflated forbidden rectangle.  The differential-drive tracker is
             * much more stable on the already-verified A-code style motion:
             * finish one map axis, then the other.  Expand each clear diagonal
             * hop into Manhattan sub-goals while still checking the inflated
             * occupancy grid.  This keeps the planner generic, but avoids the
             * car chasing a diagonal/behind waypoint and spinning in place.
             */
            if (!RadarR1_AStarAxisExpandPath(path, g_poseMap.yaw_deg)) {
                result.fail_code = PLAN_FAIL_PATH_OVERFLOW;
                return result;
            }
#if RADAR_R1_ASTAR_HOME_SHORTCUT_ENABLE
            if (!RadarR1_AStarHomeShortcutPath(path)) {
                result.fail_code = PLAN_FAIL_PATH_OVERFLOW;
                return result;
            }
#endif
#endif
            result.ok = true;
            result.fail_code = PLAN_FAIL_NONE;
            result.path_len = path->len;
            return result;
        }

        g_nodeState[current] &= (uint8_t) ~0x01U;
        g_nodeState[current] |= 0x02U;
        openCount--;

        currentX = (int16_t) (current % MAP_WIDTH_CELLS);
        currentY = (int16_t) (current / MAP_WIDTH_CELLS);

        for (n = 0U; n < (uint8_t) (sizeof(offsetX) / sizeof(offsetX[0])); ++n) {
            int16_t nextX = (int16_t) (currentX + offsetX[n]);
            int16_t nextY = (int16_t) (currentY + offsetY[n]);
            int16_t nextIndex;
            uint16_t nextCost;
            uint32_t candidateCost;

            if (RadarR1_AStarCellTooCloseToForbidden(nextX, nextY)) {
                continue;
            }
            if ((offsetX[n] != 0) && (offsetY[n] != 0)) {
                if (RadarR1_AStarCellTooCloseToForbidden((int16_t) (currentX + offsetX[n]), currentY) ||
                    RadarR1_AStarCellTooCloseToForbidden(currentX, (int16_t) (currentY + offsetY[n]))) {
                    continue;
                }
            }

            nextIndex = CellIndex(nextX, nextY);
            if ((g_nodeState[nextIndex] & 0x02U) != 0U) {
                continue;
            }

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            candidateCost = (uint32_t) g_cost[current] + moveCost[n];
            candidateCost += RadarR1_AStarForbiddenSoftCost(nextX, nextY);
            {
                uint16_t targetReward =
                    RadarR1_AStarTargetPocketReward(nextX, nextY);
                candidateCost = (candidateCost > targetReward) ?
                    (candidateCost - targetReward) : 0U;
            }
            {
                int16_t prevDx = 0;
                int16_t prevDy = 0;
                bool fromStart = (current == startIndex);
                if (fromStart) {
                    RadarR1_AStarDirFromYaw(g_poseMap.yaw_deg, &prevDx, &prevDy);
                } else if (g_parent[current] >= 0) {
                    int16_t parent = g_parent[current];
                    int16_t parentX = (int16_t) (parent % MAP_WIDTH_CELLS);
                    int16_t parentY = (int16_t) (parent / MAP_WIDTH_CELLS);
                    prevDx = (int16_t) (currentX - parentX);
                    prevDy = (int16_t) (currentY - parentY);
                }
                if ((prevDx != 0) || (prevDy != 0)) {
                    candidateCost += RadarR1_AStarTurnPenalty(
                        prevDx, prevDy, offsetX[n], offsetY[n], fromStart);
                }
            }
            if (candidateCost > 0xFFFEU) {
                candidateCost = 0xFFFEU;
            }
            nextCost = (uint16_t) candidateCost;
#else
            nextCost = (uint16_t) (g_cost[current] + moveCost[n]);
#endif
            if ((g_nodeState[nextIndex] & 0x01U) == 0U) {
                g_nodeState[nextIndex] |= 0x01U;
                openCount++;
            }
            if (nextCost < g_cost[nextIndex]) {
                g_cost[nextIndex] = nextCost;
                g_parent[nextIndex] = current;
            }
        }
    }

    result.fail_code = PLAN_FAIL_NO_PATH;
    return result;
}

#if !SPEED_C4_TEST_ENABLE
static char MotionTest_ModeChar(void)
{
    if (g_motionTestTrim < 0) {
        return 'L';
    }
    if (g_motionTestTrim > 0) {
        return 'R';
    }
    return 'B';
}

static uint16_t MotionTest_CurrentDurationMs(void)
{
    return MOTION_C35_DURATION_MS;
}
#endif

#if SPEED_C4_OPEN_LOOP_DIAG
static char SpeedC4_SequenceChar(void)
{
    switch (g_speedC4Sequence) {
    case SPEED_C4_SEQUENCE_BASE_PAIR:
        return 'B';
    case SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR:
        return 'R';
    case SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR:
        return 'L';
    default:
        return '-';
    }
}
#endif

static void Oled_ShowMotorSpeed(int16_t left, int16_t right, bool running)
{
    uint8_t keys = 0U;
#if SPEED_C4_TEST_ENABLE && SPEED_C4_OPEN_LOOP_DIAG
    uint8_t pulseIndex = (g_speedC4PulseTotal > 0U) ?
        (uint8_t) (g_speedC4PulseIndex + 1U) : 0U;
    uint16_t runSec = running ?
        (uint16_t) Clamp32((int32_t) ((g_tickMs - g_speedC4StartMs) / 1000U), 0, 9999) : 0U;
#endif

    keys |= Key1_IsPressed() ? 0x01U : 0U;
    keys |= Key2_IsPressed() ? 0x02U : 0U;
    keys |= Key3_IsPressed() ? 0x04U : 0U;
    keys |= Key4_IsPressed() ? 0x08U : 0U;

    Oled_Clear();
#if SPEED_C4_TEST_ENABLE
#if SPEED_C4_OPEN_LOOP_DIAG
    Oled_ShowString(1U, 1U, running ? "C4.11 WIND RUN " : "C4.11 WIND STP ");
    Oled_ShowString(2U, 1U, "D:");
    Oled_ShowChar(2U, 3U, SpeedC4_SequenceChar());
    Oled_ShowString(2U, 5U, "T:");
    Oled_ShowUnsigned4(2U, 7U, runSec);
    Oled_ShowString(2U, 12U, "P:");
    Oled_ShowChar(2U, 14U, (char) ('0' + pulseIndex));
    Oled_ShowChar(2U, 15U, '/');
    Oled_ShowChar(2U, 16U, (char) ('0' + g_speedC4PulseTotal));
#else
    Oled_ShowString(1U, 1U, running ? "C5.1 PID RUN   " : "C5.1 PID STOP  ");
    Oled_ShowString(2U, 1U, "T:");
    Oled_ShowUnsigned4(2U, 3U, (uint16_t) Abs16(g_speedC4TargetCps));
    Oled_ShowString(2U, 8U, "C:");
    Oled_ShowChar(2U, 10U, SpeedC4_ControlChar(g_speedC4Control));
    Oled_ShowString(2U, 12U, "P:");
    Oled_ShowChar(2U, 14U, SpeedC4_PhaseChar(g_speedC4Phase));
#endif
#else
    Oled_ShowString(1U, 1U, running ? "C3.6 ENC RUN    " : "C3.6 ENC STOP   ");
    Oled_ShowString(2U, 1U, "M:");
    Oled_ShowChar(2U, 3U, MotionTest_ModeChar());
    Oled_ShowString(2U, 5U, "P:");
    Oled_ShowUnsigned4(2U, 7U, (uint16_t) Abs16(g_motorTestPwm));
    Oled_ShowString(2U, 12U, "T:");
    Oled_ShowUnsigned4(2U, 14U, MotionTest_CurrentDurationMs());
#endif
    Oled_ShowString(3U, 1U, "L:");
    Oled_ShowSignedPwm(3U, 3U, left);
    Oled_ShowString(3U, 9U, "R:");
    Oled_ShowSignedPwm(3U, 11U, right);
    Oled_ShowString(4U, 1U, "K1234:");
    Oled_ShowChar(4U, 7U, (keys & 0x01U) ? '1' : '0');
    Oled_ShowChar(4U, 8U, (keys & 0x02U) ? '1' : '0');
    Oled_ShowChar(4U, 9U, (keys & 0x04U) ? '1' : '0');
    Oled_ShowChar(4U, 10U, (keys & 0x08U) ? '1' : '0');
}

static void NavSpeedLoop_Reset(uint32_t nowMs)
{
    int16_t left = 0;
    int16_t right = 0;

    Encoder_GetCounts(&left, &right);
    memset(&g_navSpeedLoop, 0, sizeof(g_navSpeedLoop));
    g_navSpeedLoopPrevLeft = left;
    g_navSpeedLoopPrevRight = right;
    g_navSpeedLoopLastMs = nowMs;
    g_navSpeedLoopReady = true;
}

static void NavSpeedLoop_SetTargetFromPwm(int16_t leftCmd, int16_t rightCmd)
{
    /*
     * Transitional interface: the outer pose loop still computes the proven
     * R1 PWM-shaped command.  The first closed-loop log showed that using
     * raw PWM directly as cps target made the loop think both wheels were
     * massively over-speed (measured straight speeds were around thousands
     * of encoder counts/s while targets were around 300).  Use a coarse
     * feed-forward PWM->cps scale here and keep the correction authority
     * bounded; this makes the speed loop a trim layer, not a second
     * path-following controller fighting yaw/line hold.  The first scaled
     * trial felt under-powered because the inner slew/sample were too slow,
     * so this version keeps the outer feed-forward command responsive and
     * only adds/subtracts a small encoder-based correction.
     */
    g_navSpeedLoop.target_l_cps = (int16_t) Clamp32(
        ((int32_t) leftCmd * NAV_SPEED_LOOP_TARGET_L_NUM) /
            NAV_SPEED_LOOP_TARGET_L_DEN,
        -32768, 32767);
    g_navSpeedLoop.target_r_cps = (int16_t) Clamp32(
        ((int32_t) rightCmd * NAV_SPEED_LOOP_TARGET_R_NUM) /
            NAV_SPEED_LOOP_TARGET_R_DEN,
        -32768, 32767);
    g_navSpeedLoop.pwm_ff_l = leftCmd;
    g_navSpeedLoop.pwm_ff_r = rightCmd;
    if (((g_navSpeedLoop.pwm_l <= 0) && (leftCmd > 0)) ||
        ((g_navSpeedLoop.pwm_l >= 0) && (leftCmd < 0))) {
        g_navSpeedLoop.pwm_l = leftCmd;
    }
    if (((g_navSpeedLoop.pwm_r <= 0) && (rightCmd > 0)) ||
        ((g_navSpeedLoop.pwm_r >= 0) && (rightCmd < 0))) {
        g_navSpeedLoop.pwm_r = rightCmd;
    }
}

static int16_t NavSpeedLoop_LimitStep(int16_t previous, int16_t desired)
{
    int32_t delta = (int32_t) desired - previous;

    if (previous == 0) {
        return desired;
    }
    if (delta > NAV_SPEED_LOOP_PWM_STEP_LIMIT) {
        return (int16_t) (previous + NAV_SPEED_LOOP_PWM_STEP_LIMIT);
    }
    if (delta < -NAV_SPEED_LOOP_PWM_STEP_LIMIT) {
        return (int16_t) (previous - NAV_SPEED_LOOP_PWM_STEP_LIMIT);
    }
    return desired;
}

static int16_t NavSpeedLoop_ClampSignedPwm(int32_t pwm, uint8_t *sat)
{
    *sat = 0U;
    if (pwm > RADAR_R1_MAX_PWM) {
        pwm = RADAR_R1_MAX_PWM;
        *sat = 1U;
    } else if (pwm < -RADAR_R1_MAX_PWM) {
        pwm = -RADAR_R1_MAX_PWM;
        *sat = 1U;
    }
    if ((pwm > 0) && (pwm < RADAR_R1_MIN_PWM)) {
        pwm = RADAR_R1_MIN_PWM;
    } else if ((pwm < 0) && (pwm > -RADAR_R1_MIN_PWM)) {
        pwm = -RADAR_R1_MIN_PWM;
    }
    return (int16_t) pwm;
}

static bool NavSpeedLoop_SameDirectionNonZero(void)
{
    return (((g_navSpeedLoop.target_l_cps > 0) &&
             (g_navSpeedLoop.target_r_cps > 0)) ||
            ((g_navSpeedLoop.target_l_cps < 0) &&
             (g_navSpeedLoop.target_r_cps < 0)));
}

static bool NavSpeedLoop_CanCloseFromCmd(int16_t leftCmd, int16_t rightCmd)
{
    /*
     * Current navigation speed loop is only a small straight-line trim layer:
     * the outer loop still outputs proven PWM-shaped commands, and encoder
     * feedback only trims same-direction wheel motion.  Do not close this
     * transitional loop during in-place turns or opposite-sign heading fixes;
     * the latest log showed those phases became asymmetric and over-active.
     */
    return (((leftCmd > 0) && (rightCmd > 0)) ||
            ((leftCmd < 0) && (rightCmd < 0)));
}

static void NavSpeedLoop_Update(uint32_t nowMs, bool motorEnable)
{
    int16_t countLeft = 0;
    int16_t countRight = 0;
    uint32_t dt;
    int16_t speedLeft;
    int16_t speedRightRaw;
    int16_t speedRightNorm;
    int32_t correctionLeft;
    int32_t correctionRight;
    int32_t sync = 0;
    int32_t desiredLeft;
    int32_t desiredRight;
    uint8_t satLeft = 0U;
    uint8_t satRight = 0U;

    if (!motorEnable ||
        ((g_navSpeedLoop.target_l_cps == 0) &&
         (g_navSpeedLoop.target_r_cps == 0))) {
        g_navSpeedLoop.enabled = 0U;
        g_navSpeedLoop.pwm_l = 0;
        g_navSpeedLoop.pwm_r = 0;
        NavSpeedLoop_Reset(nowMs);
        if (motorEnable) {
            Motor_Stop();
        } else {
            Motor_SetOutputDry(0, 0);
        }
        return;
    }

    if (!g_navSpeedLoopReady) {
        NavSpeedLoop_Reset(nowMs);
    }

    dt = nowMs - g_navSpeedLoopLastMs;
    if (dt < NAV_SPEED_LOOP_SAMPLE_MS) {
        if (motorEnable) {
            Motor_SetOutput(g_navSpeedLoop.pwm_l, g_navSpeedLoop.pwm_r);
        } else {
            Motor_SetOutputDry(g_navSpeedLoop.pwm_l, g_navSpeedLoop.pwm_r);
        }
        return;
    }

    Encoder_GetCounts(&countLeft, &countRight);
    speedLeft = (int16_t) ((((int32_t) countLeft - g_navSpeedLoopPrevLeft) *
                           1000) / (int32_t) dt);
    speedRightRaw = (int16_t) ((((int32_t) countRight - g_navSpeedLoopPrevRight) *
                              1000) / (int32_t) dt);
    speedRightNorm = (int16_t) Clamp32(
        ((int32_t) speedRightRaw * SPEED_C4_ENCODER_R_SCALE_NUM) /
            SPEED_C4_ENCODER_R_SCALE_DEN,
        -32768, 32767);

    g_navSpeedLoopPrevLeft = countLeft;
    g_navSpeedLoopPrevRight = countRight;
    g_navSpeedLoopLastMs = nowMs;

    g_navSpeedLoop.speed_l_cps = speedLeft;
    g_navSpeedLoop.speed_r_cps = speedRightNorm;
    g_navSpeedLoop.err_l = (int16_t) (g_navSpeedLoop.target_l_cps - speedLeft);
    g_navSpeedLoop.err_r = (int16_t) (g_navSpeedLoop.target_r_cps - speedRightNorm);
    correctionLeft = Clamp32((int32_t) g_navSpeedLoop.err_l / NAV_SPEED_LOOP_KP_DIV,
                             -NAV_SPEED_LOOP_CORRECTION_LIMIT,
                             NAV_SPEED_LOOP_CORRECTION_LIMIT);
    correctionRight = Clamp32((int32_t) g_navSpeedLoop.err_r / NAV_SPEED_LOOP_KP_DIV,
                              -NAV_SPEED_LOOP_CORRECTION_LIMIT,
                              NAV_SPEED_LOOP_CORRECTION_LIMIT);

    if (NavSpeedLoop_SameDirectionNonZero()) {
#if NAV_SPEED_LOOP_DISTANCE_SYNC_DIV > 0
        int32_t scaledRightCount =
            ((int32_t) countRight * SPEED_C4_ENCODER_R_SCALE_NUM) /
            SPEED_C4_ENCODER_R_SCALE_DEN;
        sync = ((int32_t) countLeft - scaledRightCount) /
            NAV_SPEED_LOOP_DISTANCE_SYNC_DIV;
        sync = Clamp32(sync, -NAV_SPEED_LOOP_SYNC_LIMIT,
                       NAV_SPEED_LOOP_SYNC_LIMIT);
#endif
    }

    desiredLeft = (int32_t) g_navSpeedLoop.pwm_ff_l + correctionLeft - sync;
    desiredRight = (int32_t) g_navSpeedLoop.pwm_ff_r + correctionRight + sync;

    g_navSpeedLoop.pwm_l = NavSpeedLoop_LimitStep(
        g_navSpeedLoop.pwm_l,
        NavSpeedLoop_ClampSignedPwm(desiredLeft, &satLeft));
    g_navSpeedLoop.pwm_r = NavSpeedLoop_LimitStep(
        g_navSpeedLoop.pwm_r,
        NavSpeedLoop_ClampSignedPwm(desiredRight, &satRight));
    g_navSpeedLoop.sync = (int16_t) sync;
    g_navSpeedLoop.sat_l = satLeft;
    g_navSpeedLoop.sat_r = satRight;
    g_navSpeedLoop.enabled = 1U;

    if (motorEnable) {
        Motor_SetOutput(g_navSpeedLoop.pwm_l, g_navSpeedLoop.pwm_r);
    } else {
        Motor_SetOutputDry(g_navSpeedLoop.pwm_l, g_navSpeedLoop.pwm_r);
    }
}

static const SpeedLoopStatus *NavSpeedLoop_GetStatus(void)
{
    return &g_navSpeedLoop;
}

static void OledText_Clear16(char *text)
{
    uint8_t i;

    for (i = 0U; i < 16U; ++i) {
        text[i] = ' ';
    }
    text[16] = '\0';
}

static void OledText_CopyAt(char *text, uint8_t column, const char *src)
{
    uint8_t index;

    if ((column < 1U) || (column > 16U)) {
        return;
    }

    index = (uint8_t) (column - 1U);
    while ((*src != '\0') && (index < 16U)) {
        text[index++] = *src++;
    }
}

static void OledText_CharAt(char *text, uint8_t column, char ch)
{
    if ((column >= 1U) && (column <= 16U)) {
        text[column - 1U] = ch;
    }
}

static void OledText_UnsignedAt(char *text, uint8_t column,
                                uint32_t value, uint8_t width)
{
    uint8_t i;
    uint32_t limit = 1U;

    if ((column < 1U) || (column > 16U) || (width == 0U)) {
        return;
    }
    for (i = 0U; i < width; ++i) {
        limit *= 10U;
    }
    if (value >= limit) {
        value = limit - 1U;
    }

    for (i = 0U; i < width; ++i) {
        uint8_t pos = (uint8_t) (column + width - 1U - i);
        if (pos <= 16U) {
            text[pos - 1U] = (char) ('0' + (value % 10U));
        }
        value /= 10U;
    }
}

static void OledText_Signed4At(char *text, uint8_t column, int32_t value)
{
    uint32_t absValue;

    if (column > 16U) {
        return;
    }
    OledText_CharAt(text, column, (value >= 0) ? '+' : '-');
    absValue = (value >= 0) ? (uint32_t) value : (uint32_t) (-value);
    OledText_UnsignedAt(text, (uint8_t) (column + 1U),
                        (uint32_t) Clamp32((int32_t) absValue, 0, 9999), 4U);
}

static void OledText_SignedPwmAt(char *text, uint8_t column, int16_t value)
{
    OledText_Signed4At(text, column, value);
}

static void Motor_UpdateStatusLeds(void)
{
    if (MOTOR_DIR_TEST_ENABLE != 0) {
        return;
    }

    if ((g_navState == NAV_PLAN_TO_TARGET) || (g_navState == NAV_GO_TARGET)) {
        DL_GPIO_setPins(LED1_PORT, LED1_led1_PIN);
        DL_GPIO_setPins(LED2_PORT, LED2_led2_PIN);
    } else if (g_navState == NAV_WAIT_POSE) {
        DL_GPIO_clearPins(LED1_PORT, LED1_led1_PIN);
        DL_GPIO_setPins(LED2_PORT, LED2_led2_PIN);
    } else if (g_navState == NAV_ERROR) {
        if ((g_tickMs & 0x80U) != 0U) {
            DL_GPIO_setPins(LED1_PORT, LED1_led1_PIN);
            DL_GPIO_clearPins(LED2_PORT, LED2_led2_PIN);
        } else {
            DL_GPIO_clearPins(LED1_PORT, LED1_led1_PIN);
            DL_GPIO_setPins(LED2_PORT, LED2_led2_PIN);
        }
    } else {
        DL_GPIO_setPins(LED1_PORT, LED1_led1_PIN);
        DL_GPIO_clearPins(LED2_PORT, LED2_led2_PIN);
    }
}

static void Motor_Stop(void)
{
    NavSpeedLoop_Reset(g_tickMs);
    MotorDriver_StopRaw();
}

#if !SPEED_C4_TEST_ENABLE
static void Motor_LogChain(uint32_t nowMs, uint32_t step)
{
    int16_t encLeft = 0;
    int16_t encRight = 0;
    uint16_t duration = MotionTest_CurrentDurationMs();

    (void) step;
    Encoder_GetCounts(&encLeft, &encRight);
    DebugUart_WriteString("encoder_c36.log,");
    DebugUart_WriteInt((int32_t) nowMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_motionTestRunId);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_motorTestPwm);
    DebugUart_WriteChar(',');
    DebugUart_WriteChar(MotionTest_ModeChar());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) MotorDriver_GetLastLeft());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) MotorDriver_GetLastRight());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) MotionTest_CurrentDurationMs());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) encLeft);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) encRight);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((duration == 0U) ? 0 : (int32_t) encLeft * 1000 / duration);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((duration == 0U) ? 0 : (int32_t) encRight * 1000 / duration);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) Encoder_ReadLeftState());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) Encoder_ReadRightState());
    DebugUart_WriteChar(',');
    DebugUart_WriteString(g_motionTestPulse ? "RUN" : "STOP");
    DebugUart_WriteString("\r\n");
}
#endif

#if !SPEED_C4_TEST_ENABLE
static const char *MotionC35_EventText(MotionC35Event event)
{
    switch (event) {
    case MOTION_C35_EVENT_BOOT:
        return "BOOT";
    case MOTION_C35_EVENT_START:
        return "START";
    case MOTION_C35_EVENT_STOP:
        return "STOP";
    case MOTION_C35_EVENT_TRIM:
        return "TRIM";
    case MOTION_C35_EVENT_DIR:
        return "DIR";
    case MOTION_C35_EVENT_RUN:
        return "RUN";
    case MOTION_C35_EVENT_EXPORT:
        return "EXPORT";
    default:
        return "UNK";
    }
}

static void MotionC35_LogAppend(uint32_t nowMs, MotionC35Event event,
                                int16_t pwmLeft, int16_t pwmRight)
{
    MotionC35LogEntry *entry = &g_motionC35Logs[g_motionC35LogIndex];
    int16_t encLeft = 0;
    int16_t encRight = 0;
    uint16_t duration = MotionTest_CurrentDurationMs();

    Encoder_GetCounts(&encLeft, &encRight);

    entry->time_ms = nowMs;
    entry->run_id = g_motionTestRunId;
    entry->base_pwm = g_motorTestPwm;
    entry->trim = g_motionTestTrim;
    entry->pwm_l = pwmLeft;
    entry->pwm_r = pwmRight;
    entry->duration_ms = duration;
    entry->enc_l_delta = encLeft;
    entry->enc_r_delta = encRight;
    entry->speed_l = (duration == 0U) ? 0 : (int16_t) (((int32_t) encLeft * 1000) / duration);
    entry->speed_r = (duration == 0U) ? 0 : (int16_t) (((int32_t) encRight * 1000) / duration);
    entry->left_ab = Encoder_ReadLeftState();
    entry->right_ab = Encoder_ReadRightState();
    entry->event = event;

    g_motionC35LogIndex = (uint8_t) ((g_motionC35LogIndex + 1U) % MOTION_C35_LOG_CAPACITY);
    if (g_motionC35LogCount < MOTION_C35_LOG_CAPACITY) {
        ++g_motionC35LogCount;
    } else {
        ++g_motionC35LogDropped;
    }
}
#endif

#if !SPEED_C4_TEST_ENABLE
static void MotionC35_LogExport(uint32_t nowMs)
{
    uint8_t start;
    uint8_t i;

    MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_EXPORT, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight());
    DebugUart_WriteString("encoder_c36.export begin count=");
    DebugUart_WriteInt((int32_t) g_motionC35LogCount);
    DebugUart_WriteString(" dropped=");
    DebugUart_WriteInt((int32_t) g_motionC35LogDropped);
    DebugUart_WriteString("\r\n");
    DebugUart_WriteString("encoder_c36.header,time_ms,run_id,pwm,mode,pwm_l,pwm_r,duration_ms,enc_l_delta,enc_r_delta,speed_l,speed_r,left_ab,right_ab,event\r\n");

    start = (g_motionC35LogCount == MOTION_C35_LOG_CAPACITY) ? g_motionC35LogIndex : 0U;
    for (i = 0U; i < g_motionC35LogCount; ++i) {
        uint8_t index = (uint8_t) ((start + i) % MOTION_C35_LOG_CAPACITY);
        MotionC35LogEntry *entry = &g_motionC35Logs[index];

        DebugUart_WriteString("encoder_c36.log,");
        DebugUart_WriteInt((int32_t) entry->time_ms);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->run_id);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->base_pwm);
        DebugUart_WriteChar(',');
        if (entry->trim < 0) {
            DebugUart_WriteChar('L');
        } else if (entry->trim > 0) {
            DebugUart_WriteChar('R');
        } else {
            DebugUart_WriteChar('B');
        }
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pwm_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pwm_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->duration_ms);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->enc_l_delta);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->enc_r_delta);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->speed_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->speed_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->left_ab);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->right_ab);
        DebugUart_WriteChar(',');
        DebugUart_WriteString(MotionC35_EventText(entry->event));
        DebugUart_WriteString("\r\n");
    }

    DebugUart_WriteString("encoder_c36.export end\r\n");
}
#endif

static char SpeedC4_ModeChar(SpeedC4Mode mode)
{
    switch (mode) {
    case SPEED_C4_MODE_LEFT:
        return 'L';
    case SPEED_C4_MODE_RIGHT:
        return 'R';
    case SPEED_C4_MODE_BOTH:
        return 'B';
    default:
        return '-';
    }
}

static char SpeedC4_ControlChar(SpeedC4Control control)
{
    switch (control) {
    case SPEED_C4_CONTROL_OPEN:
        return 'O';
    case SPEED_C4_CONTROL_SPEED_P:
        return 'P';
    case SPEED_C4_CONTROL_SPEED_P_DIST:
        return 'D';
    case SPEED_C4_CONTROL_DISTURB_LEFT:
        return 'L';
    case SPEED_C4_CONTROL_DISTURB_RIGHT:
        return 'R';
    default:
        return '-';
    }
}

static char SpeedC4_PhaseChar(SpeedC4Phase phase)
{
    switch (phase) {
    case SPEED_C4_PHASE_BASE:
        return 'B';
    case SPEED_C4_PHASE_DISTURB:
        return 'X';
    case SPEED_C4_PHASE_RECOVER:
        return 'R';
    case SPEED_C4_PHASE_IDLE:
    default:
        return '-';
    }
}

static const char *SpeedC4_EventText(SpeedC4Event event)
{
    switch (event) {
    case SPEED_C4_EVENT_BOOT:
        return "BOOT";
    case SPEED_C4_EVENT_TARGET:
        return "TARGET";
    case SPEED_C4_EVENT_START:
        return "START";
    case SPEED_C4_EVENT_PHASE:
        return "PHASE";
    case SPEED_C4_EVENT_RUN:
        return "RUN";
    case SPEED_C4_EVENT_STOP:
        return "STOP";
    case SPEED_C4_EVENT_EXPORT:
        return "EXPORT";
    default:
        return "UNK";
    }
}

static bool SpeedC4_IsDisturbControl(SpeedC4Control control)
{
    return ((control == SPEED_C4_CONTROL_DISTURB_LEFT) ||
            (control == SPEED_C4_CONTROL_DISTURB_RIGHT));
}

static int16_t SpeedC4_RightRawTargetFromNorm(int16_t normTarget)
{
    return (int16_t) Clamp32(
        (((int32_t) normTarget * SPEED_C4_ENCODER_R_SCALE_DEN) +
            (SPEED_C4_ENCODER_R_SCALE_NUM / 2)) /
            SPEED_C4_ENCODER_R_SCALE_NUM,
        0, 32767);
}

static SpeedC4Phase SpeedC4_PhaseForElapsed(uint32_t elapsedMs)
{
    if (elapsedMs < SPEED_C5_DISTURB_BASE_MS) {
        return SPEED_C4_PHASE_BASE;
    }
    if (elapsedMs < (SPEED_C5_DISTURB_BASE_MS + SPEED_C5_DISTURB_HOLD_MS)) {
        return SPEED_C4_PHASE_DISTURB;
    }
    return SPEED_C4_PHASE_RECOVER;
}

static uint32_t SpeedC4_RunDurationMs(void)
{
    return SpeedC4_IsDisturbControl(g_speedC4Control) ?
        SPEED_C5_DISTURB_TOTAL_MS : SPEED_C4_DURATION_MS;
}

static int16_t SpeedC4_TargetForLeft(void)
{
    int16_t target = g_speedC4TargetCps;

    if ((g_speedC4Control == SPEED_C4_CONTROL_DISTURB_LEFT) &&
        (g_speedC4Phase == SPEED_C4_PHASE_DISTURB)) {
        target = 0;
    } else if ((g_speedC4Control == SPEED_C4_CONTROL_DISTURB_RIGHT) &&
        (g_speedC4Phase == SPEED_C4_PHASE_DISTURB)) {
        target = SPEED_C5_DISTURB_LOW_TARGET_CPS;
    }

    return ((g_speedC4Mode == SPEED_C4_MODE_LEFT) ||
            (g_speedC4Mode == SPEED_C4_MODE_BOTH)) ? target : 0;
}

static int16_t SpeedC4_TargetForRight(void)
{
    int16_t target = g_speedC4TargetCps;

    if ((g_speedC4Mode != SPEED_C4_MODE_RIGHT) &&
        (g_speedC4Mode != SPEED_C4_MODE_BOTH)) {
        return 0;
    }

#if SPEED_C4_OPEN_LOOP_DIAG
    return g_speedC4TargetCps;
#else
    if ((g_speedC4Control == SPEED_C4_CONTROL_DISTURB_RIGHT) &&
        (g_speedC4Phase == SPEED_C4_PHASE_DISTURB)) {
        return 0;
    }
    if ((g_speedC4Control == SPEED_C4_CONTROL_DISTURB_LEFT) &&
        (g_speedC4Phase == SPEED_C4_PHASE_DISTURB)) {
        target = SPEED_C5_DISTURB_LOW_TARGET_CPS;
    }
    return SpeedC4_RightRawTargetFromNorm(target);
#endif
}

#if !SPEED_C4_OPEN_LOOP_DIAG
static int16_t SpeedC4_BasePwmForLeft(void)
{
    if (g_speedC4Phase == SPEED_C4_PHASE_DISTURB) {
        if (g_speedC4Control == SPEED_C4_CONTROL_DISTURB_LEFT) {
            return 0;
        }
        if (g_speedC4Control == SPEED_C4_CONTROL_DISTURB_RIGHT) {
            return SPEED_C5_DISTURB_LOW_PWM_L;
        }
    }

    return (g_speedC4TargetCps >= SPEED_C4_TARGET_HIGH_CPS) ?
        SPEED_C4_HIGH_BASE_PWM_L : SPEED_C4_BASE_PWM_L;
}

static int16_t SpeedC4_BasePwmForRight(void)
{
    if (g_speedC4Phase == SPEED_C4_PHASE_DISTURB) {
        if (g_speedC4Control == SPEED_C4_CONTROL_DISTURB_RIGHT) {
            return 0;
        }
        if (g_speedC4Control == SPEED_C4_CONTROL_DISTURB_LEFT) {
            return SPEED_C5_DISTURB_LOW_PWM_R;
        }
    }

    return (g_speedC4TargetCps >= SPEED_C4_TARGET_HIGH_CPS) ?
        SPEED_C4_HIGH_BASE_PWM_R : SPEED_C4_BASE_PWM_R;
}

static int16_t SpeedC4_ComputeSpeedCorrection(
    int16_t target, int16_t speed, uint32_t dtMs, int32_t *integral)
{
    int32_t error;
    int32_t correction;

    if (target == 0) {
        *integral = 0;
        return 0;
    }

    error = (int32_t) target - speed;
    (void) dtMs;
#if SPEED_C4_KI_DIV > 0
    *integral = Clamp32(*integral + (error * (int32_t) dtMs),
        -SPEED_C4_INTEGRAL_LIMIT, SPEED_C4_INTEGRAL_LIMIT);
    correction = (error / SPEED_C4_KP_DIV) + (*integral / SPEED_C4_KI_DIV);
#else
    *integral = 0;
    correction = error / SPEED_C4_KP_DIV;
#endif
    correction = Clamp32(correction, -SPEED_C4_CORRECTION_LIMIT, SPEED_C4_CORRECTION_LIMIT);
    return (int16_t) correction;
}

static int16_t SpeedC4_ClampPwmWithSat(int32_t pwm, uint8_t *sat)
{
    *sat = 0U;
    if (pwm > SPEED_C4_PWM_MAX) {
        pwm = SPEED_C4_PWM_MAX;
        *sat = 1U;
    } else if (pwm < SPEED_C4_PWM_MIN) {
        pwm = SPEED_C4_PWM_MIN;
        *sat = 1U;
    }

    return (int16_t) pwm;
}

static int16_t SpeedC4_LimitPwmStep(int16_t current, int16_t desired)
{
    int32_t delta = (int32_t) desired - current;

    if (current == 0) {
        return desired;
    }
    if (delta > SPEED_C4_PWM_STEP_LIMIT) {
        return (int16_t) (current + SPEED_C4_PWM_STEP_LIMIT);
    }
    if (delta < -SPEED_C4_PWM_STEP_LIMIT) {
        return (int16_t) (current - SPEED_C4_PWM_STEP_LIMIT);
    }
    return desired;
}
#endif

static void SpeedC4_LogAppend(uint32_t nowMs, SpeedC4Event event,
                              int16_t targetLeft, int16_t targetRight,
                              int16_t speedLeft, int16_t speedRight,
                              int16_t pwmLeft, int16_t pwmRight,
                              int16_t sync, uint8_t satLeft, uint8_t satRight)
{
    SpeedC4LogEntry *entry = &g_speedC4Logs[g_speedC4LogIndex];
    int16_t countLeft;
    int16_t countRight;

    Encoder_GetCounts(&countLeft, &countRight);

    entry->time_ms = nowMs;
    entry->run_id = g_speedC4RunId;
#if SPEED_C4_OPEN_LOOP_DIAG
    entry->pulse_index = (g_speedC4PulseTotal > 0U) ?
        (uint8_t) (g_speedC4PulseIndex + 1U) : 0U;
    entry->pulse_total = g_speedC4PulseTotal;
#else
    entry->pulse_index = 0U;
    entry->pulse_total = 0U;
#endif
    entry->elapsed_ms = g_speedC4Run ?
        (uint16_t) Clamp32((int32_t) (nowMs - g_speedC4StartMs), 0, 65535) : 0U;
    entry->mode = g_speedC4Mode;
    entry->control = g_speedC4Control;
    entry->phase = g_speedC4Phase;
    entry->target_l = targetLeft;
    entry->target_r = targetRight;
    entry->speed_l = speedLeft;
    entry->speed_r = speedRight;
    entry->err_l = (int16_t) (targetLeft - speedLeft);
    entry->err_r = (int16_t) (targetRight - speedRight);
    entry->pwm_l = pwmLeft;
    entry->pwm_r = pwmRight;
    entry->count_l = countLeft;
    entry->count_r = countRight;
    entry->sync = sync;
    entry->norm_err = (int16_t) Clamp32(
        (int32_t) countLeft -
            (((int32_t) countRight * SPEED_C4_ENCODER_R_SCALE_NUM) /
                SPEED_C4_ENCODER_R_SCALE_DEN),
        -32768, 32767);
    entry->yaw_delta = g_poseMap.valid ?
        NormalizeDeg((int16_t) (g_poseMap.yaw_deg - g_speedC4YawStartDeg)) : 0;
#if SPEED_C4_KI_DIV > 0
    entry->i_l = (int16_t) Clamp32(g_speedC4IntegralLeft / SPEED_C4_KI_DIV, -32768, 32767);
    entry->i_r = (int16_t) Clamp32(g_speedC4IntegralRight / SPEED_C4_KI_DIV, -32768, 32767);
#else
    entry->i_l = 0;
    entry->i_r = 0;
#endif
    entry->sat_l = satLeft;
    entry->sat_r = satRight;
    entry->left_ab = Encoder_ReadLeftState();
    entry->right_ab = Encoder_ReadRightState();
    entry->event = event;

    g_speedC4LogIndex = (uint8_t) ((g_speedC4LogIndex + 1U) % MOTION_C35_LOG_CAPACITY);
    if (g_speedC4LogCount < MOTION_C35_LOG_CAPACITY) {
        ++g_speedC4LogCount;
    } else {
        ++g_speedC4LogDropped;
    }
}

static void SpeedC4_LogExport(uint32_t nowMs)
{
    uint8_t start;
    uint8_t i;

    SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_EXPORT,
        SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
        0, 0, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), 0, 0U, 0U);

    DebugUart_WriteString("speed_c4.export begin count=");
    DebugUart_WriteInt((int32_t) g_speedC4LogCount);
    DebugUart_WriteString(" dropped=");
    DebugUart_WriteInt((int32_t) g_speedC4LogDropped);
    DebugUart_WriteString("\r\n");
    DebugUart_WriteString("speed_c4.header,time_ms,run_id,pulse_index,pulse_total,elapsed_ms,mode,control,phase,target_l,target_r,speed_l,speed_r,err_l,err_r,pwm_l,pwm_r,count_l,count_r,sync,norm_err,yaw_delta,i_l,i_r,sat_l,sat_r,left_ab,right_ab,event\r\n");

    start = (g_speedC4LogCount == MOTION_C35_LOG_CAPACITY) ? g_speedC4LogIndex : 0U;
    for (i = 0U; i < g_speedC4LogCount; ++i) {
        uint8_t index = (uint8_t) ((start + i) % MOTION_C35_LOG_CAPACITY);
        SpeedC4LogEntry *entry = &g_speedC4Logs[index];

        DebugUart_WriteString("speed_c4.log,");
        DebugUart_WriteInt((int32_t) entry->time_ms);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->run_id);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pulse_index);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pulse_total);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->elapsed_ms);
        DebugUart_WriteChar(',');
        DebugUart_WriteChar(SpeedC4_ModeChar(entry->mode));
        DebugUart_WriteChar(',');
        DebugUart_WriteChar(SpeedC4_ControlChar(entry->control));
        DebugUart_WriteChar(',');
        DebugUart_WriteChar(SpeedC4_PhaseChar(entry->phase));
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->target_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->target_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->speed_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->speed_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->err_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->err_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pwm_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->pwm_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->count_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->count_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->sync);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->norm_err);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->yaw_delta);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->i_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->i_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->sat_l);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->sat_r);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->left_ab);
        DebugUart_WriteChar(',');
        DebugUart_WriteInt((int32_t) entry->right_ab);
        DebugUart_WriteChar(',');
        DebugUart_WriteString(SpeedC4_EventText(entry->event));
        DebugUart_WriteString("\r\n");
    }

    DebugUart_WriteString("speed_c4.export end\r\n");
}

#if !SPEED_C4_OPEN_LOOP_DIAG
static void SpeedC4_ResetPidAtPhaseBoundary(uint32_t nowMs)
{
    int16_t countLeft;
    int16_t countRight;

    Encoder_GetCounts(&countLeft, &countRight);
    g_speedC4PrevLeft = countLeft;
    g_speedC4PrevRight = countRight;
    g_speedC4LastSampleMs = nowMs;
    g_speedC4LastSyncCorrection = 0;
    g_speedC4IntegralLeft = 0;
    g_speedC4IntegralRight = 0;
}

static void SpeedC4_ApplyPhaseBaseOutput(void)
{
    g_speedC4PwmLeft = SpeedC4_TargetForLeft() ? SpeedC4_BasePwmForLeft() : 0;
    g_speedC4PwmRight = SpeedC4_TargetForRight() ? SpeedC4_BasePwmForRight() : 0;
    Motor_SetOutput(g_speedC4PwmLeft, g_speedC4PwmRight);
}

static void SpeedC4_ServiceDisturbPhase(uint32_t nowMs)
{
    SpeedC4Phase nextPhase;

    if (!g_speedC4Run || !SpeedC4_IsDisturbControl(g_speedC4Control)) {
        return;
    }

    nextPhase = SpeedC4_PhaseForElapsed(nowMs - g_speedC4StartMs);
    if (nextPhase == g_speedC4Phase) {
        return;
    }

    g_speedC4Phase = nextPhase;
    SpeedC4_ResetPidAtPhaseBoundary(nowMs);
    SpeedC4_ApplyPhaseBaseOutput();
    SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_PHASE,
        SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
        0, 0, g_speedC4PwmLeft, g_speedC4PwmRight, 0, 0U, 0U);
}
#endif

static void SpeedC4_Start(uint32_t nowMs, SpeedC4Mode mode)
{
    g_speedC4Run = true;
    g_speedC4Mode = mode;
    g_speedC4Phase = SPEED_C4_PHASE_BASE;
    g_speedC4StartMs = nowMs;
    g_speedC4LastSampleMs = nowMs;
    g_speedC4LastOledMs = nowMs;
#if !SPEED_C4_OPEN_LOOP_DIAG
    ++g_speedC4RunId;
#endif
    Encoder_ResetCounts();
    g_speedC4YawStartDeg = g_poseMap.valid ? g_poseMap.yaw_deg : 0;
    g_speedC4PrevLeft = 0;
    g_speedC4PrevRight = 0;
    g_speedC4LastSyncCorrection = 0;
    g_speedC4IntegralLeft = 0;
    g_speedC4IntegralRight = 0;
#if SPEED_C4_OPEN_LOOP_DIAG
    g_speedC4PwmLeft = SpeedC4_TargetForLeft() ? g_speedC4OpenPwmLeft : 0;
    g_speedC4PwmRight = SpeedC4_TargetForRight() ? g_speedC4OpenPwmRight : 0;
#else
    g_speedC4PwmLeft = SpeedC4_TargetForLeft() ? SpeedC4_BasePwmForLeft() : 0;
    g_speedC4PwmRight = SpeedC4_TargetForRight() ? SpeedC4_BasePwmForRight() : 0;
#endif
    Motor_SetOutput(g_speedC4PwmLeft, g_speedC4PwmRight);
    SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_START,
        SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
        0, 0, g_speedC4PwmLeft, g_speedC4PwmRight, 0, 0U, 0U);
}

#if !SPEED_C4_OPEN_LOOP_DIAG
static void SpeedC4_SelectTarget(uint32_t nowMs, int16_t targetCps, SpeedC4Control control)
{
    if (g_speedC4Run) {
        return;
    }
    g_speedC4TargetCps = targetCps;
    g_speedC4Control = control;
    g_speedC4Mode = SPEED_C4_MODE_NONE;
    g_speedC4Phase = SPEED_C4_PHASE_IDLE;
    SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_TARGET,
        targetCps, targetCps, 0, 0,
        SpeedC4_BasePwmForLeft(), SpeedC4_BasePwmForRight(), 0, 0U, 0U);
}
#endif

#if SPEED_C4_OPEN_LOOP_DIAG
static uint8_t SpeedC4_PulseTotalForSequence(SpeedC4Sequence sequence)
{
    switch (sequence) {
    case SPEED_C4_SEQUENCE_BASE_PAIR:
    case SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR:
    case SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR:
        return SPEED_C4_PULSE_PAIR_COUNT;
    default:
        return 0U;
    }
}

static int16_t SpeedC4_LeftPwmForSequence(SpeedC4Sequence sequence)
{
    switch (sequence) {
    case SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR:
        return SPEED_C4_PULSE_RIGHT_MORE_L;
    case SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR:
        return SPEED_C4_PULSE_RIGHT_LESS_L;
    case SPEED_C4_SEQUENCE_BASE_PAIR:
    default:
        return SPEED_C4_PULSE_BASE_L;
    }
}

static int16_t SpeedC4_RightPwmForSequence(SpeedC4Sequence sequence)
{
    switch (sequence) {
    case SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR:
        return SPEED_C4_PULSE_RIGHT_MORE_R;
    case SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR:
        return SPEED_C4_PULSE_RIGHT_LESS_R;
    case SPEED_C4_SEQUENCE_BASE_PAIR:
    default:
        return SPEED_C4_PULSE_BASE_R;
    }
}

static bool SpeedC4_IsPulseBusy(void)
{
    return (g_speedC4Run ||
            g_speedC4PulseGap ||
            (g_speedC4Sequence != SPEED_C4_SEQUENCE_NONE));
}

static void SpeedC4_ResetPulseSequence(void)
{
    g_speedC4Sequence = SPEED_C4_SEQUENCE_NONE;
    g_speedC4PulseGap = false;
    g_speedC4PulseIndex = 0U;
    g_speedC4PulseTotal = 0U;
    g_speedC4PulseGapStartMs = 0U;
}

static void SpeedC4_StartCurrentPulse(uint32_t nowMs)
{
    if (g_speedC4Sequence == SPEED_C4_SEQUENCE_NONE) {
        SpeedC4_ResetPulseSequence();
        return;
    }

    g_speedC4TargetCps = SPEED_C4_PULSE_PWM;
    g_speedC4OpenPwmLeft = SpeedC4_LeftPwmForSequence(g_speedC4Sequence);
    g_speedC4OpenPwmRight = SpeedC4_RightPwmForSequence(g_speedC4Sequence);
    SpeedC4_Start(nowMs, SPEED_C4_MODE_BOTH);
}

static void SpeedC4_BeginPulseSequence(uint32_t nowMs, SpeedC4Sequence sequence)
{
    if (SpeedC4_IsPulseBusy()) {
        return;
    }

    g_speedC4Sequence = sequence;
    g_speedC4PulseIndex = 0U;
    g_speedC4PulseTotal = SpeedC4_PulseTotalForSequence(sequence);
    g_speedC4PulseGap = false;
    ++g_speedC4RunId;

    if (g_speedC4PulseTotal == 0U) {
        SpeedC4_ResetPulseSequence();
        return;
    }

    SpeedC4_StartCurrentPulse(nowMs);
}

static void SpeedC4_StopCurrentPulse(uint32_t nowMs)
{
    uint32_t elapsed;
    int16_t countLeft;
    int16_t countRight;
    int16_t avgSpeedLeft;
    int16_t avgSpeedRight;

    if (!g_speedC4Run) {
        return;
    }

    elapsed = nowMs - g_speedC4StartMs;
    Encoder_GetCounts(&countLeft, &countRight);
    avgSpeedLeft = (elapsed == 0U) ? 0 :
        (int16_t) (((int32_t) countLeft * 1000) / (int32_t) elapsed);
    avgSpeedRight = (elapsed == 0U) ? 0 :
        (int16_t) (((int32_t) countRight * 1000) / (int32_t) elapsed);

    SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
        SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
        avgSpeedLeft, avgSpeedRight, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), 0, 0U, 0U);
    g_speedC4Run = false;
    Motor_Stop();

    if ((g_speedC4PulseIndex + 1U) >= g_speedC4PulseTotal) {
        SpeedC4_ResetPulseSequence();
    } else {
        ++g_speedC4PulseIndex;
        g_speedC4PulseGap = true;
        g_speedC4PulseGapStartMs = nowMs;
    }
}

static void SpeedC4_AbortPulseSequence(uint32_t nowMs)
{
    if (g_speedC4Run) {
        SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
            SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
            0, 0, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), 0, 0U, 0U);
    }

    g_speedC4Run = false;
    Motor_Stop();
    SpeedC4_ResetPulseSequence();
}

static void SpeedC4_ServicePulseSequence(uint32_t nowMs)
{
    if (g_speedC4Run && ((nowMs - g_speedC4StartMs) >= SPEED_C4_PULSE_MS)) {
        SpeedC4_StopCurrentPulse(nowMs);
    }

    if (!g_speedC4Run &&
        g_speedC4PulseGap &&
        ((nowMs - g_speedC4PulseGapStartMs) >= SPEED_C4_PULSE_GAP_MS)) {
        g_speedC4PulseGap = false;
        SpeedC4_StartCurrentPulse(nowMs);
    }
}
#endif

static void SpeedC4_Update(uint32_t nowMs)
{
    bool key4Pressed = Key4_IsPressed();
    bool key3Pressed = Key3_IsPressed();
    bool key2Pressed = Key2_IsPressed();
    bool key1Pressed = Key1_IsPressed();

    g_devStage = DEV_STAGE_C0_MOTOR_DIR;
    g_navState = NAV_MOTOR_TEST;
    g_faultCode = FAULT_NONE;

#if SPEED_C4_OPEN_LOOP_DIAG
    if (key4Pressed && !g_key4LastPressed) {
        if (SpeedC4_IsPulseBusy()) {
            SpeedC4_AbortPulseSequence(nowMs);
        } else {
            SpeedC4_BeginPulseSequence(nowMs, SPEED_C4_SEQUENCE_BASE_PAIR);
        }
    }
    g_key4LastPressed = key4Pressed;

    if (key3Pressed && !g_key3LastPressed) {
        if (SpeedC4_IsPulseBusy()) {
            SpeedC4_AbortPulseSequence(nowMs);
        } else {
            SpeedC4_BeginPulseSequence(nowMs, SPEED_C4_SEQUENCE_RIGHT_MORE_PAIR);
        }
    }
    g_key3LastPressed = key3Pressed;

    if (key2Pressed && !g_key2LastPressed) {
        if (SpeedC4_IsPulseBusy()) {
            SpeedC4_AbortPulseSequence(nowMs);
        } else {
            SpeedC4_BeginPulseSequence(nowMs, SPEED_C4_SEQUENCE_RIGHT_LESS_PAIR);
        }
    }
    g_key2LastPressed = key2Pressed;

    if (key1Pressed && !g_key1LastPressed && !SpeedC4_IsPulseBusy()) {
        SpeedC4_LogExport(nowMs);
    }
    g_key1LastPressed = key1Pressed;
#else
    if (key4Pressed && !g_key4LastPressed) {
        if (g_speedC4Run) {
            g_speedC4Run = false;
            SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
                SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
                0, 0, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), g_speedC4LastSyncCorrection, 0U, 0U);
            Motor_Stop();
            g_speedC4Phase = SPEED_C4_PHASE_IDLE;
        } else {
            SpeedC4_SelectTarget(nowMs, SPEED_C4_TARGET_LOW_CPS, SPEED_C4_CONTROL_SPEED_P_DIST);
            SpeedC4_Start(nowMs, SPEED_C4_MODE_BOTH);
        }
    }
    g_key4LastPressed = key4Pressed;

    if (key3Pressed && !g_key3LastPressed) {
        if (g_speedC4Run) {
            g_speedC4Run = false;
            SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
                SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
                0, 0, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), g_speedC4LastSyncCorrection, 0U, 0U);
            Motor_Stop();
            g_speedC4Phase = SPEED_C4_PHASE_IDLE;
        } else {
            SpeedC4_SelectTarget(nowMs, SPEED_C4_TARGET_LOW_CPS, SPEED_C4_CONTROL_DISTURB_LEFT);
            SpeedC4_Start(nowMs, SPEED_C4_MODE_BOTH);
        }
    }
    g_key3LastPressed = key3Pressed;

    if (key2Pressed && !g_key2LastPressed) {
        if (g_speedC4Run) {
            g_speedC4Run = false;
            SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
                SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
                0, 0, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), g_speedC4LastSyncCorrection, 0U, 0U);
            Motor_Stop();
            g_speedC4Phase = SPEED_C4_PHASE_IDLE;
        } else {
            SpeedC4_SelectTarget(nowMs, SPEED_C4_TARGET_LOW_CPS, SPEED_C4_CONTROL_DISTURB_RIGHT);
            SpeedC4_Start(nowMs, SPEED_C4_MODE_BOTH);
        }
    }
    g_key2LastPressed = key2Pressed;

    if (key1Pressed && !g_key1LastPressed && !g_speedC4Run) {
        SpeedC4_LogExport(nowMs);
    }
    g_key1LastPressed = key1Pressed;
#endif

#if !SPEED_C4_OPEN_LOOP_DIAG
    SpeedC4_ServiceDisturbPhase(nowMs);
#endif

    if (g_speedC4Run && ((nowMs - g_speedC4LastSampleMs) >= SPEED_C4_SAMPLE_MS)) {
        int16_t countLeft;
        int16_t countRight;
        int16_t deltaLeft;
        int16_t deltaRight;
        int16_t speedLeft;
        int16_t speedRight;
        int16_t targetLeft = SpeedC4_TargetForLeft();
        int16_t targetRight = SpeedC4_TargetForRight();
#if !SPEED_C4_OPEN_LOOP_DIAG
        int16_t speedCorrectionLeft;
        int16_t speedCorrectionRight;
        int32_t desiredLeft;
        int32_t desiredRight;
#endif
        int16_t syncCorrection = 0;
        uint8_t satLeft = 0U;
        uint8_t satRight = 0U;
        uint32_t dt = nowMs - g_speedC4LastSampleMs;

        Encoder_GetCounts(&countLeft, &countRight);
        deltaLeft = (int16_t) (countLeft - g_speedC4PrevLeft);
        deltaRight = (int16_t) (countRight - g_speedC4PrevRight);
        speedLeft = (dt == 0U) ? 0 : (int16_t) (((int32_t) deltaLeft * 1000) / (int32_t) dt);
        speedRight = (dt == 0U) ? 0 : (int16_t) (((int32_t) deltaRight * 1000) / (int32_t) dt);

        g_speedC4PrevLeft = countLeft;
        g_speedC4PrevRight = countRight;
        g_speedC4LastSampleMs = nowMs;

#if SPEED_C4_OPEN_LOOP_DIAG
        g_speedC4IntegralLeft = 0;
        g_speedC4IntegralRight = 0;
        syncCorrection = 0;
        g_speedC4PwmLeft = targetLeft ?
            (int16_t) Clamp32((int32_t) g_speedC4OpenPwmLeft,
                SPEED_C4_PWM_MIN, SPEED_C4_PWM_MAX) : 0;
        g_speedC4PwmRight = targetRight ?
            (int16_t) Clamp32((int32_t) g_speedC4OpenPwmRight,
                SPEED_C4_PWM_MIN, SPEED_C4_PWM_MAX) : 0;
#else
        if (g_speedC4Control == SPEED_C4_CONTROL_OPEN) {
            g_speedC4IntegralLeft = 0;
            g_speedC4IntegralRight = 0;
            speedCorrectionLeft = 0;
            speedCorrectionRight = 0;
        } else {
            speedCorrectionLeft = SpeedC4_ComputeSpeedCorrection(
                targetLeft, speedLeft, dt, &g_speedC4IntegralLeft);
            speedCorrectionRight = SpeedC4_ComputeSpeedCorrection(
                targetRight, speedRight, dt, &g_speedC4IntegralRight);
        }

        if ((g_speedC4Control == SPEED_C4_CONTROL_SPEED_P_DIST) &&
            (targetLeft != 0) && (targetRight != 0)) {
            int32_t scaledRightSpeed = ((int32_t) speedRight * SPEED_C4_ENCODER_R_SCALE_NUM) /
                SPEED_C4_ENCODER_R_SCALE_DEN;
            int32_t scaledRightCount = ((int32_t) countRight * SPEED_C4_ENCODER_R_SCALE_NUM) /
                SPEED_C4_ENCODER_R_SCALE_DEN;
            int32_t speedSync = 0;
            int32_t distanceSync = ((int32_t) countLeft - scaledRightCount) / SPEED_C4_DISTANCE_SYNC_DIV;
#if SPEED_C4_SPEED_SYNC_DIV > 0
            speedSync = ((int32_t) speedLeft - scaledRightSpeed) / SPEED_C4_SPEED_SYNC_DIV;
#else
            (void) scaledRightSpeed;
#endif
            speedSync = Clamp32(speedSync, -SPEED_C4_SYNC_LIMIT, SPEED_C4_SYNC_LIMIT);
            distanceSync = Clamp32(distanceSync, -SPEED_C4_TOTAL_SYNC_LIMIT, SPEED_C4_TOTAL_SYNC_LIMIT);
            syncCorrection = (int16_t) Clamp32(speedSync + distanceSync,
                -(SPEED_C4_SYNC_LIMIT + SPEED_C4_TOTAL_SYNC_LIMIT),
                SPEED_C4_SYNC_LIMIT + SPEED_C4_TOTAL_SYNC_LIMIT);
        }

        desiredLeft = (targetLeft == 0) ? 0 :
            (int32_t) SpeedC4_BasePwmForLeft() + speedCorrectionLeft - syncCorrection;
        desiredRight = (targetRight == 0) ? 0 :
            (int32_t) SpeedC4_BasePwmForRight() + speedCorrectionRight + syncCorrection;
        g_speedC4PwmLeft = SpeedC4_LimitPwmStep(
            g_speedC4PwmLeft,
            (targetLeft == 0) ? 0 : SpeedC4_ClampPwmWithSat(desiredLeft, &satLeft));
        g_speedC4PwmRight = SpeedC4_LimitPwmStep(
            g_speedC4PwmRight,
            (targetRight == 0) ? 0 : SpeedC4_ClampPwmWithSat(desiredRight, &satRight));
#endif
        Motor_SetOutput(g_speedC4PwmLeft, g_speedC4PwmRight);
        g_speedC4LastSyncCorrection = syncCorrection;
        SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_RUN,
            targetLeft, targetRight, speedLeft, speedRight,
            g_speedC4PwmLeft, g_speedC4PwmRight, syncCorrection, satLeft, satRight);
    }

#if SPEED_C4_OPEN_LOOP_DIAG
    SpeedC4_ServicePulseSequence(nowMs);
#else
    if ((SpeedC4_RunDurationMs() > 0U) &&
        g_speedC4Run && ((nowMs - g_speedC4StartMs) >= SpeedC4_RunDurationMs())) {
        uint32_t elapsed = nowMs - g_speedC4StartMs;
        int16_t finalCountLeft;
        int16_t finalCountRight;
        int16_t avgSpeedLeft;
        int16_t avgSpeedRight;

        Encoder_GetCounts(&finalCountLeft, &finalCountRight);
        avgSpeedLeft = (elapsed == 0U) ? 0 :
            (int16_t) (((int32_t) finalCountLeft * 1000) / (int32_t) elapsed);
        avgSpeedRight = (elapsed == 0U) ? 0 :
            (int16_t) (((int32_t) finalCountRight * 1000) / (int32_t) elapsed);

        SpeedC4_LogAppend(nowMs, SPEED_C4_EVENT_STOP,
            SpeedC4_TargetForLeft(), SpeedC4_TargetForRight(),
            avgSpeedLeft, avgSpeedRight, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(),
            g_speedC4LastSyncCorrection, 0U, 0U);
        g_speedC4Run = false;
        Motor_Stop();
        g_speedC4Phase = SPEED_C4_PHASE_IDLE;
    }
#endif

    if (!g_speedC4Run) {
        Motor_Stop();
        g_speedC4Phase = SPEED_C4_PHASE_IDLE;
    }

    if ((nowMs - g_speedC4LastOledMs) >=
        (g_speedC4Run ? SPEED_C4_OLED_RUN_MS : SPEED_C4_OLED_STOP_MS)) {
        g_speedC4LastOledMs = nowMs;
        Oled_ShowMotorSpeed(MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), g_speedC4Run);
    }
    AppendLog(nowMs, 0, 0);
}

static void Motor_SetOutput(int16_t left, int16_t right)
{
    MotorDriver_SetOutput(left, right);
}

static void Motor_SetOutputDry(int16_t left, int16_t right)
{
    MotorDriver_SetOutputDry(left, right);
}

static void AppendLog(uint32_t nowMs, int16_t errDistCm, int16_t errYawDeg)
{
    NavLogEntry *entry = &g_navLogs[g_navLogWriteIndex];

    entry->tick_ms = nowMs;
    entry->pose_x_cm = (int16_t) g_poseMap.x_cm;
    entry->pose_y_cm = (int16_t) g_poseMap.y_cm;
    entry->target_x_cm = (int16_t) g_targetPoint.target_map_frame.x_cm;
    entry->target_y_cm = (int16_t) g_targetPoint.target_map_frame.y_cm;
    entry->err_dist_cm = errDistCm;
    entry->err_yaw_deg = errYawDeg;
    entry->cmd_left = MotorDriver_GetLastLeft();
    entry->cmd_right = MotorDriver_GetLastRight();
    entry->state = (uint8_t) g_navState;
    entry->dev_stage = (uint8_t) g_devStage;
    entry->fault_code = (uint8_t) g_faultCode;
    entry->plan_fail = g_planFailCode;
    entry->path_source = (uint8_t) g_path.source;
    entry->path_len = g_path.len;
    entry->waypoint_index = g_path.current_index;
    entry->err_x_cm = (int16_t) Clamp32(g_mission.control_error_cm.x_cm, -32768, 32767);
    entry->err_y_cm = (int16_t) Clamp32(g_mission.control_error_cm.y_cm, -32768, 32767);
    entry->mission_leg = (uint8_t) g_mission.leg;
    entry->forbidden_count = g_mission.forbidden_count;

    g_navLogWriteIndex = (uint8_t) ((g_navLogWriteIndex + 1U) % NAV_LOG_CAPACITY);
}

static void ClearTarget(void)
{
    memset((void *) &g_targetPoint, 0, sizeof(g_targetPoint));
}

static void ApplyPoseFromRadar(const RadarFrame_t *frame)
{
    RadarData_t data;
    Point2D rawPoint;
    Point2D mapPoint;
    int16_t rawYawDeg;
    int16_t yawDeg;

    Radar_ExtractData(frame, &data);
    rawPoint.x_cm = data.x_cm;
    rawPoint.y_cm = data.y_cm;
    mapPoint = TransformPoint(rawPoint, RADAR_SWAP_AXES, RADAR_X_SIGN,
                              RADAR_Y_SIGN, RADAR_X_OFFSET_CM, RADAR_Y_OFFSET_CM);
    rawYawDeg = NormalizeDeg((int16_t) data.yaw_deg);
    yawDeg = rawYawDeg;

#if JY901_ENABLE && JY901_USE_YAW_FOR_R1 && !RADAR_R1_USE_BRIDGE_YAW_ONLY
    /*
     * For the PB7 +90deg test, radar UART provides X/Y.  Heading control uses
     * the local JY901 yaw zeroed at PB7 so the first action is a deterministic
     * left turn to +90 even if Pi/radar yaw is delayed or stale.
     */
    if (Jy901_IsFresh(g_tickMs)) {
        yawDeg = g_jy901YawDeg;
    }
#endif

#if JY901_ENABLE && !RADAR_R1_USE_BRIDGE_YAW_ONLY && RADAR_R1_PI_YAW_JUMP_DEG > 0
    if (g_radarR1PiYawHaveLast) {
        int16_t piYawDelta = NormalizeDeg((int16_t)
            (rawYawDeg - g_radarR1PiYawLastRawDeg));
        if (Abs16(piYawDelta) > RADAR_R1_PI_YAW_JUMP_DEG) {
            g_radarR1PiYawFallbackUntilMs =
                g_tickMs + RADAR_R1_PI_YAW_FALLBACK_MS;
        }
    } else {
        g_radarR1PiYawHaveLast = true;
    }
    g_radarR1PiYawLastRawDeg = rawYawDeg;
#endif

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
    if (!g_radarR1OriginLocked) {
        g_poseRaw.x_cm = rawPoint.x_cm;
        g_poseRaw.y_cm = rawPoint.y_cm;
        g_poseRaw.yaw_deg = rawYawDeg;
        g_poseRaw.valid = true;
        g_poseRaw.stamp_ms = g_tickMs;
        if (!RadarR1_OriginStableFeed(mapPoint, yawDeg, g_tickMs)) {
            g_poseMap.valid = false;
            return;
        }
    }
    mapPoint.x_cm -= g_radarR1OriginMap.x_cm;
    mapPoint.y_cm -= g_radarR1OriginMap.y_cm;
    yawDeg = NormalizeDeg((int16_t) (yawDeg - g_radarR1OriginYawDeg));

    /*
     * Prefer the yaw delivered by the Raspberry Pi pose bridge so this test
     * checks radar/Pi-side alignment.  If that yaw jumps implausibly, keep the
     * raw Pi yaw in g_poseRaw for diagnosis and temporarily fall back to the
     * local JY901 yaw for control.
     */
    if (!g_radarR1YawFilterLocked) {
        g_radarR1YawFilteredDeg = yawDeg;
        g_radarR1YawFilterLocked = true;
    } else {
        int16_t delta = NormalizeDeg((int16_t) (yawDeg - g_radarR1YawFilteredDeg));
        g_radarR1YawFilteredDeg =
            NormalizeDeg((int16_t) (g_radarR1YawFilteredDeg +
                                    (delta / RADAR_R1_YAW_FILTER_DIV)));
    }
    yawDeg = g_radarR1YawFilteredDeg;
#if JY901_ENABLE && !RADAR_R1_USE_BRIDGE_YAW_ONLY
    if (((int32_t) (g_tickMs - g_radarR1PiYawFallbackUntilMs) < 0) &&
        Jy901_IsFresh(g_tickMs)) {
        yawDeg = g_jy901YawDeg;
    }
#endif
#endif

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_POSE_JUMP_REJECT_CM > 0
    if (g_poseMap.valid) {
        int32_t jumpX = Abs32(mapPoint.x_cm - g_poseMap.x_cm);
        int32_t jumpY = Abs32(mapPoint.y_cm - g_poseMap.y_cm);
        if ((jumpX > RADAR_R1_POSE_JUMP_REJECT_CM) ||
            (jumpY > RADAR_R1_POSE_JUMP_REJECT_CM)) {
            /*
             * Field log after the JY901B change showed a single radar x jump
             * from about +49 cm to -979 cm while y/yaw remained plausible.
             * Reject such impossible one-frame coordinate jumps instead of
             * letting the waypoint controller re-plan/turn on a false pose.
             */
            g_radarFrameErrCount++;
            return;
        }
    }
#endif

    g_poseRaw.x_cm = rawPoint.x_cm;
    g_poseRaw.y_cm = rawPoint.y_cm;
    g_poseRaw.yaw_deg = rawYawDeg;
    g_poseRaw.valid = true;
    g_poseRaw.stamp_ms = g_tickMs;

    g_poseMap.x_cm = mapPoint.x_cm;
    g_poseMap.y_cm = mapPoint.y_cm;
    g_poseMap.yaw_deg = yawDeg;
    g_poseMap.valid = true;
    g_poseMap.stamp_ms = g_tickMs;
    Mission_UpdatePoseTargetError();
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    /*
     * UART ISR must not run the navigation state machine or drive motors.
     * It only latches a "radar frame says Y stop is due" request; the 20 ms
     * main navigation loop consumes it and performs Motor_Stop safely.
     */
    RadarR1_RequestUrgentYStopFromPose(g_tickMs);
#endif
}

static void ApplyTargetFromWireless(const WirelessFrame_t *frame)
{
    Point2D rawPoint;
    Point2D mapPoint;
    uint8_t qrId = WirelessW0_NormalizeQrId(frame->qr_id);

    rawPoint.x_cm = frame->x_cm;
    rawPoint.y_cm = frame->y_cm;
    mapPoint = WirelessTargetMapPoint(qrId, rawPoint);

    if (g_uiCarRunEnabled && g_manualRouteRunActive) {
        /*
         * Manual PB7/PB25/PB0 tests must not be retargeted by a late or
         * repeated airplane frame.  Keep RX/OK diagnostics, but do not touch
         * g_targetPoint or g_manualTarget while a manual route is active.
         */
        return;
    }

    g_targetPoint.valid = true;
    g_targetPoint.qr_type = qrId;
    g_targetPoint.target_raw_flight_frame = rawPoint;
    g_targetPoint.target_map_frame = mapPoint;
    g_targetPoint.stamp_ms = g_tickMs;
    g_mission.target_source = TARGET_SOURCE_WIRELESS;
    Mission_UpdatePoseTargetError();

#if MANUAL_TARGET_INPUT_ENABLE
    /*
     * Keep the received flight target as the manual fallback too.  The
     * navigator refreshes manual targets after TARGET_TIMEOUT_MS; copying the
     * wireless target here prevents a one-shot airplane frame from expiring
     * mid-run and reverting to the previous PB7/PB25 test coordinate.
     */
    g_manualTargetQr = qrId;
    g_manualTargetX = (int16_t) mapPoint.x_cm;
    g_manualTargetY = (int16_t) mapPoint.y_cm;
#endif

#if WIRELESS_TARGET_AUTO_START_ENABLE && RADAR_R1_NAV_TEST_ENABLE
    if (!g_uiCarRunEnabled) {
        RadarR1_OfflineLogReset();
        g_radarR1EverArmed = true;
        g_radarR1LastArmedMs = g_tickMs;
        g_radarR1SlewLeft = 0;
        g_radarR1SlewRight = 0;
        NavSpeedLoop_Reset(g_tickMs);
        g_radarR1RouteDone = false;
        g_radarR1PreReturnFineUsed = false;
        g_radarR1RouteBActive = (qrId == QR_ID_B);
        Jy901_ResetYawOriginToCurrent();
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
        /* Flight coordinates are arbitrary QR targets: use generic A* to the
         * received coordinate, fine-stop there, then return home for scoring. */
        g_p7MissionModeEnable = true;
        g_mission.leg = MISSION_LEG_TO_TARGET;
#endif
#if RADAR_R1_ZERO_AT_FIRST_POSE
        RadarR1_ResetAutoOrigin();
        g_navState = NAV_WAIT_POSE;
#else
        g_navState = PoseIsFresh(g_tickMs) ? NAV_PLAN_TO_TARGET : NAV_WAIT_POSE;
#endif
        g_uiCarRunEnabled = true;
        g_manualRouteRunActive = false;
        RadarR1_OfflineLogAppend(g_tickMs);
    } else {
        Navigation_ResetPlan();
        g_navState = PoseIsFresh(g_tickMs) ? NAV_PLAN_TO_TARGET : NAV_WAIT_POSE;
    }
#endif
}

void SysTick_Handler(void)
{
    g_tickMs++;
    Encoder_Poll1ms();
}

void radar_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(radar_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte = (uint8_t) DL_UART_Main_receiveData(radar_INST);
        RadarFrame_t frame;
        g_radarRxByteCount++;
        if (Radar_ParseByte(byte, &frame)) {
            g_radarFrameOkCount++;
            ApplyPoseFromRadar(&frame);
        }
        break;
    }
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
    case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        (void) DL_UART_Main_receiveData(radar_INST);
        g_radarFrameErrCount++;
        break;
    case DL_UART_MAIN_IIDX_PARITY_ERROR:
    case DL_UART_MAIN_IIDX_NOISE_ERROR:
    case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        g_radarFrameErrCount++;
        break;
    default:
        break;
    }
}

static void WirelessW0_ShiftRawByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0U; i < (WIRELESS_FRAME_LEN - 1U); ++i) {
        g_wirelessRxWindow[i] = g_wirelessRxWindow[(uint8_t) (i + 1U)];
    }
    g_wirelessRxWindow[WIRELESS_FRAME_LEN - 1U] = byte;
}

static void WirelessW0_StoreLastRawFrame(void)
{
    uint8_t i;

    for (i = 0U; i < WIRELESS_FRAME_LEN; ++i) {
        g_wirelessLastRawFrame[i] = g_wirelessRxWindow[i];
    }
    g_wirelessLastRawFrameValid = 1U;
}

static void WirelessW0_CopyLastRawFrame(uint8_t *rawFrame, uint8_t *valid)
{
    uint8_t i;

    for (i = 0U; i < WIRELESS_FRAME_LEN; ++i) {
        rawFrame[i] = g_wirelessLastRawFrame[i];
    }
    *valid = g_wirelessLastRawFrameValid;
}

static void WirelessW0_StoreLastFrame(const WirelessFrame_t *frame)
{
    g_wirelessLastFrame.header1 = frame->header1;
    g_wirelessLastFrame.header2 = frame->header2;
    g_wirelessLastFrame.len = frame->len;
    g_wirelessLastFrame.type = frame->type;
    g_wirelessLastFrame.seq = frame->seq;
    g_wirelessLastFrame.target_id = frame->target_id;
    g_wirelessLastFrame.qr_id = frame->qr_id;
    g_wirelessLastFrame.x_cm = frame->x_cm;
    g_wirelessLastFrame.y_cm = frame->y_cm;
    g_wirelessLastFrame.flags = frame->flags;
    g_wirelessLastFrame.checksum = frame->checksum;
    g_wirelessLastFrame.tail = frame->tail;
}

static uint8_t WirelessW0_NormalizeQrId(uint8_t qrId)
{
    /*
     * Flight side currently reports the C QR as numeric ID 1.  Internally the
     * car keeps legacy QR bytes CA/CB/CC for display, fallback coordinates,
     * and manual route parity.  Normalize only the known flight code here; if
     * future A/B numeric IDs are defined, add them explicitly after field
     * confirmation.
     */
    if (qrId == 1U) {
        return QR_ID_C;
    }
    return qrId;
}

static void WirelessW0_SetEvent(uint8_t code, WirelessParseStatus_t status)
{
    g_wirelessLastEventCode = code;
    g_wirelessLastParseStatus = (uint8_t) status;
    g_wirelessLastEventMs = g_tickMs;
    g_wirelessEventPending = true;
}

static void WirelessW0_HandleFrame(const WirelessFrame_t *frame)
{
    WirelessW0_StoreLastFrame(frame);
    g_wirelessFrameOkCount++;

    if (!WirelessDiag_TargetIsForCar(frame->target_id)) {
        g_wirelessIgnoredCount++;
        WirelessW0_SetEvent('I', WIRELESS_PARSE_FRAME_OK);
        return;
    }
    if ((frame->flags & WIRELESS_FLAG_VALID) == 0U) {
        g_wirelessIgnoredCount++;
        WirelessW0_SetEvent('V', WIRELESS_PARSE_FRAME_OK);
        return;
    }
    if ((frame->seq != 0U) &&
        (g_wirelessHaveSeq != 0U) &&
        (frame->seq == g_wirelessLastSeq)) {
        g_wirelessDuplicateCount++;
        WirelessW0_SetEvent('D', WIRELESS_PARSE_FRAME_OK);
        return;
    }

    if (frame->seq != 0U) {
        g_wirelessHaveSeq = 1U;
        g_wirelessLastSeq = frame->seq;
    }
    WirelessW0_StoreLastRawFrame();
    g_wirelessTargetUpdateCount++;
    ApplyTargetFromWireless(frame);
    WirelessW0_SetEvent('F', WIRELESS_PARSE_FRAME_OK);
}

static void WirelessW0_HandleParseStatus(WirelessParseStatus_t status,
                                         const WirelessFrame_t *frame)
{
    switch (status) {
    case WIRELESS_PARSE_FRAME_OK:
        WirelessW0_HandleFrame(frame);
        break;
    case WIRELESS_PARSE_LEN_ERR:
        g_wirelessFrameErrCount++;
        g_wirelessLenErrCount++;
        WirelessW0_SetEvent('L', status);
        break;
    case WIRELESS_PARSE_TYPE_ERR:
        g_wirelessFrameErrCount++;
        g_wirelessTypeErrCount++;
        WirelessW0_SetEvent('T', status);
        break;
    case WIRELESS_PARSE_CHECKSUM_ERR:
        g_wirelessFrameErrCount++;
        g_wirelessChecksumErrCount++;
        WirelessW0_SetEvent('C', status);
        break;
    case WIRELESS_PARSE_TAIL_ERR:
        g_wirelessFrameErrCount++;
        g_wirelessTailErrCount++;
        WirelessW0_SetEvent('E', status);
        break;
    case WIRELESS_PARSE_NONE:
    default:
        break;
    }
}

void gyro_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(gyro_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte = (uint8_t) DL_UART_Main_receiveData(gyro_INST);
#if RADAR_R1_NAV_TEST_ENABLE
        if (g_radarR1LogFrozen) {
            break;
        }
#endif
        g_jy901RxByteCount++;
        Jy901_ParseByte(byte);
        break;
    }
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
    case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        (void) DL_UART_Main_receiveData(gyro_INST);
        g_jy901FrameErrCount++;
        break;
    case DL_UART_MAIN_IIDX_PARITY_ERROR:
    case DL_UART_MAIN_IIDX_NOISE_ERROR:
    case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        g_jy901FrameErrCount++;
        break;
    default:
        break;
    }
}

void openmv_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(openmv_INST)) {
    case DL_UART_MAIN_IIDX_RX: {
        uint8_t byte = (uint8_t) DL_UART_Main_receiveData(openmv_INST);
        WirelessFrame_t frame;
        WirelessParseStatus_t status;

        g_wirelessRxByteCount++;
        WirelessW0_ShiftRawByte(byte);
        status = Wireless_ParseByte(byte, &frame);
        WirelessW0_HandleParseStatus(status, &frame);
        break;
    }
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
    case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        (void) DL_UART_Main_receiveData(openmv_INST);
        g_wirelessFrameErrCount++;
        WirelessW0_SetEvent('U', WIRELESS_PARSE_NONE);
        break;
    case DL_UART_MAIN_IIDX_PARITY_ERROR:
    case DL_UART_MAIN_IIDX_NOISE_ERROR:
    case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        g_wirelessFrameErrCount++;
        WirelessW0_SetEvent('U', WIRELESS_PARSE_NONE);
        break;
    default:
        break;
    }
}

static void UartRxInit(UART_Regs *inst, IRQn_Type irqn)
{
    while (!DL_UART_Main_isRXFIFOEmpty(inst)) {
        (void) DL_UART_Main_receiveData(inst);
    }

    DL_UART_Main_clearInterruptStatus(inst,
                                      DL_UART_MAIN_INTERRUPT_RX |
                                      DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                      DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                      DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                      DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                      DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_enableInterrupt(inst,
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);
}

static void WirelessUart_Force115200(void)
{
    DL_UART_Main_disable(openmv_INST);
    DL_UART_Main_setOversampling(openmv_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(openmv_INST,
        WIRELESS_UART_IBRD_32MHZ_115200,
        WIRELESS_UART_FBRD_32MHZ_115200);
    DL_UART_Main_enable(openmv_INST);
}

static void RadarUart_Force115200(void)
{
    DL_UART_Main_disable(radar_INST);
    DL_UART_Main_setOversampling(radar_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(radar_INST,
        WIRELESS_UART_IBRD_32MHZ_115200,
        WIRELESS_UART_FBRD_32MHZ_115200);
    DL_UART_Main_enable(radar_INST);
}

static void RadarR0_PrintHeader(void)
{
    DebugUart_WriteString("radar_r0.header,time_ms,rx,ok,err,valid,age_ms,raw_x,raw_y,raw_yaw,map_x,map_y,map_yaw\r\n");
    DebugUart_WriteString("radar_r0.boot UART2/radar PB16_RX PB15_TX 115200_8N1 frame=14B_FA_AB motor_off debug=PA10_TX_PA11_RX_115200_8N1\r\n");
}

static void RadarR0_PrintLog(uint32_t nowMs)
{
    uint32_t stampMs = g_poseMap.stamp_ms;
    bool valid = g_poseMap.valid;
    uint32_t ageMs = valid ? (uint32_t) (nowMs - stampMs) : 0U;
    int32_t rawX = g_poseRaw.x_cm;
    int32_t rawY = g_poseRaw.y_cm;
    int32_t rawYaw = g_poseRaw.yaw_deg;
    int32_t mapX = g_poseMap.x_cm;
    int32_t mapY = g_poseMap.y_cm;
    int32_t mapYaw = g_poseMap.yaw_deg;

    DebugUart_WriteString("radar_r0.log,");
    DebugUart_WriteInt((int32_t) nowMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarRxByteCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameOkCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(valid ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) ageMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(rawX);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(rawY);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(rawYaw);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(mapX);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(mapY);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(mapYaw);
    DebugUart_WriteString("\r\n");
}

static void RadarR0_ShowOled(uint32_t nowMs)
{
    uint32_t ageMs;

    if ((nowMs - g_radarR0LastOledMs) < RADAR_R0_OLED_MS) {
        return;
    }
    g_radarR0LastOledMs = nowMs;
    ageMs = g_poseMap.valid ? (uint32_t) (nowMs - g_poseMap.stamp_ms) : 0U;

    Oled_Clear();
    Oled_ShowString(1U, 1U, "R0 RADAR RX    ");
    Oled_ShowString(2U, 1U, "OK:");
    Oled_ShowUnsigned4(2U, 4U, WirelessDiag_ClampU16(g_radarFrameOkCount));
    Oled_ShowString(2U, 10U, "ER:");
    Oled_ShowUnsigned4(2U, 13U, WirelessDiag_ClampU16(g_radarFrameErrCount));
    Oled_ShowString(3U, 1U, "X:");
    Oled_ShowSigned4(3U, 3U, (int16_t) g_poseMap.x_cm);
    Oled_ShowString(3U, 9U, "Y:");
    Oled_ShowSigned4(3U, 11U, (int16_t) g_poseMap.y_cm);
    Oled_ShowString(4U, 1U, "Yaw:");
    Oled_ShowSigned4(4U, 5U, g_poseMap.yaw_deg);
    Oled_ShowString(4U, 11U, "A:");
    Oled_ShowUnsigned3(4U, 13U, WirelessDiag_ClampU16(ageMs));
}

static void RadarR0_Update(uint32_t nowMs)
{
    Motor_Stop();

    if ((nowMs - g_radarR0LastLogMs) >= RADAR_R0_LOG_MS) {
        g_radarR0LastLogMs = nowMs;
        RadarR0_PrintLog(nowMs);
    }
    RadarR0_ShowOled(nowMs);
}

static void RadarR1_PrintHeader(void)
{
    DebugUart_WriteString("nav_r1.header,time_ms,armed,motor_out,pose_valid,age_ms,target_valid,state,fault,plan_ok,plan_fail,path_len,wp,raw_x,raw_y,raw_yaw,pose_x,pose_y,yaw,desired_yaw,target_x,target_y,err_x,err_y,sl_tl,sl_tr,sl_l,sl_r,sl_sync,leg,target_src,forbid_n,wp_x,wp_y,dist,yaw_err,line_err,line_acc,line_bias,ctrl_stage,cmd_l,cmd_r,rx,ok,err,gyro_rx,gyro_ok,gyro_err,gyro_age,turn_wait_ms,turn_stable_ms,turn_dx,turn_dy,turn_active,turn_done,post_turn,start_gate,radar_dy,enc_fwd,jy_yaw,stop_src,protect,org_state,org_frames\r\n");
    DebugUart_WriteString("nav_r1.boot tag=");
    DebugUart_WriteString(RADAR_R1_TUNE_TAG);
    DebugUart_WriteString(" bridge_xyz_half_diag UART2/radar=115200_8N1 yaw=Pi_bridge JY901=log_only debug=PA10_115200_8N1 max_pwm=");
    DebugUart_WriteInt((int32_t) RADAR_R1_MAX_PWM);
    DebugUart_WriteString(" target=");
    DebugUart_WriteInt((int32_t) g_manualTargetX);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_manualTargetY);
    DebugUart_WriteString(" yaw_filter_div=");
    DebugUart_WriteInt((int32_t) RADAR_R1_YAW_FILTER_DIV);
    DebugUart_WriteString(" slew=");
    DebugUart_WriteInt((int32_t) RADAR_R1_CMD_SLEW_PER_UPDATE);
    DebugUart_WriteString(" min_pwm=");
    DebugUart_WriteInt((int32_t) RADAR_R1_MIN_PWM);
    DebugUart_WriteString(" turn_pwm=");
    DebugUart_WriteInt((int32_t) RADAR_R1_TURN_IN_PLACE_PWM);
    DebugUart_WriteString(" head_pwm=");
    DebugUart_WriteInt((int32_t) RADAR_R1_HEADING_CORRECT_PWM);
    DebugUart_WriteString(" straight_lr=");
    DebugUart_WriteInt((int32_t) RADAR_R1_STRAIGHT_BASE_L);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_STRAIGHT_BASE_R);
    DebugUart_WriteString(" settle_deg=");
    DebugUart_WriteInt((int32_t) RADAR_R1_TURN_SETTLE_DEG);
    DebugUart_WriteString(" settle_ms=");
    DebugUart_WriteInt((int32_t) RADAR_R1_FIRST_TURN_SETTLE_MS);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_RIGHT_TURN_SETTLE_MS);
    DebugUart_WriteString(" line_div/max=");
    DebugUart_WriteInt((int32_t) RADAR_R1_CROSSTRACK_GAIN_DIV);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_CROSSTRACK_MAX_DEG);
    DebugUart_WriteString(" y_seg=");
    DebugUart_WriteInt((int32_t) RADAR_R1_FIXED_Y_SEGMENT_CM);
    DebugUart_WriteString(" y_reach=");
    DebugUart_WriteInt((int32_t) RADAR_R1_FIXED_Y_REACH_CM);
    DebugUart_WriteString(" seg_settle=");
    DebugUart_WriteInt((int32_t) RADAR_R1_SEGMENT_SETTLE_MS);
    DebugUart_WriteString(" post=");
    DebugUart_WriteInt((int32_t) RADAR_R1_POST_TURN_RAMP_MS);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_POST_TURN_LEFT_BOOST_PWM);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_POST_TURN_START_MIN_L);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) RADAR_R1_POST_TURN_START_MIN_R);
    DebugUart_WriteString(" fixed_route=");
    DebugUart_WriteInt((int32_t) RADAR_R1_FIXED_ROUTE_TEST);
    DebugUart_WriteString(" p7_mission=");
    DebugUart_WriteInt(g_p7MissionModeEnable ? 1 : 0);
    DebugUart_WriteString(" speed_loop=");
    DebugUart_WriteInt((int32_t) NAV_SPEED_LOOP_ENABLE);
    DebugUart_WriteString(" map=");
    DebugUart_WriteInt((int32_t) g_mission.map.x_min_cm);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) g_mission.map.x_max_cm);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) g_mission.map.y_min_cm);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) g_mission.map.y_max_cm);
    DebugUart_WriteString(" home=");
    DebugUart_WriteInt((int32_t) g_mission.map.home_map_frame.x_cm);
    DebugUart_WriteChar('/');
    DebugUart_WriteInt((int32_t) g_mission.map.home_map_frame.y_cm);
    DebugUart_WriteString(" forbid=");
    DebugUart_WriteInt((int32_t) g_mission.forbidden_count);
    DebugUart_WriteString(" motor_out=");
    DebugUart_WriteInt((g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) ? 1 : 0);
    DebugUart_WriteString(" offline_ram=128x200ms stream_raw=0 PB7_short=run_stop_freeze_log_and_reset_yaw0 PB7_long=export PB6=menu_ok PB25=up_plus PB0=down_minus PA11_cmd=E_or_D_export oled=mission/map/target_cmd\r\n");
}

static void RadarR1_PrintRawLog(uint32_t nowMs)
{
    DebugUart_WriteString("radar_raw,");
    DebugUart_WriteInt((int32_t) nowMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseRaw.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseRaw.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_poseRaw.yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameOkCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameErrCount);
    DebugUart_WriteString("\r\n");
}

static void RadarR1_PrintLog(uint32_t nowMs)
{
    bool poseValid = PoseIsFresh(nowMs);
    bool targetValid = TargetIsFresh(nowMs);
    bool planOk = g_lastPlan.ok;
    uint32_t poseAgeMs = g_poseMap.valid ? (nowMs - g_poseMap.stamp_ms) : 9999U;
    uint32_t gyroAgeMs = g_jy901Valid ? (nowMs - g_jy901StampMs) : 9999U;
    int32_t wpX = 0;
    int32_t wpY = 0;
    int16_t errDist = 0;
    int16_t errYaw = 0;
    int16_t desiredYaw = 0;
    int16_t lineErr = 0;
    const SpeedLoopStatus *speedLoop = NavSpeedLoop_GetStatus();

    if ((g_path.len > 0U) && (g_path.current_index < g_path.len)) {
        PathPoint *wp = &g_path.points[g_path.current_index];
        int32_t dx;
        int32_t dy;
        wpX = wp->x_cm;
        wpY = wp->y_cm;
        if (poseValid) {
            dx = wpX - g_poseMap.x_cm;
            dy = wpY - g_poseMap.y_cm;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
                if (g_path.current_index >= 1U) {
                    lineErr = RadarR1_FixedRouteLineError(g_path.current_index, dx, dy);
                    desiredYaw = RadarR1_FixedRouteDesiredYaw(g_path.current_index, lineErr,
                                                                g_poseMap.yaw_deg, nowMs);
                    errDist = RadarR1_FixedRouteAxisDistance(g_path.current_index, dx, dy);
                } else {
                    desiredYaw = ApproxAtan2Deg(dy, dx);
                    errDist = ApproxDistanceCm(dx, dy);
                }
            } else
#endif
            {
            desiredYaw = ApproxAtan2Deg(dy, dx);
            errDist = ApproxDistanceCm(dx, dy);
            }
            errYaw = NormalizeDeg((int16_t) (desiredYaw - g_poseMap.yaw_deg));
        }
    }

    DebugUart_WriteString("nav_r1.log,");
    DebugUart_WriteInt((int32_t) nowMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_uiCarRunEnabled ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(poseValid ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) WirelessDiag_ClampU16(poseAgeMs));
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(targetValid ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_navState);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_faultCode);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(planOk ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_planFailCode);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_path.len);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_path.current_index);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseRaw.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseRaw.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_poseRaw.yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseMap.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_poseMap.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_poseMap.yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) desiredYaw);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_targetPoint.target_map_frame.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_targetPoint.target_map_frame.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_mission.control_error_cm.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(g_mission.control_error_cm.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) speedLoop->target_l_cps);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) speedLoop->target_r_cps);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) speedLoop->speed_l_cps);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) speedLoop->speed_r_cps);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) speedLoop->sync);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_mission.leg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_mission.target_source);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_mission.forbidden_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(wpX);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(wpY);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) errDist);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) errYaw);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) lineErr);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) RadarR1_CtrlStage(errDist, errYaw));
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) MotorDriver_GetLastLeft());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) MotorDriver_GetLastRight());
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarRxByteCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameOkCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_radarFrameErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_jy901RxByteCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_jy901FrameOkCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_jy901FrameErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) WirelessDiag_ClampU16(gyroAgeMs));
    DebugUart_WriteString("\r\n");
}

static void RadarR1_FillLogEntry(uint32_t nowMs, RadarR1LogEntry *entry)
{
    bool poseValid = PoseIsFresh(nowMs);
    bool targetValid = TargetIsFresh(nowMs);
    uint32_t poseAgeMs = g_poseMap.valid ? (nowMs - g_poseMap.stamp_ms) : 9999U;
    uint32_t gyroAgeMs = g_jy901Valid ? (nowMs - g_jy901StampMs) : 9999U;
    int32_t wpX = 0;
    int32_t wpY = 0;
    int16_t errDist = 0;
    int16_t errYaw = 0;
    int16_t desiredYaw = 0;
    int16_t lineErr = 0;
    uint8_t flags = 0U;
    uint32_t turnSettleRemainingMs = 0U;
    uint32_t turnStableMs = 0U;
    int32_t turnPoseDx = 0;
    int32_t turnPoseDy = 0;
    uint8_t startGate = 0U;
    int16_t radarDyCm = 0;
    const SpeedLoopStatus *speedLoop = NavSpeedLoop_GetStatus();

    if ((g_path.len > 0U) && (g_path.current_index < g_path.len)) {
        PathPoint *wp = &g_path.points[g_path.current_index];
        wpX = wp->x_cm;
        wpY = wp->y_cm;
        if (poseValid) {
            int32_t dx = wpX - g_poseMap.x_cm;
            int32_t dy = wpY - g_poseMap.y_cm;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
                if (g_path.current_index >= 1U) {
                    lineErr = RadarR1_FixedRouteLineError(g_path.current_index, dx, dy);
                    desiredYaw = RadarR1_FixedRouteDesiredYaw(g_path.current_index, lineErr,
                                                                g_poseMap.yaw_deg, nowMs);
                    errDist = RadarR1_FixedRouteAxisDistance(g_path.current_index, dx, dy);
                } else {
                    desiredYaw = ApproxAtan2Deg(dy, dx);
                    errDist = ApproxDistanceCm(dx, dy);
                }
            } else
#endif
            {
            desiredYaw = ApproxAtan2Deg(dy, dx);
            errDist = ApproxDistanceCm(dx, dy);
            }
            errYaw = NormalizeDeg((int16_t) (desiredYaw - g_poseMap.yaw_deg));
        }
    }

    if (g_uiCarRunEnabled) {
        flags |= 0x01U;
    }
    if (g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) {
        flags |= 0x02U;
    }
    if (poseValid) {
        flags |= 0x04U;
    }
    if (targetValid) {
        flags |= 0x08U;
    }
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    if (g_radarR1TurnSettleActive) {
        if ((int32_t) (g_radarR1TurnSettleUntilMs - nowMs) > 0) {
            turnSettleRemainingMs = g_radarR1TurnSettleUntilMs - nowMs;
        }
        turnStableMs = nowMs - g_radarR1TurnSettleStableSinceMs;
        turnPoseDx = (int32_t) g_poseMap.x_cm - g_radarR1TurnSettleLastX;
        turnPoseDy = (int32_t) g_poseMap.y_cm - g_radarR1TurnSettleLastY;
    }
    if (g_radarR1TurnSettleDone) {
        startGate |= 0x01U;
    }
    if (Abs16(errYaw) <= RADAR_R1_TURN_SETTLE_DEG) {
        startGate |= 0x02U;
    }
    if (poseValid && (poseAgeMs <= RADAR_R1_TURN_FRESH_MAX_MS)) {
        startGate |= 0x04U;
    }
    if (!g_radarR1TurnSettleActive ||
        (turnStableMs >= RADAR_R1_TURN_POSE_STABLE_MS)) {
        startGate |= 0x08U;
    }
    if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
        (g_path.len > 0U)) {
        radarDyCm = (int16_t) Clamp32(
            (int32_t) g_poseMap.y_cm - g_path.points[0].y_cm,
            -32768, 32767);
    }
    if ((g_radarR1UrgentYStopPending != false) ||
        ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
         (g_path.len > 1U) &&
         (radarDyCm >= RadarR1_FixedYStopGateDeltaCm()))) {
        startGate |= 0x10U;
    }
    if (g_radarR1ProtectFault != 0U) {
        startGate |= 0x20U;
    }
    if (g_radarR1OriginLocked) {
        startGate |= 0x40U;
    }
    if (g_radarR1OriginState == 1U) {
        startGate |= 0x80U;
    }
#endif

    entry->time_ms = nowMs;
    entry->raw_x_cm = g_poseRaw.x_cm;
    entry->raw_y_cm = g_poseRaw.y_cm;
    entry->pose_x_cm = g_poseMap.x_cm;
    entry->pose_y_cm = g_poseMap.y_cm;
    entry->target_x_cm = g_targetPoint.target_map_frame.x_cm;
    entry->target_y_cm = g_targetPoint.target_map_frame.y_cm;
    entry->raw_yaw_deg = g_poseRaw.yaw_deg;
    entry->yaw_deg = g_poseMap.yaw_deg;
    entry->desired_yaw_deg = desiredYaw;
    entry->wp_x_cm = (int16_t) wpX;
    entry->wp_y_cm = (int16_t) wpY;
    entry->err_dist_cm = errDist;
    entry->err_yaw_deg = errYaw;
    entry->line_err_cm = lineErr;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    entry->line_err_acc = g_radarR1LineErrAccumScore;
    entry->line_bias_pwm = g_radarR1LineErrAccumBiasPwm;
#else
    entry->line_err_acc = 0;
    entry->line_bias_pwm = 0;
#endif
    entry->cmd_left = MotorDriver_GetLastLeft();
    entry->cmd_right = MotorDriver_GetLastRight();
    entry->rx_count = WirelessDiag_ClampU16(g_radarRxByteCount);
    entry->ok_count = WirelessDiag_ClampU16(g_radarFrameOkCount);
    entry->err_count = WirelessDiag_ClampU16(g_radarFrameErrCount);
    entry->pose_age_ms = WirelessDiag_ClampU16(poseAgeMs);
    entry->gyro_rx_count = WirelessDiag_ClampU16(g_jy901RxByteCount);
    entry->gyro_ok_count = WirelessDiag_ClampU16(g_jy901FrameOkCount);
    entry->gyro_err_count = WirelessDiag_ClampU16(g_jy901FrameErrCount);
    entry->gyro_age_ms = WirelessDiag_ClampU16(gyroAgeMs);
    entry->turn_settle_remaining_ms = turnSettleRemainingMs;
    entry->turn_stable_ms = turnStableMs;
    entry->turn_pose_dx_cm = (int16_t) Clamp32(turnPoseDx, -32768, 32767);
    entry->turn_pose_dy_cm = (int16_t) Clamp32(turnPoseDy, -32768, 32767);
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    entry->turn_settle_active = g_radarR1TurnSettleActive ? 1U : 0U;
    entry->turn_settle_done = g_radarR1TurnSettleDone ? 1U : 0U;
    entry->post_turn_active = g_radarR1PostTurnRampActive ? 1U : 0U;
#else
    entry->turn_settle_active = 0U;
    entry->turn_settle_done = 0U;
    entry->post_turn_active = 0U;
#endif
    entry->start_gate = startGate;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    entry->radar_dy_cm = radarDyCm;
    entry->enc_forward_counts = g_radarR1GuardForwardCounts;
    entry->jy_yaw_deg = g_jy901YawDeg;
    entry->stop_source = g_radarR1StopSource;
    entry->protect_fault = g_radarR1ProtectFault;
    entry->origin_state = g_radarR1OriginState;
    entry->origin_frames = g_radarR1OriginStableFrames;
#else
    entry->radar_dy_cm = 0;
    entry->enc_forward_counts = 0;
    entry->jy_yaw_deg = 0;
    entry->stop_source = 0U;
    entry->protect_fault = 0U;
    entry->origin_state = 0U;
    entry->origin_frames = 0U;
#endif
    entry->err_x_cm = (int16_t) Clamp32(g_mission.control_error_cm.x_cm, -32768, 32767);
    entry->err_y_cm = (int16_t) Clamp32(g_mission.control_error_cm.y_cm, -32768, 32767);
    entry->speed_target_l = speedLoop->target_l_cps;
    entry->speed_target_r = speedLoop->target_r_cps;
    entry->speed_l = speedLoop->speed_l_cps;
    entry->speed_r = speedLoop->speed_r_cps;
    entry->speed_sync = speedLoop->sync;
    entry->state = (uint8_t) g_navState;
    entry->fault = (uint8_t) g_faultCode;
    entry->plan_ok = g_lastPlan.ok ? 1U : 0U;
    entry->plan_fail = g_planFailCode;
    entry->path_len = g_path.len;
    entry->waypoint_index = g_path.current_index;
    entry->ctrl_stage = RadarR1_CtrlStage(errDist, errYaw);
    entry->flags = flags;
    entry->mission_leg = (uint8_t) g_mission.leg;
    entry->target_source = (uint8_t) g_mission.target_source;
    entry->forbidden_count = g_mission.forbidden_count;
}

static void RadarR1_OfflineLogAppend(uint32_t nowMs)
{
    RadarR1LogEntry *entry = &g_radarR1Logs[g_radarR1LogIndex];

    if (g_radarR1LogFrozen) {
        return;
    }

    /*
     * Preserve the last live radar window.  If the Pi->MCU TTL is unplugged
     * after a field test, the loop keeps running and pose_valid becomes 0.
     * Do not let those stale timeout samples overwrite the useful motion/noise
     * samples in RAM.  If no pose has ever been logged, keep one stale row so
     * the failure is still visible.
     */
    if (!PoseIsFresh(nowMs) && (g_radarR1LogCount > 0U)) {
        return;
    }

    RadarR1_FillLogEntry(nowMs, entry);
    g_radarR1LogIndex = (uint8_t) ((g_radarR1LogIndex + 1U) %
                                   RADAR_R1_LOG_CAPACITY);
    if (g_radarR1LogCount < RADAR_R1_LOG_CAPACITY) {
        ++g_radarR1LogCount;
    } else {
        ++g_radarR1LogDropped;
    }
}

static void RadarR1_OfflineLogReset(void)
{
    memset(g_radarR1Logs, 0, sizeof(g_radarR1Logs));
    g_radarR1LogIndex = 0U;
    g_radarR1LogCount = 0U;
    g_radarR1LogDropped = 0U;
    g_radarR1LogFrozen = false;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    g_radarR1LineErrFilterLocked = false;
    g_radarR1LineErrFilterWp = 0U;
    g_radarR1LineErrFilteredCm = 0;
    RadarR1_ResetLineErrAccum();
#endif
}

static void RadarR1_OfflineLogFreeze(uint32_t nowMs)
{
    /*
     * Snapshot the stop instant once, then refuse all later radar/control
     * samples.  This keeps the just-finished test run in RAM until PB6/PA11
     * export, even if the Pi keeps streaming pose frames while the car is
     * parked beside the computer.
     */
    if (!g_radarR1LogFrozen) {
        RadarR1_OfflineLogAppend(nowMs);
        g_radarR1LogFrozen = true;
    }
}

static void RadarR1_PrintSavedEntry(const RadarR1LogEntry *entry)
{
    DebugUart_WriteString("nav_r1.log,");
    DebugUart_WriteInt((int32_t) entry->time_ms);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((entry->flags & 0x01U) ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((entry->flags & 0x02U) ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((entry->flags & 0x04U) ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->pose_age_ms);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((entry->flags & 0x08U) ? 1 : 0);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->state);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->fault);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->plan_ok);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->plan_fail);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->path_len);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->waypoint_index);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->raw_x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->raw_y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->raw_yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->pose_x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->pose_y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->desired_yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->target_x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt(entry->target_y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->err_x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->err_y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->speed_target_l);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->speed_target_r);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->speed_l);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->speed_r);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->speed_sync);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->mission_leg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->target_source);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->forbidden_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->wp_x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->wp_y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->err_dist_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->err_yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->line_err_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->line_err_acc);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->line_bias_pwm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->ctrl_stage);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->cmd_left);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->cmd_right);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->rx_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->ok_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->err_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->gyro_rx_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->gyro_ok_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->gyro_err_count);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->gyro_age_ms);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_settle_remaining_ms);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_stable_ms);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_pose_dx_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_pose_dy_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_settle_active);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->turn_settle_done);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->post_turn_active);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->start_gate);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->radar_dy_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->enc_forward_counts);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->jy_yaw_deg);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->stop_source);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->protect_fault);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->origin_state);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) entry->origin_frames);
    DebugUart_WriteString("\r\n");
}

static void RadarR1_OfflineLogExport(uint32_t nowMs)
{
    uint8_t start;
    uint8_t i;

    RadarR1_OfflineLogAppend(nowMs);
    DebugUart_WriteString("nav_r1.export begin tag=");
    DebugUart_WriteString(RADAR_R1_TUNE_TAG);
    DebugUart_WriteString(" count=");
    DebugUart_WriteInt((int32_t) g_radarR1LogCount);
    DebugUart_WriteString(" dropped=");
    DebugUart_WriteInt((int32_t) g_radarR1LogDropped);
    DebugUart_WriteString(" sample_ms=");
    DebugUart_WriteInt((int32_t) RADAR_R1_OFFLINE_LOG_MS);
    DebugUart_WriteString(" capacity=");
    DebugUart_WriteInt((int32_t) RADAR_R1_LOG_CAPACITY);
    DebugUart_WriteString("\r\n");
    DebugUart_WriteString("nav_r1.header,time_ms,armed,motor_out,pose_valid,age_ms,target_valid,state,fault,plan_ok,plan_fail,path_len,wp,raw_x,raw_y,raw_yaw,pose_x,pose_y,yaw,desired_yaw,target_x,target_y,err_x,err_y,sl_tl,sl_tr,sl_l,sl_r,sl_sync,leg,target_src,forbid_n,wp_x,wp_y,dist,yaw_err,line_err,line_acc,line_bias,ctrl_stage,cmd_l,cmd_r,rx,ok,err,gyro_rx,gyro_ok,gyro_err,gyro_age,turn_wait_ms,turn_stable_ms,turn_dx,turn_dy,turn_active,turn_done,post_turn,start_gate,radar_dy,enc_fwd,jy_yaw,stop_src,protect,org_state,org_frames\r\n");

    start = (g_radarR1LogCount == RADAR_R1_LOG_CAPACITY) ?
        g_radarR1LogIndex : 0U;
    for (i = 0U; i < g_radarR1LogCount; ++i) {
        uint8_t index = (uint8_t) ((start + i) % RADAR_R1_LOG_CAPACITY);
        RadarR1_PrintSavedEntry(&g_radarR1Logs[index]);
    }
    DebugUart_WriteString("nav_r1.export end\r\n");
}

static void RadarR1_ShowOled(uint32_t nowMs, bool forceAll)
{
    static bool s_oledInitialized = false;
    static uint8_t s_oledLineIndex = 0U;
    uint8_t lineNo;
    uint8_t linesToDraw = forceAll ? 4U : 1U;
    char line[17];

    if (!forceAll && s_oledInitialized &&
        ((nowMs - g_radarR1LastOledMs) < RADAR_R1_OLED_SLICE_MS)) {
        return;
    }
    g_radarR1LastOledMs = nowMs;

    if (!s_oledInitialized || forceAll) {
        Oled_Clear();
        s_oledInitialized = true;
        s_oledLineIndex = 0U;
    }

    for (uint8_t i = 0U; i < linesToDraw; i++) {
        lineNo = (uint8_t) (s_oledLineIndex + 1U);
        OledText_Clear16(line);

#if RADAR_R1_PHASE8_OLED_DIAG
        if (lineNo == 1U) {
            OledText_CopyAt(line, 1U, "FL RX");
            OledText_UnsignedAt(line, 6U,
                                WirelessDiag_ClampU16(g_wirelessRxByteCount), 4U);
            OledText_CopyAt(line, 11U, "OK");
            OledText_UnsignedAt(line, 13U,
                                WirelessDiag_ClampU16(g_wirelessFrameOkCount), 4U);
        } else if (lineNo == 2U) {
            OledText_CopyAt(line, 1U, "Q:");
            OledText_CharAt(line, 3U, WirelessDiag_QrChar(g_targetPoint.qr_type));
            OledText_CopyAt(line, 5U, "S:");
            OledText_UnsignedAt(line, 7U, (uint32_t) g_mission.target_source, 1U);
            OledText_CopyAt(line, 9U, g_uiCarRunEnabled ? "RUN" : "STP");
            OledText_CopyAt(line, 13U, "ER");
            OledText_UnsignedAt(line, 15U,
                                WirelessDiag_ClampU16(g_wirelessFrameErrCount), 2U);
        } else if (lineNo == 3U) {
            OledText_CopyAt(line, 1U, "FX:");
            OledText_Signed4At(line, 4U,
                               g_targetPoint.target_raw_flight_frame.x_cm);
            OledText_CopyAt(line, 10U, "Y:");
            OledText_Signed4At(line, 12U,
                               g_targetPoint.target_raw_flight_frame.y_cm);
        } else {
            OledText_CopyAt(line, 1U, "MX:");
            OledText_Signed4At(line, 4U,
                               g_targetPoint.target_map_frame.x_cm);
            OledText_CopyAt(line, 10U, "Y:");
            OledText_Signed4At(line, 12U,
                               g_targetPoint.target_map_frame.y_cm);
        }
#else
        if (lineNo == 1U) {
            OledText_CopyAt(line, 1U, "R1 MAP          ");
            OledText_CopyAt(line, 8U, g_uiCarRunEnabled ? "RUN" : "STOP");
        } else if (lineNo == 2U) {
            OledText_CopyAt(line, 1U, "X:");
            OledText_Signed4At(line, 3U, (int16_t) g_poseMap.x_cm);
            OledText_CopyAt(line, 9U, "Y:");
            OledText_Signed4At(line, 11U, (int16_t) g_poseMap.y_cm);
        } else if (lineNo == 3U) {
            OledText_CopyAt(line, 1U, "Yaw:");
            OledText_Signed4At(line, 5U, g_poseMap.yaw_deg);
            OledText_CopyAt(line, 11U, "WP:");
            OledText_UnsignedAt(line, 14U, g_path.current_index, 2U);
        } else {
            OledText_CopyAt(line, 1U, "WX:");
            if (g_path.len > 0U) {
                OledText_Signed4At(line, 4U,
                                   g_path.points[g_path.current_index].x_cm);
                OledText_CopyAt(line, 10U, "Y:");
                OledText_Signed4At(line, 12U,
                                   g_path.points[g_path.current_index].y_cm);
            } else {
                OledText_Signed4At(line, 4U, 0);
                OledText_CopyAt(line, 10U, "Y:");
                OledText_Signed4At(line, 12U, 0);
            }
        }
#endif

        Oled_ShowString(lineNo, 1U, line);
        s_oledLineIndex = (uint8_t) ((s_oledLineIndex + 1U) & 0x03U);
    }
}

static void RadarR1_Update(uint32_t nowMs)
{
    bool key4Pressed = Key4_IsPressed();
    bool key3Pressed = Key3_IsPressed();
    bool key2Pressed = Key2_IsPressed();
    bool key1Pressed = Key1_IsPressed();
    bool key1ShortHome = (!key1Pressed && g_key1LastPressed &&
                          !g_uiKey1LongHandled &&
                          (g_uiMode == UI_MODE_HOME));

    Ui_Update(nowMs, key4Pressed, key3Pressed, key2Pressed, key1Pressed);
    g_key4LastPressed = key4Pressed;
    g_key3LastPressed = key3Pressed;
    g_key2LastPressed = key2Pressed;
    g_key1LastPressed = key1Pressed;

    if (!TargetIsFresh(nowMs) && !g_radarR1RouteDone) {
        ManualTarget_Apply(nowMs, false);
    }

    Navigation_Update(nowMs);
    if (!g_uiCarRunEnabled) {
        Motor_SetOutputDry(MotorDriver_GetLastLeft(), MotorDriver_GetLastRight());
    } else {
        g_radarR1EverArmed = true;
        g_radarR1LastArmedMs = nowMs;
    }

#if RADAR_R1_NAV_TEST_ENABLE
    if (PoseIsFresh(nowMs)) {
        DL_GPIO_setPins(LED1_PORT, LED1_led1_PIN);
    } else {
        DL_GPIO_clearPins(LED1_PORT, LED1_led1_PIN);
    }
    if (g_uiCarRunEnabled && g_lastPlan.ok && (g_path.len > 0U)) {
        DL_GPIO_setPins(LED2_PORT, LED2_led2_PIN);
    } else {
        DL_GPIO_clearPins(LED2_PORT, LED2_led2_PIN);
    }
#endif

    DebugCommand_Update(nowMs);

    if ((nowMs - g_radarR1LastOfflineLogMs) >= RADAR_R1_OFFLINE_LOG_MS) {
        g_radarR1LastOfflineLogMs = nowMs;
        if (g_uiCarRunEnabled ||
            !g_radarR1EverArmed ||
            ((nowMs - g_radarR1LastArmedMs) <= RADAR_R1_LOG_UNARMED_TAIL_MS)) {
            RadarR1_OfflineLogAppend(nowMs);
        }
    }

    if (key1ShortHome) {
        RadarR1_OfflineLogExport(nowMs);
    }

#if RADAR_R1_STREAM_LOG_ENABLE
    if ((nowMs - g_radarR1LastLogMs) >= RADAR_R1_LOG_MS) {
        g_radarR1LastLogMs = nowMs;
        RadarR1_PrintRawLog(nowMs);
    }
#endif
#if RADAR_R1_OLED_WHILE_RUNNING_ENABLE
    RadarR1_ShowOled(nowMs, false);
#else
    if (!g_uiCarRunEnabled) {
        RadarR1_ShowOled(nowMs, false);
    }
#endif
}

static void WirelessW0_CopyLastFrame(WirelessFrame_t *frame)
{
    frame->header1 = g_wirelessLastFrame.header1;
    frame->header2 = g_wirelessLastFrame.header2;
    frame->len = g_wirelessLastFrame.len;
    frame->type = g_wirelessLastFrame.type;
    frame->seq = g_wirelessLastFrame.seq;
    frame->target_id = g_wirelessLastFrame.target_id;
    frame->qr_id = g_wirelessLastFrame.qr_id;
    frame->x_cm = g_wirelessLastFrame.x_cm;
    frame->y_cm = g_wirelessLastFrame.y_cm;
    frame->flags = g_wirelessLastFrame.flags;
    frame->checksum = g_wirelessLastFrame.checksum;
    frame->tail = g_wirelessLastFrame.tail;
}

static void WirelessW0_PrintLog(uint32_t nowMs, uint8_t eventCode)
{
    WirelessFrame_t frame;
    Point2D rawPoint;
    Point2D mapPoint;
    uint8_t rawFrame[WIRELESS_FRAME_LEN];
    uint8_t rawValid;
    uint8_t i;

    __disable_irq();
    WirelessW0_CopyLastFrame(&frame);
    WirelessW0_CopyLastRawFrame(rawFrame, &rawValid);
    __enable_irq();

    rawPoint.x_cm = frame.x_cm;
    rawPoint.y_cm = frame.y_cm;
    mapPoint = WirelessTargetMapPoint(WirelessW0_NormalizeQrId(frame.qr_id),
                                      rawPoint);

    DebugUart_WriteString("wireless_w0.log,");
    DebugUart_WriteInt((int32_t) nowMs);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessRxByteCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessTxFrameCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessFrameOkCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessFrameErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessChecksumErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessLenErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessTypeErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessTailErrCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessDuplicateCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessIgnoredCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessTargetUpdateCount);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.seq);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.target_id);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.qr_id);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) mapPoint.x_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) mapPoint.y_cm);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) frame.flags);
    DebugUart_WriteChar(',');
    DebugUart_WriteInt((int32_t) g_wirelessLastParseStatus);
    DebugUart_WriteChar(',');
    DebugUart_WriteString(WirelessDiag_EventText(eventCode));
    for (i = 0U; i < WIRELESS_FRAME_LEN; ++i) {
        DebugUart_WriteChar(',');
        DebugUart_WriteInt(rawValid ? (int32_t) rawFrame[i] : 0);
    }
    DebugUart_WriteString("\r\n");
}


#if MANUAL_TARGET_INPUT_ENABLE
static uint8_t ManualTarget_NextQr(uint8_t qr, int8_t dir)
{
    if (dir >= 0) {
        if (qr == QR_ID_A) {
            return QR_ID_B;
        }
        if (qr == QR_ID_B) {
            return QR_ID_C;
        }
        return QR_ID_A;
    }

    if (qr == QR_ID_C) {
        return QR_ID_B;
    }
    if (qr == QR_ID_B) {
        return QR_ID_A;
    }
    return QR_ID_C;
}

static int16_t ManualTarget_NextStep(int16_t step, int8_t dir)
{
    static const int16_t steps[] = {1, 5, 10, 20, 50};
    uint8_t i;
    uint8_t index = 2U;

    for (i = 0U; i < (uint8_t) (sizeof(steps) / sizeof(steps[0])); ++i) {
        if (steps[i] == step) {
            index = i;
            break;
        }
    }

    if (dir >= 0) {
        index = (uint8_t) ((index + 1U) % (uint8_t) (sizeof(steps) / sizeof(steps[0])));
    } else if (index == 0U) {
        index = (uint8_t) ((sizeof(steps) / sizeof(steps[0])) - 1U);
    } else {
        index--;
    }
    return steps[index];
}

static char Coord_FieldChar(void)
{
    switch (g_coordField) {
    case COORD_FIELD_QR:
        return 'Q';
    case COORD_FIELD_X:
        return 'X';
    case COORD_FIELD_Y:
        return 'Y';
    case COORD_FIELD_STEP:
        return 'S';
    case COORD_FIELD_APPLY:
        return 'A';
    case COORD_FIELD_BACK:
        return 'B';
    default:
        return '?';
    }
}

static char Map_FieldChar(void)
{
    switch (g_mapField) {
    case MAP_FIELD_X_MIN: return 'A';
    case MAP_FIELD_X_MAX: return 'B';
    case MAP_FIELD_Y_MIN: return 'C';
    case MAP_FIELD_Y_MAX: return 'D';
    case MAP_FIELD_BOUNDARY: return 'M';
    case MAP_FIELD_OBSTACLE: return 'O';
    case MAP_FIELD_HOME_X: return 'X';
    case MAP_FIELD_HOME_Y: return 'Y';
    case MAP_FIELD_APPLY: return 'P';
    case MAP_FIELD_BACK: return 'K';
    default: return '?';
    }
}

static char Zone_FieldChar(void)
{
    switch (g_zoneField) {
    case ZONE_FIELD_INDEX: return 'I';
    case ZONE_FIELD_COUNT_SET: return 'N';
    case ZONE_FIELD_X_MIN: return 'A';
    case ZONE_FIELD_Y_MIN: return 'B';
    case ZONE_FIELD_X_MAX: return 'C';
    case ZONE_FIELD_Y_MAX: return 'D';
    case ZONE_FIELD_APPLY: return 'P';
    case ZONE_FIELD_BACK: return 'K';
    default: return '?';
    }
}

static const char *Ui_MenuTitle(void)
{
    switch (g_uiMenuItem) {
    case UI_MENU_COORD: return "COORD POS       ";
    case UI_MENU_MAP: return "MAP BOUNDARY    ";
    case UI_MENU_ZONE: return "FORBID ZONE     ";
    case UI_MENU_TASK: return "TASK CONTROL    ";
    case UI_MENU_DEBUG: return "DEBUG INFO      ";
    default: return "UNKNOWN         ";
    }
}

static void ManualTarget_Apply(uint32_t nowMs, bool sendFrame)
{
    Point2D point;
    WirelessFrame_t frame;

    point.x_cm = g_manualTargetX;
    point.y_cm = g_manualTargetY;

    if (sendFrame) {
        WirelessW0_SendSample(nowMs, g_manualTargetQr, g_manualTargetX, g_manualTargetY);
    }

    g_targetPoint.valid = true;
    g_targetPoint.qr_type = g_manualTargetQr;
    g_targetPoint.target_raw_flight_frame = point;
    g_targetPoint.target_map_frame = point;
    g_targetPoint.stamp_ms = nowMs;
    g_mission.target_source = TARGET_SOURCE_MANUAL;
    Mission_UpdatePoseTargetError();
    g_manualTargetApplyCount++;

    frame.header1 = WIRELESS_FRAME_HEADER1;
    frame.header2 = WIRELESS_FRAME_HEADER2;
    frame.len = WIRELESS_FRAME_LEN;
    frame.type = WIRELESS_TYPE_TARGET;
    frame.seq = g_wirelessTxSeq;
    frame.target_id = WIRELESS_W0_TARGET_ID;
    frame.qr_id = g_manualTargetQr;
    frame.x_cm = g_manualTargetX;
    frame.y_cm = g_manualTargetY;
    frame.flags = WIRELESS_FLAG_VALID;
    frame.checksum = 0U;
    frame.tail = WIRELESS_FRAME_TAIL;
    WirelessW0_StoreLastFrame(&frame);
    WirelessW0_SetEvent('M', WIRELESS_PARSE_FRAME_OK);
}

static void Coord_AdjustValue(int8_t dir)
{
    switch (g_coordField) {
    case COORD_FIELD_QR:
        g_manualTargetQr = ManualTarget_NextQr(g_manualTargetQr, dir);
        break;
    case COORD_FIELD_X:
        g_manualTargetX = (int16_t) Clamp32((int32_t) g_manualTargetX +
                                            (int32_t) dir * g_manualTargetStep,
                                            g_mission.map.x_min_cm,
                                            g_mission.map.x_max_cm);
        break;
    case COORD_FIELD_Y:
        g_manualTargetY = (int16_t) Clamp32((int32_t) g_manualTargetY +
                                            (int32_t) dir * g_manualTargetStep,
                                            g_mission.map.y_min_cm,
                                            g_mission.map.y_max_cm);
        break;
    case COORD_FIELD_STEP:
        g_manualTargetStep = ManualTarget_NextStep(g_manualTargetStep, dir);
        break;
    default:
        break;
    }
}

static void Coord_MoveField(int8_t dir)
{
    uint8_t field = (uint8_t) g_coordField;

    if (dir >= 0) {
        field = (uint8_t) ((field + 1U) % (uint8_t) COORD_FIELD_COUNT);
    } else if (field == 0U) {
        field = (uint8_t) COORD_FIELD_COUNT - 1U;
    } else {
        field--;
    }
    g_coordField = (CoordField) field;
}

static void Map_MoveField(int8_t dir)
{
    uint8_t field = (uint8_t) g_mapField;

    if (dir >= 0) {
        field = (uint8_t) ((field + 1U) % (uint8_t) MAP_FIELD_COUNT);
    } else if (field == 0U) {
        field = (uint8_t) MAP_FIELD_COUNT - 1U;
    } else {
        field--;
    }
    g_mapField = (MapField) field;
}

static void Zone_MoveField(int8_t dir)
{
    uint8_t field = (uint8_t) g_zoneField;

    if (dir >= 0) {
        field = (uint8_t) ((field + 1U) % (uint8_t) ZONE_FIELD_COUNT);
    } else if (field == 0U) {
        field = (uint8_t) ZONE_FIELD_COUNT - 1U;
    } else {
        field--;
    }
    g_zoneField = (ZoneField) field;
}

static void Task_MoveField(int8_t dir)
{
    uint8_t field = (uint8_t) g_taskField;

    if (dir >= 0) {
        field = (uint8_t) ((field + 1U) % (uint8_t) TASK_FIELD_COUNT);
    } else if (field == 0U) {
        field = (uint8_t) TASK_FIELD_COUNT - 1U;
    } else {
        field--;
    }
    g_taskField = (TaskField) field;
}

static void Map_AdjustValue(int8_t dir)
{
    int32_t delta = (int32_t) dir * g_menuStep;

    switch (g_mapField) {
    case MAP_FIELD_X_MIN:
        g_mission.map.x_min_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.x_min_cm + delta, -999, g_mission.map.x_max_cm - MAP_CELL_CM);
        break;
    case MAP_FIELD_X_MAX:
        g_mission.map.x_max_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.x_max_cm + delta, g_mission.map.x_min_cm + MAP_CELL_CM, 999);
        break;
    case MAP_FIELD_Y_MIN:
        g_mission.map.y_min_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.y_min_cm + delta, -999, g_mission.map.y_max_cm - MAP_CELL_CM);
        break;
    case MAP_FIELD_Y_MAX:
        g_mission.map.y_max_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.y_max_cm + delta, g_mission.map.y_min_cm + MAP_CELL_CM, 999);
        break;
    case MAP_FIELD_BOUNDARY:
        g_mission.map.boundary_margin_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.boundary_margin_cm + delta, 0, 80);
        break;
    case MAP_FIELD_OBSTACLE:
        g_mission.map.obstacle_margin_cm = (int16_t) Clamp32(
            (int32_t) g_mission.map.obstacle_margin_cm + delta, 0, 80);
        break;
    case MAP_FIELD_HOME_X:
        g_mission.map.home_map_frame.x_cm = Clamp32(
            g_mission.map.home_map_frame.x_cm + delta,
            g_mission.map.x_min_cm, g_mission.map.x_max_cm);
        break;
    case MAP_FIELD_HOME_Y:
        g_mission.map.home_map_frame.y_cm = Clamp32(
            g_mission.map.home_map_frame.y_cm + delta,
            g_mission.map.y_min_cm, g_mission.map.y_max_cm);
        break;
    default:
        break;
    }
    g_mission.map.valid = Mission_ConfigIsValid();
}

static void Zone_AdjustValue(int8_t dir)
{
    ForbiddenZone *zone;
    int32_t delta = (int32_t) dir * g_menuStep;

    if (g_zoneField == ZONE_FIELD_INDEX) {
        int32_t maxIndex = (g_mission.forbidden_count == 0U) ? 0 :
            (int32_t) g_mission.forbidden_count - 1;
        g_zoneEditIndex = (uint8_t) Clamp32((int32_t) g_zoneEditIndex + dir,
                                            0, maxIndex);
        return;
    }
    if (g_zoneField == ZONE_FIELD_COUNT_SET) {
        g_mission.forbidden_count = (uint8_t) Clamp32(
            (int32_t) g_mission.forbidden_count + dir, 0, MAP_FORBIDDEN_MAX);
        if ((g_zoneEditIndex >= g_mission.forbidden_count) &&
            (g_mission.forbidden_count > 0U)) {
            g_zoneEditIndex = (uint8_t) (g_mission.forbidden_count - 1U);
        }
        return;
    }
    if (g_mission.forbidden_count == 0U) {
        return;
    }
    zone = &g_mission.forbidden[g_zoneEditIndex];
    zone->valid = true;
    zone->source = 1U; /* 1 = menu/manual */
    switch (g_zoneField) {
    case ZONE_FIELD_X_MIN:
        zone->rect.x_min_cm = (int16_t) Clamp32(
            (int32_t) zone->rect.x_min_cm + delta,
            g_mission.map.x_min_cm, zone->rect.x_max_cm - MAP_CELL_CM);
        break;
    case ZONE_FIELD_Y_MIN:
        zone->rect.y_min_cm = (int16_t) Clamp32(
            (int32_t) zone->rect.y_min_cm + delta,
            g_mission.map.y_min_cm, zone->rect.y_max_cm - MAP_CELL_CM);
        break;
    case ZONE_FIELD_X_MAX:
        zone->rect.x_max_cm = (int16_t) Clamp32(
            (int32_t) zone->rect.x_max_cm + delta,
            zone->rect.x_min_cm + MAP_CELL_CM, g_mission.map.x_max_cm);
        break;
    case ZONE_FIELD_Y_MAX:
        zone->rect.y_max_cm = (int16_t) Clamp32(
            (int32_t) zone->rect.y_max_cm + delta,
            zone->rect.y_min_cm + MAP_CELL_CM, g_mission.map.y_max_cm);
        break;
    default:
        break;
    }
}

static void Ui_MenuMove(int8_t dir)
{
    uint8_t item = (uint8_t) g_uiMenuItem;

    if (dir >= 0) {
        item = (uint8_t) ((item + 1U) % (uint8_t) UI_MENU_COUNT);
    } else if (item == 0U) {
        item = (uint8_t) UI_MENU_COUNT - 1U;
    } else {
        item--;
    }
    g_uiMenuItem = (UiMenuItem) item;
}

static void Ui_HandleMenuShort(uint32_t nowMs)
{
    if (g_uiMode == UI_MODE_HOME) {
        g_uiMode = UI_MODE_MENU;
        return;
    }

    if (g_uiMode == UI_MODE_MENU) {
        if (g_uiMenuItem == UI_MENU_COORD) {
            g_uiMode = UI_MODE_COORD;
            g_coordEditing = false;
        } else if (g_uiMenuItem == UI_MENU_MAP) {
            g_uiMode = UI_MODE_MAP;
            g_mapEditing = false;
        } else if (g_uiMenuItem == UI_MENU_ZONE) {
            g_uiMode = UI_MODE_ZONE;
            g_zoneEditing = false;
        } else if (g_uiMenuItem == UI_MENU_TASK) {
            g_uiMode = UI_MODE_TASK;
        } else if (g_uiMenuItem == UI_MENU_DEBUG) {
            g_uiMode = UI_MODE_DEBUG;
        }
        return;
    }

    if (g_uiMode == UI_MODE_COORD) {
        if (g_coordEditing) {
            g_coordEditing = false;
            return;
        }

        if (g_coordField == COORD_FIELD_APPLY) {
            ManualTarget_Apply(nowMs, true);
            return;
        }
        if (g_coordField == COORD_FIELD_BACK) {
            g_uiMode = UI_MODE_MENU;
            g_coordEditing = false;
            return;
        }
        g_coordEditing = true;
        return;
    }

    if (g_uiMode == UI_MODE_MAP) {
        if (g_mapEditing) {
            g_mapEditing = false;
            return;
        }
        if (g_mapField == MAP_FIELD_APPLY) {
            g_mission.map.valid = Mission_ConfigIsValid();
            GridMap_Rebuild();
            return;
        }
        if (g_mapField == MAP_FIELD_BACK) {
            g_uiMode = UI_MODE_MENU;
            g_mapEditing = false;
            return;
        }
        g_mapEditing = true;
        return;
    }

    if (g_uiMode == UI_MODE_ZONE) {
        if (g_zoneEditing) {
            g_zoneEditing = false;
            return;
        }
        if (g_zoneField == ZONE_FIELD_APPLY) {
            GridMap_Rebuild();
            return;
        }
        if (g_zoneField == ZONE_FIELD_BACK) {
            g_uiMode = UI_MODE_MENU;
            g_zoneEditing = false;
            return;
        }
        g_zoneEditing = true;
        return;
    }

    if (g_uiMode == UI_MODE_TASK) {
        if (g_taskField == TASK_FIELD_TO_TARGET) {
            g_p7MissionModeEnable = true;
            g_mission.leg = MISSION_LEG_TO_TARGET;
            Navigation_ResetPlan();
            g_navState = PoseIsFresh(nowMs) ? NAV_PLAN_TO_TARGET : NAV_WAIT_POSE;
            g_uiCarRunEnabled = true;
            return;
        }
        if (g_taskField == TASK_FIELD_HOME) {
            g_p7MissionModeEnable = true;
            g_mission.leg = MISSION_LEG_TO_HOME;
            Navigation_ResetPlan();
            g_navState = PoseIsFresh(nowMs) ? NAV_PLAN_HOME : NAV_WAIT_POSE;
            g_uiCarRunEnabled = true;
            return;
        }
        if (g_taskField == TASK_FIELD_STOP) {
            g_uiCarRunEnabled = false;
            Motor_Stop();
            RadarR1_OfflineLogFreeze(nowMs);
            return;
        }
        if (g_taskField == TASK_FIELD_CLEAR_LOG) {
            RadarR1_OfflineLogReset();
            return;
        }
        if (g_taskField == TASK_FIELD_BACK) {
            g_uiMode = UI_MODE_MENU;
            return;
        }
    }

    if (g_uiMode == UI_MODE_DEBUG) {
        g_uiMode = UI_MODE_MENU;
    }
}

static void Ui_HandleMenuLong(void)
{
    if (g_uiMode == UI_MODE_HOME) {
        g_uiMode = UI_MODE_MENU;
        g_coordEditing = false;
    } else {
        g_uiMode = UI_MODE_HOME;
        g_coordEditing = false;
        g_mapEditing = false;
        g_zoneEditing = false;
    }
}

static void Ui_ToggleRunRoute(uint32_t nowMs, uint8_t routeQr)
{
        if (g_uiCarRunEnabled) {
            g_uiCarRunEnabled = false;
            g_manualRouteRunActive = false;
            Motor_Stop();
            NavSpeedLoop_Reset(nowMs);
            RadarR1_OfflineLogFreeze(nowMs);
            /*
             * Phase-9 field diagnostics: PB7 pause should immediately dump the
             * frozen offline log.  This avoids depending on the debug RX
             * command path after a run, and preserves the oscillation window.
             */
            RadarR1_OfflineLogExport(nowMs);
        } else {
            RadarR1_OfflineLogReset();
            g_radarR1EverArmed = true;
            g_radarR1LastArmedMs = nowMs;
            g_radarR1SlewLeft = 0;
            g_radarR1SlewRight = 0;
            NavSpeedLoop_Reset(nowMs);
            g_radarR1RouteDone = false;
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE && RADAR_R1_A_FORBID_TEST_DISABLE_B
            if (routeQr == QR_ID_B) {
                routeQr = QR_ID_A;
            }
#endif
            g_radarR1RouteBActive = (routeQr == QR_ID_B);
            if (routeQr == QR_ID_B) {
                g_manualTargetQr = QR_ID_B;
                g_manualTargetX = (int16_t) (RADAR_R1_FIXED_FIRST_X_CM +
                    RADAR_R1_B_FIXED_FINAL_X_SIGN *
                    RADAR_R1_B_RUNTIME_FINAL_X_CM);
                g_manualTargetY = RADAR_R1_B_RUNTIME_FINAL_Y_CM;
            } else if (routeQr == QR_ID_C) {
                g_manualTargetQr = QR_ID_C;
                g_manualTargetX = RADAR_R1_C_ASTAR_TARGET_X_CM;
                g_manualTargetY = RADAR_R1_C_ASTAR_TARGET_Y_CM;
            } else {
                g_manualTargetQr = QR_ID_A;
                g_manualTargetX = (int16_t) (RADAR_R1_FIXED_FIRST_X_CM +
                    RADAR_R1_FIXED_FINAL_X_DELTA_CM);
                g_manualTargetY = RADAR_R1_FIXED_FINAL_Y_CM;
            }
            /*
             * PB7 route start must not reuse a still-fresh target from the
             * previous run.  We just selected the manual QR/coordinate above;
             * latch it into g_targetPoint immediately before Navigation_Update()
             * plans, otherwise TargetIsFresh() may keep the old A-code target
             * for one more run and make the car appear to randomly choose A/C.
             */
            ManualTarget_Apply(nowMs, false);
            Jy901_ResetYawOriginToCurrent();
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            /* Manual QR tests use the generic A* planner.  After QR fine-stop,
             * all routes enter the return-home leg so the competition flow
             * includes the previously-missing return to start. */
            g_p7MissionModeEnable = true;
            g_mission.leg = MISSION_LEG_TO_TARGET;
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
            RadarR1_ResetAutoOrigin();
#endif
            Navigation_ResetPlan();
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
            g_navState = NAV_WAIT_POSE;
#else
            if (TargetIsFresh(nowMs)) {
                g_navState = PoseIsFresh(nowMs) ? NAV_PLAN_TO_TARGET : NAV_WAIT_POSE;
            } else {
                g_navState = NAV_WAIT_TARGET;
            }
#endif
            g_uiCarRunEnabled = true;
            g_radarR1PreReturnFineUsed = false;
            g_manualRouteRunActive = true;
            RadarR1_OfflineLogAppend(nowMs);
        }
}

static void Ui_ToggleRun(uint32_t nowMs)
{
    Ui_ToggleRunRoute(nowMs, QR_ID_A);
}

static void Ui_Update(uint32_t nowMs, bool key4Pressed, bool key3Pressed,
                      bool key2Pressed, bool key1Pressed)
{
    if (key2Pressed && !g_key2LastPressed) {
        g_uiKey2PressStartMs = nowMs;
        g_uiKey2LongHandled = false;
    }
    if (key2Pressed && !g_uiKey2LongHandled &&
        ((uint32_t) (nowMs - g_uiKey2PressStartMs) >= UI_RUN_LONG_PRESS_MS)) {
        RadarR1_OfflineLogExport(nowMs);
        g_uiKey2LongHandled = true;
    }
    if (!key2Pressed && g_key2LastPressed && !g_uiKey2LongHandled) {
        Ui_ToggleRun(nowMs);
    }

    if (key1Pressed && !g_key1LastPressed) {
        g_uiKey1PressStartMs = nowMs;
        g_uiKey1LongHandled = false;
    }
    if (key1Pressed && !g_uiKey1LongHandled &&
        ((uint32_t) (nowMs - g_uiKey1PressStartMs) >= UI_MENU_LONG_PRESS_MS)) {
        Ui_HandleMenuLong();
        g_uiKey1LongHandled = true;
    }
    if (!key1Pressed && g_key1LastPressed && !g_uiKey1LongHandled) {
        if (g_radarR1LogFrozen) {
            RadarR1_OfflineLogExport(nowMs);
        } else {
            Ui_HandleMenuShort(nowMs);
        }
    }

    if (key3Pressed && !g_key3LastPressed) {
        if ((g_uiMode == UI_MODE_HOME) && !g_uiCarRunEnabled) {
            /* TEST_DISABLE_B only disables the historical B route on PB25.
             * PB25 is now the C-code manual test entry, so it must still
             * start QR_ID_C in HOME mode. */
            Ui_ToggleRunRoute(nowMs, QR_ID_C);
        } else if (g_uiMode == UI_MODE_MENU) {
            Ui_MenuMove(-1);
        } else if (g_uiMode == UI_MODE_COORD) {
            if (g_coordEditing) {
                Coord_AdjustValue(1);
            } else {
                Coord_MoveField(-1);
            }
        } else if (g_uiMode == UI_MODE_MAP) {
            if (g_mapEditing) {
                Map_AdjustValue(1);
            } else {
                Map_MoveField(-1);
            }
        } else if (g_uiMode == UI_MODE_ZONE) {
            if (g_zoneEditing) {
                Zone_AdjustValue(1);
            } else {
                Zone_MoveField(-1);
            }
        } else if (g_uiMode == UI_MODE_TASK) {
            Task_MoveField(-1);
        }
    }

    if (key4Pressed && !g_key4LastPressed) {
        if ((g_uiMode == UI_MODE_HOME) && !g_uiCarRunEnabled) {
            /* PB0 is temporarily reserved for B-code route testing.
             * Menu/down/minus functions are intentionally disabled during
             * airplane-coordinate integration preparation. */
            Ui_ToggleRunRoute(nowMs, QR_ID_B);
        }
    }
}

static void Ui_ShowHomeOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, g_uiCarRunEnabled ? "HOME RUN PB7PAU" : "HOME PAU PB7RUN");
    Oled_ShowString(2U, 1U, "T:");
    Oled_ShowChar(2U, 3U, WirelessDiag_QrChar(g_manualTargetQr));
    Oled_ShowString(2U, 5U, " X:");
    Oled_ShowSigned4(2U, 8U, g_manualTargetX);
    Oled_ShowString(3U, 1U, "Y:");
    Oled_ShowSigned4(3U, 3U, g_manualTargetY);
    Oled_ShowString(3U, 10U, "OK:");
    Oled_ShowUnsigned3(3U, 13U, WirelessDiag_ClampU16(g_manualTargetApplyCount));
    Oled_ShowString(4U, 1U, "PB6 Menu P7LExp");
}

static void Ui_ShowMenuOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, "MENU PB6 OK   ");
    Oled_ShowString(2U, 1U, "> ");
    Oled_ShowString(2U, 3U, Ui_MenuTitle());
    Oled_ShowString(3U, 1U, "PB25 UP PB0 DN");
    Oled_ShowString(4U, 1U, "Hold PB6 Back ");
}

static void Ui_ShowCoordOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, g_coordEditing ? "COORD EDIT F:" : "COORD SEL  F:");
    Oled_ShowChar(1U, 14U, Coord_FieldChar());
    Oled_ShowString(2U, 1U, "Q:");
    Oled_ShowChar(2U, 3U, WirelessDiag_QrChar(g_manualTargetQr));
    Oled_ShowString(2U, 5U, " X:");
    Oled_ShowSigned4(2U, 8U, g_manualTargetX);
    Oled_ShowString(3U, 1U, "Y:");
    Oled_ShowSigned4(3U, 3U, g_manualTargetY);
    Oled_ShowString(3U, 10U, "S:");
    Oled_ShowUnsigned3(3U, 12U, (uint16_t) g_manualTargetStep);
    if (g_coordField == COORD_FIELD_APPLY) {
        Oled_ShowString(4U, 1U, "PB6 APPLY SEND ");
    } else if (g_coordField == COORD_FIELD_BACK) {
        Oled_ShowString(4U, 1U, "PB6 BACK MENU  ");
    } else if (g_coordEditing) {
        Oled_ShowString(4U, 1U, "PB25+ PB0- OK ");
    } else {
        Oled_ShowString(4U, 1U, "PB25^ PB0v OK ");
    }
}

static void Ui_ShowMapOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, g_mapEditing ? "MAP EDIT F:" : "MAP SEL  F:");
    Oled_ShowChar(1U, 14U, Map_FieldChar());
    Oled_ShowString(2U, 1U, "X:");
    Oled_ShowSigned4(2U, 3U, g_mission.map.x_min_cm);
    Oled_ShowChar(2U, 8U, '/');
    Oled_ShowSigned4(2U, 9U, g_mission.map.x_max_cm);
    Oled_ShowString(3U, 1U, "Y:");
    Oled_ShowSigned4(3U, 3U, g_mission.map.y_min_cm);
    Oled_ShowChar(3U, 8U, '/');
    Oled_ShowSigned4(3U, 9U, g_mission.map.y_max_cm);
    Oled_ShowString(4U, 1U, "H:");
    Oled_ShowSigned4(4U, 3U, (int16_t) g_mission.map.home_map_frame.x_cm);
    Oled_ShowChar(4U, 8U, ',');
    Oled_ShowSigned4(4U, 9U, (int16_t) g_mission.map.home_map_frame.y_cm);
}

static void Ui_ShowZoneOled(void)
{
    ForbiddenZone *zone = (g_mission.forbidden_count > 0U) ?
        &g_mission.forbidden[g_zoneEditIndex] : 0;

    Oled_Clear();
    Oled_ShowString(1U, 1U, g_zoneEditing ? "ZONE EDIT F:" : "ZONE SEL  F:");
    Oled_ShowChar(1U, 14U, Zone_FieldChar());
    Oled_ShowString(2U, 1U, "I:");
    Oled_ShowUnsigned3(2U, 3U, g_zoneEditIndex);
    Oled_ShowString(2U, 8U, "N:");
    Oled_ShowUnsigned3(2U, 10U, g_mission.forbidden_count);
    if (zone != 0) {
        Oled_ShowString(3U, 1U, "A:");
        Oled_ShowSigned4(3U, 3U, zone->rect.x_min_cm);
        Oled_ShowChar(3U, 8U, ',');
        Oled_ShowSigned4(3U, 9U, zone->rect.y_min_cm);
        Oled_ShowString(4U, 1U, "B:");
        Oled_ShowSigned4(4U, 3U, zone->rect.x_max_cm);
        Oled_ShowChar(4U, 8U, ',');
        Oled_ShowSigned4(4U, 9U, zone->rect.y_max_cm);
    } else {
        Oled_ShowString(3U, 1U, "NO FORBID ZONE ");
        Oled_ShowString(4U, 1U, "SET N FIRST    ");
    }
}

static void Ui_ShowTaskOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, "TASK PB6 DO   ");
    Oled_ShowString(2U, 1U, "> ");
    switch (g_taskField) {
    case TASK_FIELD_TO_TARGET: Oled_ShowString(2U, 3U, "GO TARGET     "); break;
    case TASK_FIELD_HOME: Oled_ShowString(2U, 3U, "GO HOME       "); break;
    case TASK_FIELD_STOP: Oled_ShowString(2U, 3U, "STOP/FREEZE   "); break;
    case TASK_FIELD_CLEAR_LOG: Oled_ShowString(2U, 3U, "CLEAR LOG     "); break;
    case TASK_FIELD_BACK: Oled_ShowString(2U, 3U, "BACK MENU     "); break;
    default: Oled_ShowString(2U, 3U, "UNKNOWN       "); break;
    }
    Oled_ShowString(3U, 1U, "Leg:");
    Oled_ShowUnsigned3(3U, 5U, (uint16_t) g_mission.leg);
    Oled_ShowString(3U, 10U, "S:");
    Oled_ShowUnsigned3(3U, 12U, (uint16_t) g_navState);
    Oled_ShowString(4U, 1U, "PB25/PB0 SEL  ");
}

static void Ui_ShowDebugOled(void)
{
    Oled_Clear();
    Oled_ShowString(1U, 1U, "DEBUG PB6 BACK");
    Oled_ShowString(2U, 1U, "P:");
    Oled_ShowSigned4(2U, 3U, (int16_t) g_poseMap.x_cm);
    Oled_ShowChar(2U, 8U, ',');
    Oled_ShowSigned4(2U, 9U, (int16_t) g_poseMap.y_cm);
    Oled_ShowString(3U, 1U, "E:");
    Oled_ShowSigned4(3U, 3U, (int16_t) g_mission.control_error_cm.x_cm);
    Oled_ShowChar(3U, 8U, ',');
    Oled_ShowSigned4(3U, 9U, (int16_t) g_mission.control_error_cm.y_cm);
    Oled_ShowString(4U, 1U, "Y:");
    Oled_ShowSigned4(4U, 3U, g_poseMap.yaw_deg);
    Oled_ShowString(4U, 9U, "F:");
    Oled_ShowUnsigned3(4U, 11U, (uint16_t) g_planFailCode);
}

static void Ui_ShowOled(uint32_t nowMs)
{
    (void) nowMs;

    switch (g_uiMode) {
    case UI_MODE_MENU:
        Ui_ShowMenuOled();
        break;
    case UI_MODE_COORD:
        Ui_ShowCoordOled();
        break;
    case UI_MODE_MAP:
        Ui_ShowMapOled();
        break;
    case UI_MODE_ZONE:
        Ui_ShowZoneOled();
        break;
    case UI_MODE_TASK:
        Ui_ShowTaskOled();
        break;
    case UI_MODE_DEBUG:
        Ui_ShowDebugOled();
        break;
    case UI_MODE_HOME:
    default:
        Ui_ShowHomeOled();
        break;
    }
}

#endif

static void WirelessW0_PrintHeader(void)
{
    DebugUart_WriteString("wireless_w0.header,time_ms,rx,tx,ok,err,checksum_err,len_err,type_err,tail_err,dup,ignored,update,seq,target,qr,raw_x,raw_y,map_x,map_y,flags,status,event,raw0,raw1,raw2,raw3,raw4,raw5,raw6,raw7,raw8\r\n");
    #if MANUAL_TARGET_INPUT_ENABLE
    DebugUart_WriteString("wireless_w0.boot rx_diag AS32 UART3/openmv 115200_8N1 frame=AABB_target_qr_xy_FF OLED=target_qr_rawxy motor_off\r\n");
#else
    DebugUart_WriteString("wireless_w0.boot rx_diag AS32 UART3/openmv PA14_TX PA13_RX 115200_8N1 MD0_MD1_GND AUX_NC motor_off frame=AABB_target_qr_xy_FF\r\n");
#endif
}

static void WirelessW0_ShowOled(uint32_t nowMs)
{
#if MANUAL_TARGET_INPUT_ENABLE && !WIRELESS_W0_RX_ENABLE
    Ui_ShowOled(nowMs);
    return;
#else
    WirelessFrame_t frame;
    Point2D rawPoint;
    Point2D mapPoint;
    uint8_t rawFrame[WIRELESS_FRAME_LEN];
    uint8_t rawValid;
    uint8_t i;

    __disable_irq();
    WirelessW0_CopyLastFrame(&frame);
    WirelessW0_CopyLastRawFrame(rawFrame, &rawValid);
    __enable_irq();

    rawPoint.x_cm = frame.x_cm;
    rawPoint.y_cm = frame.y_cm;
    mapPoint = WirelessTargetMapPoint(WirelessW0_NormalizeQrId(frame.qr_id),
                                      rawPoint);

    Oled_Clear();
#if WIRELESS_W0_RX_ENABLE
    Oled_ShowString(1U, 1U, "W0 RX 9B 115K ");
#else
    Oled_ShowString(1U, 1U, "W0 TX 9B 115K ");
#endif
    Oled_ShowString(2U, 1U, "RX:");
    Oled_ShowUnsigned4(2U, 4U, WirelessDiag_ClampU16(g_wirelessRxByteCount));
    Oled_ShowString(2U, 9U, "OK:");
    Oled_ShowUnsigned4(2U, 12U, WirelessDiag_ClampU16(g_wirelessFrameOkCount));

    if (rawValid != 0U) {
        Oled_ShowString(3U, 1U, "0:");
        for (i = 0U; i < 5U; ++i) {
            Oled_ShowHexByte(3U, (uint8_t) (3U + i * 2U), rawFrame[i]);
        }
        Oled_ShowString(4U, 1U, "5:");
        for (i = 5U; i < WIRELESS_FRAME_LEN; ++i) {
            Oled_ShowHexByte(4U, (uint8_t) (3U + (i - 5U) * 2U), rawFrame[i]);
        }
        Oled_ShowString(4U, 12U, "I:");
        Oled_ShowUnsigned3(4U, 14U, WirelessDiag_ClampU16(g_wirelessIgnoredCount));
    } else {
        Oled_ShowString(3U, 1U, "NO RAW FRAME    ");
        Oled_ShowString(4U, 1U, "WAIT AA BB ... ");
    }
#endif
}

static bool WirelessW0_SendByte(uint8_t data)
{
    uint32_t startMs = g_tickMs;
    uint32_t spin = 0U;

    while (((uint32_t) (g_tickMs - startMs) < WIRELESS_W0_TX_TIMEOUT_MS) &&
           (spin < WIRELESS_W0_TX_SPIN_LIMIT)) {
        if (DL_UART_Main_transmitDataCheck(openmv_INST, data)) {
            return true;
        }
        spin++;
    }

    g_wirelessTxByteDropCount++;
    return false;
}

static bool WirelessW0_SendBytes(const uint8_t *data, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; ++i) {
        if (!WirelessW0_SendByte(data[i])) {
            return false;
        }
    }
    return true;
}

static void WirelessW0_SendLegacyPing(void)
{
#if WIRELESS_W0_LEGACY_PING_ENABLE
    uint8_t legacyPing[WIRELESS_FRAME_LEN];

    Wireless_BuildTargetFrame(0U, WIRELESS_W0_TARGET_ID, QR_ID_A, 0, 0,
                              WIRELESS_FLAG_VALID, legacyPing);
    (void) WirelessW0_SendBytes(legacyPing, WIRELESS_FRAME_LEN);
#endif
}

static void WirelessW0_SendSample(uint32_t nowMs, uint8_t qrId, int16_t xCm, int16_t yCm)
{
    uint8_t buf[WIRELESS_FRAME_LEN];
    uint8_t seq;
    WirelessFrame_t frame;

    seq = (uint8_t) (g_wirelessTxSeq + 1U);
    if (seq == 0U) {
        seq = 1U;
    }
    g_wirelessTxSeq = seq;
    Wireless_BuildTargetFrame(seq, WIRELESS_W0_TARGET_ID, qrId, xCm, yCm,
                              WIRELESS_FLAG_VALID, buf);
    if (WirelessW0_SendBytes(buf, WIRELESS_FRAME_LEN)) {
        g_wirelessTxFrameCount++;
    }

    frame.header1 = WIRELESS_FRAME_HEADER1;
    frame.header2 = WIRELESS_FRAME_HEADER2;
    frame.len = WIRELESS_FRAME_LEN;
    frame.type = WIRELESS_TYPE_TARGET;
    frame.seq = seq;
    frame.target_id = WIRELESS_W0_TARGET_ID;
    frame.qr_id = qrId;
    frame.x_cm = xCm;
    frame.y_cm = yCm;
    frame.flags = WIRELESS_FLAG_VALID;
    frame.checksum = 0U;
    frame.tail = WIRELESS_FRAME_TAIL;
    WirelessW0_StoreLastFrame(&frame);
#if WIRELESS_W0_DEBUG_LOG_ENABLE
    WirelessW0_PrintLog(nowMs, 'X');
#else
    (void) nowMs;
#endif
}

static void WirelessW0_SendAutoPing(uint32_t nowMs)
{
    WirelessW0_SendSample(nowMs, QR_ID_A, 100, 120);
    WirelessW0_SendLegacyPing();
}

static bool WirelessW0_TakeEvent(uint8_t *eventCode)
{
    bool pending;

    __disable_irq();
    pending = g_wirelessEventPending;
    *eventCode = g_wirelessLastEventCode;
    g_wirelessEventPending = false;
    __enable_irq();
    return pending;
}

static void WirelessW0_Update(uint32_t nowMs)
{
    bool key4Pressed = Key4_IsPressed();
    bool key3Pressed = Key3_IsPressed();
    bool key2Pressed = Key2_IsPressed();
    bool key1Pressed = Key1_IsPressed();
    uint8_t eventCode = 0U;

    Motor_Stop();

#if WIRELESS_W0_AUTO_TX_ENABLE
    if ((nowMs - g_wirelessW0LastAutoTxMs) >= WIRELESS_W0_AUTO_TX_MS) {
        g_wirelessW0LastAutoTxMs = nowMs;
        WirelessW0_SendAutoPing(nowMs);
    }
#endif

#if MANUAL_TARGET_INPUT_ENABLE && !WIRELESS_W0_RX_ENABLE
    Ui_Update(nowMs, key4Pressed, key3Pressed, key2Pressed, key1Pressed);
#else
#if !WIRELESS_W0_RX_ENABLE
    if (key4Pressed && !g_key4LastPressed) {
        WirelessW0_SendSample(nowMs, QR_ID_A, 100, 120);
        WirelessW0_SendLegacyPing();
    }
    if (key3Pressed && !g_key3LastPressed) {
        WirelessW0_SendSample(nowMs, QR_ID_B, -40, 210);
        WirelessW0_SendLegacyPing();
    }
    if (key2Pressed && !g_key2LastPressed) {
        WirelessW0_SendSample(nowMs, QR_ID_C, 230, 150);
        WirelessW0_SendLegacyPing();
    }
#endif
    if (key1Pressed && !g_key1LastPressed) {
#if WIRELESS_W0_DEBUG_LOG_ENABLE
        WirelessW0_PrintLog(nowMs, 'S');
#endif
    }
#endif

    g_key4LastPressed = key4Pressed;
    g_key3LastPressed = key3Pressed;
    g_key2LastPressed = key2Pressed;
    g_key1LastPressed = key1Pressed;

#if WIRELESS_W0_DEBUG_LOG_ENABLE
    if (WirelessW0_TakeEvent(&eventCode)) {
        WirelessW0_PrintLog(nowMs, eventCode);
    } else if ((nowMs - g_wirelessW0LastLogMs) >= WIRELESS_W0_LOG_MS) {
        g_wirelessW0LastLogMs = nowMs;
        WirelessW0_PrintLog(nowMs, 'K');
    }
#else
    (void) eventCode;
#endif

    if ((nowMs - g_wirelessW0LastOledMs) >= WIRELESS_W0_OLED_MS) {
        g_wirelessW0LastOledMs = nowMs;
        WirelessW0_ShowOled(nowMs);
    }
}

static void Navigation_ResetPlan(void)
{
    memset(&g_path, 0, sizeof(g_path));
    g_lastPlan.ok = false;
    g_lastPlan.fail_code = PLAN_FAIL_NONE;
    g_lastPlan.path_len = 0U;
    g_path.source = PATH_SRC_NONE;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    g_radarR1AstarYawSlewLocked = false;
    g_radarR1AstarYawSlewWp = 0U;
    g_radarR1AstarYawSlewDeg = 0;
    g_radarR1AstarFineTryCount = 0U;
    g_radarR1AstarFinePhase = 0U;
    g_radarR1AstarFineStableSinceMs = 0U;
    g_radarR1AstarFineDriveStartMs = 0U;
    g_radarR1BTargetFineSinceMs = 0U;
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    g_radarR1TurnSettleActive = false;
    g_radarR1TurnSettleDone = false;
    g_radarR1PostTurnRampActive = false;
    g_radarR1SegmentSettleActive = false;
    g_radarR1UrgentYStopPending = false;
    g_radarR1UrgentYStopPoseY = 0;
    g_radarR1UrgentYStopMs = 0U;
    g_radarR1TurnSettleUntilMs = 0U;
    g_radarR1TurnSettleStableSinceMs = 0U;
    g_radarR1PostTurnRampStartMs = 0U;
    g_radarR1SegmentSettleUntilMs = 0U;
    g_radarR1TurnSettleLastX = 0;
    g_radarR1TurnSettleLastY = 0;
    g_radarR1YawFilterLocked = false;
    g_radarR1YawFilteredDeg = 0;
    g_radarR1PiYawHaveLast = false;
    g_radarR1PiYawLastRawDeg = 0;
    g_radarR1PiYawFallbackUntilMs = 0U;
    g_radarR1LineErrFilterLocked = false;
    g_radarR1LineErrFilterWp = 0U;
    g_radarR1LineErrFilteredCm = 0;
    RadarR1_ResetLineErrAccum();
    RadarR1_GuardReset();
#endif
    NavSpeedLoop_Reset(g_tickMs);
}

static void Navigation_Init(void)
{
    Mission_InitDefaults(g_tickMs);
    GridMap_Rebuild();
    Navigation_ResetPlan();
#if RADAR_R1_NAV_TEST_ENABLE
    g_navState = NAV_WAIT_TARGET;
#else
    g_navState = (MOTOR_DIR_TEST_ENABLE != 0) ? NAV_MOTOR_TEST : NAV_WAIT_TARGET;
#endif
    g_planFailCode = PLAN_FAIL_NONE;

    SysTick_Config(CPUCLK_FREQ / 1000U);

    MotorDriver_ForceGpioInit();
    Keys_ForcePullupInit();
    Encoder_ForceGpioInit();
    Gray_SelectInit();
    MotorDriver_SetStandby(true);
    Motor_Stop();
    DL_TimerG_startCounter(dianji_g8_INST);
    DL_GPIO_clearPins(GPIO_b13_PORT, GPIO_b13_PIN_jg_PIN);
    Oled_InitMinimal();
    g_motorTestPwm = MOTION_C35_BASE_PWM;
    Oled_ShowMotorSpeed(0, 0, false);

    WirelessUart_Force115200();
    RadarUart_Force115200();
#if WIRELESS_W0_TEST_ENABLE
#if WIRELESS_W0_RX_ENABLE
    UartRxInit(openmv_INST, openmv_INST_INT_IRQN);
#endif
#elif RADAR_R1_NAV_TEST_ENABLE || RADAR_R0_TEST_ENABLE
    UartRxInit(radar_INST, radar_INST_INT_IRQN);
#if WIRELESS_W0_RX_ENABLE
    /* In radar navigation mode the airplane AS32 receiver shares the OpenMV
     * UART.  The previous build only enabled this IRQ in W0-only test mode,
     * so flight frames could arrive electrically but never reach
     * openmv_INST_IRQHandler()/ApplyTargetFromWireless(). */
    UartRxInit(openmv_INST, openmv_INST_INT_IRQN);
#endif
#if JY901_ENABLE
    UartRxInit(gyro_INST, gyro_INST_INT_IRQN);
#endif
#else
    UartRxInit(radar_INST, radar_INST_INT_IRQN);
    UartRxInit(openmv_INST, openmv_INST_INT_IRQN);
#endif
    Motor_UpdateStatusLeds();
}

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
static int32_t RadarR1_FixedTurnPwm(int16_t errYawDeg)
{
    if (Abs16(errYawDeg) <= RADAR_R1_TURN_SLOWDOWN_DEG) {
        return RADAR_R1_TURN_SLOW_PWM;
    }
    return RADAR_R1_TURN_IN_PLACE_PWM;
}

static int32_t RadarR1_FixedFinalTurnPwm(int16_t errYawDeg)
{
    if (Abs16(errYawDeg) <= RADAR_R1_TURN_SLOWDOWN_DEG) {
        return RADAR_R1_FINAL_TURN_SLOW_PWM;
    }
    return RADAR_R1_FINAL_TURN_PWM;
}

static int16_t RadarR1_FixedXTargetFromStart(uint8_t waypointIndex,
                                             int16_t startX)
{
    if (RadarR1_RuntimeBActive() && (waypointIndex == 3U)) {
        /*
         * B-code final short leg targets the QR-side absolute x, but the field
         * log showed yaw=0 turns can shift SLAM x heavily.  Therefore the
         * target remains absolute x=+15 while a B-only encoder cap and low PWM
         * prevent chasing a bad radar x for a long distance.
         */
        (void) startX;
        return RADAR_R1_B_RUNTIME_FINAL_X_CM;
    }
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
    /*
     * A-code field test uses an absolute map X stop.  The previous relative
     * rule used (turn-end x - 81), so when the 180-deg turn ended near x=-33
     * it generated wp_x=-114 and the car stopped too far left.
     */
    if (waypointIndex == 3U) {
        (void) startX;
        return (int16_t) (RADAR_R1_A_FIXED_FINAL_X_SIGN *
                          RADAR_R1_A_FIXED_FINAL_X_CM);
    }
#endif
    return (int16_t) Clamp32(
        (int32_t) startX + RadarR1_FixedRouteXDeltaCm(waypointIndex),
        -32768, 32767);
}

static int16_t RadarR1_FixedXStartFromTarget(uint8_t waypointIndex,
                                             int16_t targetX)
{
    return (int16_t) Clamp32(
        (int32_t) targetX - RadarR1_FixedRouteXDeltaCm(waypointIndex),
        -32768, 32767);
}

static bool RadarR1_BuildFixedRoute(PathBuffer *path)
{
    int16_t startX = (int16_t) g_poseMap.x_cm;
    int16_t startY = (int16_t) g_poseMap.y_cm;
    int16_t finalY = (int16_t) (startY +
        (RadarR1_RuntimeBActive() ?
         RADAR_R1_B_RUNTIME_FINAL_Y_CM :
         RADAR_R1_FIXED_Y_TARGET_CM));

    path->len = 0U;
    path->current_index = 0U;
    path->source = PATH_SRC_FIXED_WAYPOINTS;
    path->points[0].x_cm = startX;
    path->points[0].y_cm = startY;
    path->points[1].x_cm = startX;
    path->points[1].y_cm = finalY;
    path->points[2].x_cm = startX;
    path->points[2].y_cm = finalY;
    path->points[3].x_cm = startX;
    path->points[3].y_cm = finalY;
    path->len = 4U;
#if RADAR_R1_RETURN_CORNER60_ENABLE
    if (RadarR1_RuntimeBActive()) {
        return true;
    }
    /*
     * A-code return smoke test:
     *   after reaching the A-code X leg, stop, turn to +90, then to 0, drive
     *   a short +X return segment, then right-turn to -90 and drive the
     *   starting straight line back toward the original origin.
     */
    path->points[4].x_cm = startX;
    path->points[4].y_cm = finalY;
    path->points[5].x_cm = startX;
    path->points[5].y_cm = finalY;
    path->points[6].x_cm = startX;
    path->points[6].y_cm = finalY;
    path->points[7].x_cm = startX;
    path->points[7].y_cm = finalY;
    path->points[8].x_cm = startX;
    path->points[8].y_cm = startY;
    path->len = 9U;
#endif
    return true;
}

static int16_t RadarR1_FinalTurnLockedY(void)
{
    int16_t lockY = g_path.points[2].y_cm;
    int32_t dy = (int32_t) g_poseMap.y_cm - lockY;

    dy = Clamp32(dy,
                 -RADAR_R1_FINAL_TURN_Y_DRIFT_LIMIT_CM,
                 RADAR_R1_FINAL_TURN_Y_DRIFT_LIMIT_CM);
    return (int16_t) Clamp32((int32_t) lockY + dy, -32768, 32767);
}

static void RadarR1_StartFinalTurnPrestop(uint32_t nowMs, uint8_t finalIndex)
{
    uint32_t holdMs = RADAR_R1_FINAL_TURN_PRESTOP_MS;

    g_path.current_index = finalIndex;
    g_path.points[g_path.current_index].x_cm =
        (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767);
    g_path.points[g_path.current_index].y_cm =
        (int16_t) Clamp32(g_poseMap.y_cm, -32768, 32767);
    if (((g_path.current_index + 1U) < g_path.len) &&
        RadarR1_FixedRouteIsRightLeg((uint8_t) (g_path.current_index + 1U))) {
        g_path.points[g_path.current_index + 1U].y_cm =
            g_path.points[g_path.current_index].y_cm;
    }
    if (RadarR1_FixedRouteIsReturnTurn(finalIndex)) {
        holdMs = (finalIndex == 4U) ?
            RADAR_R1_RETURN_PRETURN_STOP_MS :
            RADAR_R1_RETURN_TURN_GAP_MS;
    }
    g_radarR1SegmentSettleActive = true;
    g_radarR1SegmentSettleUntilMs = nowMs + holdMs;
    g_radarR1TurnSettleStableSinceMs = nowMs;
    g_radarR1TurnSettleLastX = (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767);
    g_radarR1TurnSettleLastY = (int16_t) Clamp32(g_poseMap.y_cm, -32768, 32767);
    g_radarR1PostTurnRampActive = false;
    g_radarR1SlewLeft = 0;
    g_radarR1SlewRight = 0;
    NavSpeedLoop_Reset(nowMs);
    Motor_Stop();
    AppendLog(nowMs, 0, 0);
}
#else
static bool RadarR1_BuildFixedRoute(PathBuffer *path)
{
    (void) path;
    return false;
}
#endif

static void TrackCurrentWaypoint(uint32_t nowMs)
{
    int32_t dx;
    int32_t dy;
    int16_t errDist;
    int16_t desiredYaw;
    int16_t rawDesiredYaw;
    int16_t errYaw;
    int16_t lineErr = 0;
    int16_t lineErrForControl = 0;
    int32_t forward;
    int32_t turn;
    int32_t left;
    int32_t right;
    int32_t maxPwm = MOTOR_PWM_MAX;
    PathPoint *waypoint;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    bool fixedCmdReady = false;
    bool fixedStraightReady = false;
    bool fixedEarlyReached = false;
    int16_t fixedRadarDyCm = 0;
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    bool astarAlignCmd = false;
    bool astarWeakFollowCmd = false;
    bool astarTargetFineActive = false;
    bool astarTargetBonusReached = false;
    bool astarTargetAcceptReached = false;
    bool astarTargetZoneNow = false;
    bool astarInsideForbidden = false;
    int16_t astarRawYawErr = 0;
#endif

    if (g_path.len == 0U) {
        g_navState = NAV_PLAN_TO_TARGET;
        Motor_Stop();
        AppendLog(nowMs, 0, 0);
        return;
    }

    if (g_path.current_index >= g_path.len) {
        ClearTarget();
        Navigation_ResetPlan();
        g_navState = NAV_WAIT_TARGET;
        Motor_Stop();
        AppendLog(nowMs, 0, 0);
        return;
    }

    waypoint = &g_path.points[g_path.current_index];
    dx = waypoint->x_cm - g_poseMap.x_cm;
    dy = waypoint->y_cm - g_poseMap.y_cm;
    errDist = ApproxDistanceCm(dx, dy);
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
        errDist = RadarR1_FixedRouteAxisDistance(g_path.current_index, dx, dy);
    }
#endif

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    if ((g_path.source == PATH_SRC_ASTAR) ||
        (g_path.source == PATH_SRC_ASTAR_HOME)) {
        astarInsideForbidden = RadarR1_AStarInsideHardForbidden();
        if (astarInsideForbidden) {
            g_radarR1ProtectFault = 6U;
            g_uiCarRunEnabled = false;
            g_navState = NAV_DONE;
            Motor_Stop();
            RadarR1_OfflineLogFreeze(nowMs);
            AppendLog(nowMs, errDist, 0);
            return;
        }
        if (RadarR1_AStarHomeStopZoneReached()) {
            g_radarR1StopSource = 7U;
            g_radarR1RouteDone = true;
            g_mission.leg = MISSION_LEG_NONE;
            g_uiCarRunEnabled = false;
            g_radarR1SlewLeft = 0;
            g_radarR1SlewRight = 0;
            NavSpeedLoop_Reset(nowMs);
            Motor_Stop();
            RadarR1_OfflineLogFreeze(nowMs);
            AppendLog(nowMs, RadarR1_AStarHomeDistanceCm(), 0);
            return;
        }
    }

    astarTargetZoneNow = RadarR1_AStarTargetZoneReached();
    if (astarTargetZoneNow && (g_radarR1StopSource == 0U)) {
        g_radarR1StopSource = 5U;
        g_radarR1SlewLeft = 0;
        g_radarR1SlewRight = 0;
        g_radarR1AstarFineTryCount = 0U;
        g_radarR1AstarFinePhase = 0U;
        g_radarR1AstarFineStableSinceMs = 0U;
        g_radarR1AstarFineDriveStartMs = 0U;
        g_radarR1BTargetFineSinceMs = 0U;
        NavSpeedLoop_Reset(nowMs);
        Motor_Stop();
        AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), 0);
        return;
    }
    astarTargetFineActive = astarTargetZoneNow ||
        ((g_radarR1StopSource == 5U) && (g_path.source == PATH_SRC_ASTAR));
    astarTargetBonusReached = RadarR1_AStarTargetBonusZoneReached();
    astarTargetAcceptReached = RadarR1_AStarTargetAcceptZoneReached();
    (void) astarTargetAcceptReached;
    /*
     * Forbidden-zone A* test stops by target-centered QR tolerance, not by the
     * last micro waypoint.  The outer 20x20 cm box is the reliable stop gate;
     * the inner 10x10 cm box is logged as a better "bonus" hit via stop_src=6.
     * This prevents the car from reaching the QR area and then chasing
     * (-80,260) again with small corrective turns.
     */
    if (astarTargetBonusReached) {
        g_radarR1StopSource = 6U;
        g_radarR1SlewLeft = 0;
        g_radarR1SlewRight = 0;
        NavSpeedLoop_Reset(nowMs);
        Motor_Stop();
        if (Mission_ShouldReturnAfterTarget()) {
            /*
             * QR reached directly in the bonus box.  Do not finish the whole
             * mission here: stop for a short radar/pose settle window, then
             * plan the return-home leg from the stable post-stop pose.
             */
            g_arrivedTargetMs = nowMs;
            g_navState = NAV_ARRIVED_TARGET;
        } else {
            g_radarR1RouteDone = true;
            g_uiCarRunEnabled = false;
            RadarR1_OfflineLogFreeze(nowMs);
            ClearTarget();
            Navigation_ResetPlan();
            g_navState = NAV_WAIT_TARGET;
        }
        AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), 0);
        return;
    }
    if (astarTargetFineActive && RadarR1_BTargetSafeAcceptReached()) {
        if (g_radarR1BTargetFineSinceMs == 0U) {
            g_radarR1BTargetFineSinceMs = nowMs;
        }
        if ((nowMs - g_radarR1BTargetFineSinceMs) >=
            RADAR_R1_B_TARGET_FINE_TIMEOUT_MS) {
            g_radarR1StopSource = 5U;
            g_radarR1SlewLeft = 0;
            g_radarR1SlewRight = 0;
            NavSpeedLoop_Reset(nowMs);
            Motor_Stop();
            if (Mission_ShouldReturnAfterTarget()) {
                g_arrivedTargetMs = nowMs;
                g_navState = NAV_ARRIVED_TARGET;
            } else {
                g_radarR1RouteDone = true;
                g_uiCarRunEnabled = false;
                RadarR1_OfflineLogFreeze(nowMs);
                ClearTarget();
                Navigation_ResetPlan();
                g_navState = NAV_WAIT_TARGET;
            }
            AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), 0);
            return;
        }
    } else {
        g_radarR1BTargetFineSinceMs = 0U;
    }
#endif

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
        (g_path.current_index >= 1U) &&
        (g_path.len > 0U)) {
        bool urgentStop = false;
        bool radarStop = false;

        fixedRadarDyCm = (int16_t) Clamp32(
            (int32_t) g_poseMap.y_cm - g_path.points[0].y_cm,
            -32768, 32767);

        /*
         * Keep encoder distance alive after the fixed-route first waypoint is
         * entered.  It is re-zeroed at the moment the +90 turn settle releases
         * the straight leg, so enc_fwd represents straight-line travel for the
         * configured encoder hard cap.  Radar still has priority for the
         * configured Y stop.
         */
        RadarR1_GuardUpdateEncoder();

        if ((g_path.current_index == 1U) && g_radarR1TurnSettleDone) {
            if (RadarR1_GuardShouldAbortStraight(fixedRadarDyCm)) {
                g_uiCarRunEnabled = false;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                g_faultCode = FAULT_POSE_TIMEOUT;
                g_navState = NAV_DONE;
                NavSpeedLoop_Reset(nowMs);
                Motor_Stop();
                RadarR1_OfflineLogFreeze(nowMs);
                AppendLog(nowMs, errDist, 0);
                return;
            }

            urgentStop = RadarR1_TakeUrgentYStop();
            radarStop = RadarR1_FixedYStopGateReached();
            if (urgentStop || radarStop) {
                g_radarR1StopSource = urgentStop ? 1U : 2U;
                fixedEarlyReached = true;
            } else if (RadarR1_GuardEncoderCapReached(g_path.current_index)) {
                /*
                 * Last-resort straight-distance cap.  If radar has not already
                 * stopped by radar, never keep driving past the encoder's
                 * calibrated straight-distance estimate.
                 */
                g_radarR1StopSource = 4U;
                fixedEarlyReached = true;
            }
        } else if (RadarR1_FixedRouteIsRightLeg(g_path.current_index) &&
                   g_radarR1TurnSettleDone &&
                   !g_radarR1SegmentSettleActive) {
            int32_t rightLegStartX =
                RadarR1_FixedXStartFromTarget(g_path.current_index, 
                    g_path.points[g_path.current_index].x_cm);
            int16_t xProgressCm = (int16_t) Clamp32(
                Abs32(rightLegStartX - (int32_t) g_poseMap.x_cm),
                0, 32767);
            if (RadarR1_GuardXCapReached(g_path.current_index)) {
                g_radarR1StopSource = 4U;
                fixedEarlyReached = true;
            } else if (errDist <=
                       (RadarR1_RuntimeBActive() ?
                        RADAR_R1_B_RUNTIME_X_STOP_GATE_CM :
                        RADAR_R1_A_SLOWDOWN_DIST_CM)) {
                g_radarR1StopSource = 2U;
                fixedEarlyReached = true;
            } else if (RadarR1_GuardShouldAbortRightLeg(xProgressCm)) {
                g_uiCarRunEnabled = false;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                g_faultCode = FAULT_POSE_TIMEOUT;
                g_navState = NAV_DONE;
                NavSpeedLoop_Reset(nowMs);
                Motor_Stop();
                RadarR1_OfflineLogFreeze(nowMs);
                AppendLog(nowMs, errDist, 0);
                return;
            }
        } else if (RadarR1_FixedRouteIsReturnHomeLeg(g_path.current_index) &&
                   g_radarR1TurnSettleDone &&
                   !g_radarR1SegmentSettleActive) {
            if (RadarR1_GuardReturnHomeCapReached()) {
                g_radarR1StopSource = 4U;
                fixedEarlyReached = true;
            }
        }
    }
    if (fixedEarlyReached &&
        ((g_path.current_index + 1U) < g_path.len) &&
        RadarR1_FixedRouteIsFinalTurn((uint8_t) (g_path.current_index + 1U))) {
        /*
         * Highest-priority validation stop: as soon as a radar frame or the
         * current control cycle reports relative Y growth >= the tuned stop gate, stop and
         * lock the final yaw waypoint at the actual pose.  This bypasses errDist
         * so the experiment tests the gate/reference, not waypoint hysteresis.
         */
        RadarR1_StartFinalTurnPrestop(nowMs,
            (uint8_t) (g_path.current_index + 1U));
        return;
    }
#endif

    if ((g_path.current_index + 1U) < g_path.len) {
        int16_t reachCm = WAYPOINT_REACH_CM;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
        bool astarAxisReached = RadarR1_AStarAxisWaypointReached(
            &g_path,
            (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767),
            (int16_t) Clamp32(g_poseMap.y_cm, -32768, 32767));
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
        bool fixedAllowReach = true;
        if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
            if (RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
                /* Final-turn waypoint is reached by yaw, not by distance. */
                reachCm = 0;
                fixedAllowReach = false;
            } else if (RadarR1_FixedRouteIsReturnHomeLeg(g_path.current_index)) {
                reachCm = RADAR_R1_A_RETURN_HOME_REACH_CM;
            } else {
                reachCm = (g_path.current_index == 1U) ?
                    RADAR_R1_FIXED_FIRST_REACH_CM : RADAR_R1_FIXED_Y_REACH_CM;
            }
        }
#endif
        if (
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            fixedAllowReach &&
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            (!astarTargetFineActive) &&
#endif
            ((errDist <= reachCm) || fixedEarlyReached
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
             || astarAxisReached
#endif
             )) {
            g_path.current_index++;
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
                /*
                 * The y-stop gate is the real position target for this field
                 * test.  React immediately: stop this control cycle, lock the
                 * final-turn waypoint to the actual pose, wait briefly, and only
                 * then start the route-specific final yaw.  Without this hold the car visibly "does
                 * nothing" at the QR edge because it has already switched to the
                 * final yaw stage and keeps moving while turning.
                 */
                RadarR1_StartFinalTurnPrestop(nowMs, g_path.current_index);
                return;
            }
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                RadarR1_FixedRouteIsRightLeg(g_path.current_index)) {
                g_path.points[g_path.current_index].x_cm =
                    RadarR1_FixedXTargetFromStart(g_path.current_index, 
                        (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767));
                g_path.points[g_path.current_index].y_cm =
                    RadarR1_FinalTurnLockedY();
                RadarR1_GuardReset();
                RadarR1_GuardUpdateEncoder();
                waypoint = &g_path.points[g_path.current_index];
                dx = waypoint->x_cm - g_poseMap.x_cm;
                dy = waypoint->y_cm - g_poseMap.y_cm;
                errDist = RadarR1_FixedRouteAxisDistance(
                    g_path.current_index, dx, dy);
            }
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                RadarR1_FixedRouteNeedsTurnSettle(g_path.current_index)) {
                g_radarR1TurnSettleActive = false;
                g_radarR1TurnSettleDone = false;
                g_radarR1PostTurnRampActive = false;
                g_radarR1TurnSettleUntilMs = 0U;
                g_radarR1TurnSettleStableSinceMs = 0U;
                g_radarR1PostTurnRampStartMs = 0U;
                g_radarR1LineErrFilterLocked = false;
                g_radarR1LineErrFilterWp = 0U;
                g_radarR1LineErrFilteredCm = 0;
                RadarR1_ResetLineErrAccum();
            }
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                (g_path.current_index > 2U) &&
                (g_path.current_index < g_path.len)) {
                /*
                 * Stop at the corner before the next leg so the radar/map and
                 * JY901 yaw have a clean frame before the next turn.
                 */
                g_radarR1SegmentSettleActive = true;
                g_radarR1SegmentSettleUntilMs = nowMs + RADAR_R1_SEGMENT_SETTLE_MS;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                Motor_Stop();
                AppendLog(nowMs, errDist, 0);
                return;
            }
#endif
            waypoint = &g_path.points[g_path.current_index];
            dx = waypoint->x_cm - g_poseMap.x_cm;
            dy = waypoint->y_cm - g_poseMap.y_cm;
            errDist = ApproxDistanceCm(dx, dy);
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
                errDist = RadarR1_FixedRouteAxisDistance(g_path.current_index, dx, dy);
            }
#endif
        }
    } else if (
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
               !((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                 RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) &&
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
               !((g_path.source == PATH_SRC_ASTAR_HOME) &&
                 !RadarR1_AStarHomeStopZoneReached()) &&
#endif
               ((errDist <=
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
                ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) ?
                 RADAR_R1_FIXED_FINAL_REACH_CM : FINAL_REACH_CM)
#else
                FINAL_REACH_CM
#endif
                )
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
                || fixedEarlyReached
#endif
               )) {
        if (g_p7MissionModeEnable || Mission_ShouldReturnAfterTarget()) {
            Motor_Stop();
            if (Mission_ShouldReturnAfterTarget()) {
                g_arrivedTargetMs = nowMs;
                g_navState = NAV_ARRIVED_TARGET;
                AppendLog(nowMs, errDist, 0);
                return;
            }
            g_mission.leg = MISSION_LEG_NONE;
            g_navState = NAV_DONE;
            g_uiCarRunEnabled = false;
            RadarR1_OfflineLogFreeze(nowMs);
            AppendLog(nowMs, errDist, 0);
            return;
        }
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
        if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
            g_radarR1RouteDone = true;
            g_uiCarRunEnabled = false;
            g_radarR1SlewLeft = 0;
            g_radarR1SlewRight = 0;
            Motor_Stop();
            RadarR1_OfflineLogFreeze(nowMs);
            ClearTarget();
            Navigation_ResetPlan();
            g_navState = NAV_WAIT_TARGET;
            AppendLog(nowMs, errDist, 0);
            return;
        }
#endif
        ClearTarget();
        Navigation_ResetPlan();
        g_navState = NAV_WAIT_TARGET;
        Motor_Stop();
        AppendLog(nowMs, errDist, 0);
        return;
    }

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    if ((g_path.source == PATH_SRC_ASTAR) ||
        (g_path.source == PATH_SRC_ASTAR_HOME)) {
        Point2D lookahead = RadarR1_AStarLookaheadTarget(
            &g_path,
            (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767),
            (int16_t) Clamp32(g_poseMap.y_cm, -32768, 32767));
        dx = lookahead.x_cm - g_poseMap.x_cm;
        dy = lookahead.y_cm - g_poseMap.y_cm;
        errDist = ApproxDistanceCm(dx, dy);
        if (astarTargetFineActive) {
            dx = (int32_t) g_targetPoint.target_map_frame.x_cm - g_poseMap.x_cm;
            dy = (int32_t) g_targetPoint.target_map_frame.y_cm - g_poseMap.y_cm;
            errDist = RadarR1_AStarTargetDistanceCm();
        }
    }
#endif

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
    if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
        if (RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
            lineErr = 0;
            lineErrForControl = 0;
            desiredYaw = RadarR1_FixedRouteDesiredYaw(
                g_path.current_index, lineErrForControl, g_poseMap.yaw_deg, nowMs);
        } else if (g_path.current_index >= 1U) {
            lineErr = RadarR1_FixedRouteLineError(g_path.current_index, dx, dy);
            lineErrForControl = RadarR1_FilterLineErrForControl(
                g_path.current_index, lineErr, nowMs);
            desiredYaw = RadarR1_FixedRouteDesiredYaw(
                g_path.current_index, lineErrForControl, g_poseMap.yaw_deg, nowMs);
        } else {
            desiredYaw = ApproxAtan2Deg(dy, dx);
        }
    } else
#endif
    {
        desiredYaw = ApproxAtan2Deg(dy, dx);
    }
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    rawDesiredYaw = desiredYaw;
    if ((g_path.source == PATH_SRC_ASTAR) ||
        (g_path.source == PATH_SRC_ASTAR_HOME)) {
        int16_t slewDelta;
        if ((!g_radarR1AstarYawSlewLocked) ||
            (g_radarR1AstarYawSlewWp != g_path.current_index)) {
            /*
             * Do not snap the controller to a diagonal A* waypoint.  Start the
             * local target heading from the current radar yaw and let it move a
             * few degrees per loop, similar to the known-good A-code 90-degree
             * turn behavior.
             */
            g_radarR1AstarYawSlewLocked = true;
            g_radarR1AstarYawSlewWp = g_path.current_index;
            g_radarR1AstarYawSlewDeg = g_poseMap.yaw_deg;
        }
        slewDelta = NormalizeDeg((int16_t) (rawDesiredYaw -
                                            g_radarR1AstarYawSlewDeg));
        if (slewDelta > RADAR_R1_A_FORBID_YAW_SLEW_DEG) {
            slewDelta = RADAR_R1_A_FORBID_YAW_SLEW_DEG;
        } else if (slewDelta < -RADAR_R1_A_FORBID_YAW_SLEW_DEG) {
            slewDelta = -RADAR_R1_A_FORBID_YAW_SLEW_DEG;
        }
        g_radarR1AstarYawSlewDeg =
            NormalizeDeg((int16_t) (g_radarR1AstarYawSlewDeg + slewDelta));
        desiredYaw = g_radarR1AstarYawSlewDeg;
        astarRawYawErr = NormalizeDeg((int16_t) (rawDesiredYaw -
                                                g_poseMap.yaw_deg));
    } else {
        g_radarR1AstarYawSlewLocked = false;
        g_radarR1AstarYawSlewWp = 0U;
        g_radarR1AstarYawSlewDeg = 0;
    }
#endif
    errYaw = NormalizeDeg((int16_t) (desiredYaw - g_poseMap.yaw_deg));
    /*
     * Do not replan just because the car is still far from the final target.
     * The previous condition compared pose->target distance with REPLAN_DIST_CM;
     * at the start of any multi-waypoint route this is naturally large, so the
     * controller kept bouncing back to NAV_PLAN_TO_TARGET before emitting any
     * waypoint command.  Future replanning should compare path/segment error.
     */

#if RADAR_R1_NAV_TEST_ENABLE
    maxPwm = RADAR_R1_MAX_PWM;
#if RADAR_R1_FIXED_ROUTE_TEST
    if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
        if (g_radarR1SegmentSettleActive) {
            bool segmentPoseStable = true;
            if (RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
                uint32_t poseAgeMs = g_poseMap.valid ?
                    (nowMs - g_poseMap.stamp_ms) : 9999U;
                int32_t settleDx =
                    (int32_t) g_poseMap.x_cm - g_radarR1TurnSettleLastX;
                int32_t settleDy =
                    (int32_t) g_poseMap.y_cm - g_radarR1TurnSettleLastY;
                if ((Abs32(settleDx) > RADAR_R1_FINAL_TURN_PRE_POSE_STABLE_CM) ||
                    (Abs32(settleDy) > RADAR_R1_FINAL_TURN_PRE_POSE_STABLE_CM)) {
                    g_radarR1TurnSettleLastX = (int16_t) g_poseMap.x_cm;
                    g_radarR1TurnSettleLastY = (int16_t) g_poseMap.y_cm;
                    g_radarR1TurnSettleStableSinceMs = nowMs;
                }
                segmentPoseStable = g_poseMap.valid &&
                    (poseAgeMs <= RADAR_R1_TURN_FRESH_MAX_MS) &&
                    ((nowMs - g_radarR1TurnSettleStableSinceMs) >=
                     RADAR_R1_FINAL_TURN_PRE_STABLE_MS);
            }
            if (((int32_t) (nowMs - g_radarR1SegmentSettleUntilMs) < 0) ||
                !segmentPoseStable) {
                Motor_Stop();
                AppendLog(nowMs, errDist, errYaw);
                return;
            }
            g_radarR1SegmentSettleActive = false;
        }
#if RADAR_R1_TURN_SETTLE_MS > 0
        if (RadarR1_FixedRouteNeedsTurnSettle(g_path.current_index) &&
            !g_radarR1TurnSettleDone) {
            if (g_radarR1TurnSettleActive) {
                bool poseStable = false;
                uint32_t poseAgeMs = g_poseMap.valid ? (nowMs - g_poseMap.stamp_ms) : 9999U;
#if JY901_USE_YAW_FOR_R1 && !RADAR_R1_USE_BRIDGE_YAW_ONLY
                uint32_t gyroAgeMs = g_jy901Valid ? (nowMs - g_jy901StampMs) : 9999U;
#endif
                int32_t settleDx = (int32_t) g_poseMap.x_cm - g_radarR1TurnSettleLastX;
                int32_t settleDy = (int32_t) g_poseMap.y_cm - g_radarR1TurnSettleLastY;
                if ((Abs32(settleDx) > RADAR_R1_TURN_POSE_STABLE_CM) ||
                    (Abs32(settleDy) > RADAR_R1_TURN_POSE_STABLE_CM)) {
                    g_radarR1TurnSettleLastX = (int16_t) g_poseMap.x_cm;
                    g_radarR1TurnSettleLastY = (int16_t) g_poseMap.y_cm;
                    g_radarR1TurnSettleStableSinceMs = nowMs;
                }
                poseStable = g_poseMap.valid &&
                    (poseAgeMs <= RADAR_R1_TURN_FRESH_MAX_MS) &&
#if JY901_USE_YAW_FOR_R1 && !RADAR_R1_USE_BRIDGE_YAW_ONLY
                    (gyroAgeMs <= RADAR_R1_TURN_FRESH_MAX_MS) &&
#else
                    true &&
#endif
                    ((nowMs - g_radarR1TurnSettleStableSinceMs) >=
                     RADAR_R1_TURN_POSE_STABLE_MS);
                if (((int32_t) (nowMs - g_radarR1TurnSettleUntilMs) < 0) ||
                    !poseStable) {
                    Motor_Stop();
                    AppendLog(nowMs, errDist, errYaw);
                    return;
                }
                g_radarR1TurnSettleActive = false;
                /*
                 * After the map/yaw settle window, re-check angle before
                 * releasing the straight leg.  If yaw drifted out of the
                 * small settle band while stopped, apply a low-power fine
                 * correction and require a fresh settle window again.  This
                 * gives the weak/slow side time to catch up without entering
                 * the straight leg with a biased heading.
                 */
                if (Abs16(errYaw) > RADAR_R1_TURN_SETTLE_RECHECK_DEG) {
                    forward = 0;
                    turn = (errYaw >= 0) ? RADAR_R1_HEADING_CORRECT_PWM :
                                           -RADAR_R1_HEADING_CORRECT_PWM;
                    fixedCmdReady = true;
                } else {
                    g_radarR1TurnSettleDone = true;
                    g_radarR1PostTurnRampActive = true;
                    g_radarR1PostTurnRampStartMs = nowMs;
                    if (g_path.current_index == 1U) {
                        g_path.points[g_path.current_index].x_cm =
                            (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767);
                        /*
                         * Start the encoder hard cap from the straight leg, not
                         * from the preceding in-place turn.  The generic guard
                         * still keeps counting once straight motion begins.
                         */
                        RadarR1_GuardReset();
                        RadarR1_GuardUpdateEncoder();
                        g_radarR1LineErrFilterLocked = false;
                        g_radarR1LineErrFilterWp = 0U;
                        g_radarR1LineErrFilteredCm = 0;
                        RadarR1_ResetLineErrAccum();
                    }
                    forward = 0;
                    turn = 0;
                }
            } else if (Abs16(errYaw) <= RADAR_R1_TURN_SETTLE_DEG) {
                /*
                 * The first straight segment is radar-accurate, while the
                 * first left turn temporarily perturbs mapping.  Once JY901B
                 * yaw says the car is aligned to the y+ leg, stop for 2s and
                 * let the radar/map pose settle before restarting straight.
                 */
                g_radarR1TurnSettleActive = true;
                /*
                 * Keep the first left-turn pause long enough for Cartographer
                 * and JY901B to agree before the 2 m y+ leg.  The final right
                 * turn only precedes a short X leg; the latest log stopped
                 * while still in ctrl_stage=4 after about 0.4 s, so use a
                 * shorter dedicated settle there to avoid looking "dead" after
                 * the right turn while still waiting for a fresh stable pose.
                 */
                if (g_path.current_index == 1U) {
                    g_radarR1TurnSettleUntilMs =
                        nowMs + RADAR_R1_FIRST_TURN_SETTLE_MS;
                } else if (RadarR1_FixedRouteIsRightLeg(g_path.current_index)) {
                    g_radarR1TurnSettleUntilMs =
                        nowMs + RADAR_R1_RIGHT_TURN_SETTLE_MS;
                } else {
                    g_radarR1TurnSettleUntilMs =
                        nowMs + RADAR_R1_TURN_SETTLE_MS;
                }
                g_radarR1TurnSettleStableSinceMs = nowMs;
                g_radarR1TurnSettleLastX = (int16_t) g_poseMap.x_cm;
                g_radarR1TurnSettleLastY = (int16_t) g_poseMap.y_cm;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                Motor_Stop();
                AppendLog(nowMs, errDist, errYaw);
                return;
            } else {
                forward = 0;
                turn = (errYaw >= 0) ? RadarR1_FixedTurnPwm(errYaw) :
                                       -RadarR1_FixedTurnPwm(errYaw);
                fixedCmdReady = true;
            }
        }
#endif
        if (!fixedCmdReady &&
            RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
            if (Abs16(errYaw) <= RADAR_R1_FIXED_FINAL_TURN_REACH_DEG) {
                if (((g_path.current_index + 1U) < g_path.len) &&
                    RadarR1_FixedRouteIsFinalTurn((uint8_t) (g_path.current_index + 1U))) {
                    g_path.current_index++;
                    g_radarR1SegmentSettleActive = true;
                    g_radarR1SegmentSettleUntilMs = nowMs +
                        ((RadarR1_FixedRouteIsReturnTurn(g_path.current_index)) ?
                         RADAR_R1_RETURN_TURN_GAP_MS :
                         RADAR_R1_SEGMENT_SETTLE_MS);
                    g_radarR1SlewLeft = 0;
                    g_radarR1SlewRight = 0;
                    NavSpeedLoop_Reset(nowMs);
                    Motor_Stop();
                    AppendLog(nowMs, errDist, errYaw);
                    return;
                }
                if (((g_path.current_index + 1U) < g_path.len) &&
                    RadarR1_FixedRouteIsRightLeg((uint8_t) (g_path.current_index + 1U))) {
                    g_path.current_index++;
                    g_path.points[g_path.current_index].x_cm =
                        RadarR1_FixedXTargetFromStart(g_path.current_index,
                            (int16_t) Clamp32(g_poseMap.x_cm, -32768, 32767));
                    g_path.points[g_path.current_index].y_cm =
                        RadarR1_FinalTurnLockedY();
                    RadarR1_GuardReset();
                    RadarR1_GuardUpdateEncoder();
                    g_radarR1SegmentSettleActive = true;
                    g_radarR1SegmentSettleUntilMs = nowMs +
                        ((RadarR1_FixedRouteIsReturnXLeg(g_path.current_index)) ?
                         RADAR_R1_RETURN_X_RELEASE_MS :
                         RADAR_R1_FINAL_X_RELEASE_MS);
                    g_radarR1TurnSettleDone = true;
                    g_radarR1PostTurnRampActive = true;
                    g_radarR1PostTurnRampStartMs = nowMs;
                    g_radarR1LineErrFilterLocked = false;
                    g_radarR1LineErrFilterWp = 0U;
                    g_radarR1LineErrFilteredCm = 0;
                    RadarR1_ResetLineErrAccum();
                    g_radarR1SlewLeft = 0;
                    g_radarR1SlewRight = 0;
                    NavSpeedLoop_Reset(nowMs);
                    Motor_Stop();
                    AppendLog(nowMs, errDist, errYaw);
                    return;
                }
                if (((g_path.current_index + 1U) < g_path.len) &&
                    RadarR1_FixedRouteIsReturnHomeLeg((uint8_t) (g_path.current_index + 1U))) {
                    g_path.current_index++;
                    g_path.points[g_path.current_index].x_cm = g_path.points[0].x_cm;
                    g_path.points[g_path.current_index].y_cm = g_path.points[0].y_cm;
                    RadarR1_GuardReset();
                    RadarR1_GuardUpdateEncoder();
                    g_radarR1SegmentSettleActive = true;
                    g_radarR1SegmentSettleUntilMs = nowMs + RADAR_R1_RETURN_HOME_RELEASE_MS;
                    g_radarR1TurnSettleDone = true;
                    g_radarR1PostTurnRampActive = true;
                    g_radarR1PostTurnRampStartMs = nowMs;
                    g_radarR1LineErrFilterLocked = false;
                    g_radarR1LineErrFilterWp = 0U;
                    g_radarR1LineErrFilteredCm = 0;
                    RadarR1_ResetLineErrAccum();
                    g_radarR1SlewLeft = 0;
                    g_radarR1SlewRight = 0;
                    NavSpeedLoop_Reset(nowMs);
                    Motor_Stop();
                    AppendLog(nowMs, errDist, errYaw);
                    return;
                }
                g_radarR1RouteDone = true;
                g_uiCarRunEnabled = false;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                Motor_Stop();
                g_navState = NAV_DONE;
                AppendLog(nowMs, errDist, errYaw);
                RadarR1_OfflineLogFreeze(nowMs);
                return;
            }
            forward = 0;
            turn = (errYaw >= 0) ? RadarR1_FixedFinalTurnPwm(errYaw) :
                                   -RadarR1_FixedFinalTurnPwm(errYaw);
            fixedCmdReady = true;
        }
        if (!fixedCmdReady &&
            !RadarR1_FixedRouteIsFinalTurn(g_path.current_index) &&
            (g_path.current_index >= 1U) &&
            g_radarR1TurnSettleDone &&
            (Abs16(errYaw) > RADAR_R1_STRAIGHT_HEADING_CORRECT_DEG)) {
            /*
             * After the one mandatory post-turn settle, do not re-enter the
             * 2s settle gate every time yaw crosses the threshold.  The
             * r1_dual_l230_h42_c24 log showed that a pure in-place reverse
             * pair (+/-160) during a straight leg makes the car walk in a
             * stop-and-go pattern.  Keep this as a heading recovery stage,
             * but bias it forward so the inner wheel is near zero instead of
             * reversing; this should smooth the straight segment while still
             * preventing very large yaw drift.
             */
            forward = RADAR_R1_STRAIGHT_RECOVERY_FORWARD_PWM;
            turn = (errYaw >= 0) ? RADAR_R1_STRAIGHT_RECOVERY_TURN_PWM :
                                   -RADAR_R1_STRAIGHT_RECOVERY_TURN_PWM;
            fixedCmdReady = true;
        }
        if (!fixedCmdReady &&
            !RadarR1_FixedRouteIsFinalTurn(g_path.current_index) &&
            (g_path.current_index >= 1U)) {
            /*
             * Straight segments use the measured open-loop wheel bias.  The
             * loaded chassis previously went straight at about L520/R700; at
             * the current 300-PWM safety ceiling that scales to about L225/R300.
             * Only a small JY901 yaw correction is added, avoiding the wide
             * left-right hunting seen after the first turn.
             */
            int32_t corrLimit = g_radarR1PostTurnRampActive ?
                RADAR_R1_POST_TURN_CORR_MAX_PWM : RADAR_R1_STRAIGHT_CORR_MAX_PWM;
            int32_t yawGain = RADAR_R1_STRAIGHT_YAW_GAIN;
            if ((g_path.current_index == 1U) &&
                (g_radarR1PostTurnRampStartMs != 0U) &&
                ((nowMs - g_radarR1PostTurnRampStartMs) <
                 RADAR_R1_LEFT_TURN_BUFFER_MS)) {
                /*
                 * The right-turn x+ leg is already straight.  The remaining
                 * bad window is only after the first left turn: yaw drifts
                 * away during the first 1~2 s after release, before the speed
                 * loop/cumulative line bias is allowed to close.  Temporarily
                 * give the yaw hold more authority on wp2, then fall back to
                 * the normal gentle straight-line parameters.
                 */
                yawGain = RADAR_R1_LEFT_TURN_BUFFER_YAW_GAIN;
                if (corrLimit < RADAR_R1_LEFT_TURN_BUFFER_CORR_MAX_PWM) {
                    corrLimit = RADAR_R1_LEFT_TURN_BUFFER_CORR_MAX_PWM;
                }
            }
            int32_t correction = Clamp32((int32_t) errYaw * yawGain,
                                         -corrLimit, corrLimit);
            left = RADAR_R1_STRAIGHT_BASE_L - correction;
            right = RADAR_R1_STRAIGHT_BASE_R + correction;
#if RADAR_R1_FIXED_ROUTE_TARGET == RADAR_R1_ROUTE_A
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                ((g_path.current_index == 1U) ||
                 (!RadarR1_RuntimeBActive() && (g_path.current_index == 3U)))) {
                int32_t remainCm = (g_path.current_index == 1U) ?
                    ((int32_t) (RadarR1_RuntimeBActive() ?
                                RADAR_R1_B_RUNTIME_FINAL_Y_CM :
                                RADAR_R1_FIXED_FINAL_Y_CM) -
                     ((int32_t) g_poseMap.y_cm - g_path.points[0].y_cm)) :
                    Abs32((int32_t) waypoint->x_cm - (int32_t) g_poseMap.x_cm);
                if ((remainCm > 0) && (remainCm <= RADAR_R1_A_SLOWDOWN_DIST_CM)) {
                    left = (left * RADAR_R1_A_SLOW_SCALE_PCT) / 100;
                    right = (right * RADAR_R1_A_SLOW_SCALE_PCT) / 100;
                }
            }
#endif
            if (g_radarR1PostTurnRampActive) {
                uint32_t elapsedMs = nowMs - g_radarR1PostTurnRampStartMs;
                int32_t scalePct;
                if (elapsedMs >= RADAR_R1_POST_TURN_RAMP_MS) {
                    scalePct = 100;
                    /*
                     * Keep g_radarR1PostTurnRampActive alive until
                     * RADAR_R1_POST_TURN_YAW_ONLY_MS so desired_yaw stays on
                     * the pure segment heading while Cartographer pose/line_err
                     * is still settling after a turn.  The actual PWM ramp and
                     * left boost still end at POST_TURN_RAMP_MS.
                     */
                    if (elapsedMs >= RADAR_R1_POST_TURN_YAW_ONLY_MS) {
                        g_radarR1PostTurnRampActive = false;
                    }
                } else {
                    scalePct = RADAR_R1_POST_TURN_SCALE_START_PCT +
                        (int32_t) ((elapsedMs *
                            (100 - RADAR_R1_POST_TURN_SCALE_START_PCT)) /
                            RADAR_R1_POST_TURN_RAMP_MS);
                }
                left = (left * scalePct) / 100;
                right = (right * scalePct) / 100;
                if (elapsedMs < RADAR_R1_POST_TURN_RAMP_MS) {
                    int32_t boost = RADAR_R1_POST_TURN_LEFT_BOOST_PWM -
                        (int32_t) ((elapsedMs * RADAR_R1_POST_TURN_LEFT_BOOST_PWM) /
                                   RADAR_R1_POST_TURN_RAMP_MS);
                    left += boost;
                }
            }
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                RadarR1_RuntimeBActive() && (g_path.current_index == 3U)) {
                /*
                 * B-code final leg is short.  Use the same idea as A-code:
                 * reduce speed inside the final 20 cm, but keep a separate
                 * 30 cm encoder cap as the hard overrun guard.
                 */
                int32_t remainCm = Abs32((int32_t) waypoint->x_cm -
                                          (int32_t) g_poseMap.x_cm);
                if ((remainCm > 0) &&
                    (remainCm <= RADAR_R1_B_RUNTIME_X_SLOWDOWN_DIST_CM)) {
                    left = (left * RADAR_R1_B_RUNTIME_X_SLOW_SCALE_PCT) / 100;
                    right = (right * RADAR_R1_B_RUNTIME_X_SLOW_SCALE_PCT) / 100;
                }
            }
            forward = (left + right) / 2;
            turn = (right - left) / 2;
            fixedCmdReady = true;
            fixedStraightReady = true;
        }
        if (!fixedCmdReady && (Abs16(errYaw) > RADAR_R1_ALIGN_DEG)) {
            forward = 0;
            turn = (errYaw >= 0) ? RadarR1_FixedTurnPwm(errYaw) :
                                   -RadarR1_FixedTurnPwm(errYaw);
        } else {
            if (fixedCmdReady) {
                /* forward/turn already prepared above. */
            } else {
            forward = errDist * MOTOR_FORWARD_GAIN;
            if (forward > maxPwm) {
                forward = maxPwm;
            }
            if (RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
                forward = 0;
            } else if (Abs16(errYaw) > RADAR_R1_DRIVE_DEG) {
                forward = RADAR_R1_MIN_PWM;
            } else if ((forward > 0) && (forward < RADAR_R1_MIN_PWM)) {
                forward = RADAR_R1_MIN_PWM;
            }
            turn = Clamp32((int32_t) errYaw * RADAR_R1_TURN_GAIN,
                           -RADAR_R1_TURN_MAX_PWM, RADAR_R1_TURN_MAX_PWM);
            }
        }
    } else
#endif
    {
    /*
     * R1 low-speed waypoint follower.
     * The last field run reached wp=2 but then kept moving past the corner;
     * with delayed/slow pose updates, hard in-place turns at every medium yaw
     * error cause the car to "乱拐".  Keep forward motion on medium yaw error,
     * cap turn separately, and only spin slowly when the waypoint is almost
     * behind the car.
     */
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
#if RADAR_R1_A_FORBID_WEAK_FOLLOW_ENABLE
    if ((g_path.source == PATH_SRC_ASTAR) ||
        (g_path.source == PATH_SRC_ASTAR_HOME)) {
        bool astarHomeLeg = (g_path.source == PATH_SRC_ASTAR_HOME);
        int16_t bigYawDeg = astarHomeLeg ?
            RADAR_R1_ASTAR_HOME_BIG_YAW_DEG :
            RADAR_R1_A_FORBID_BIG_YAW_DEG;
        int16_t midYawDeg = astarHomeLeg ?
            RADAR_R1_ASTAR_HOME_MID_YAW_DEG :
            RADAR_R1_A_FORBID_MID_YAW_DEG;
        int32_t bigTurnPwm = astarHomeLeg ?
            RADAR_R1_ASTAR_HOME_BIG_TURN_PWM :
            RADAR_R1_A_FORBID_BIG_TURN_PWM;
        int32_t midForwardPwm = astarHomeLeg ?
            RADAR_R1_ASTAR_HOME_MID_FORWARD_PWM :
            RADAR_R1_A_FORBID_MID_FORWARD_PWM;
        int32_t weakTurnMaxPwm = astarHomeLeg ?
            RADAR_R1_ASTAR_HOME_WEAK_TURN_MAX_PWM :
            RADAR_R1_A_FORBID_WEAK_TURN_MAX_PWM;
        int32_t weakForwardPwm = RADAR_R1_A_FORBID_WEAK_FORWARD_PWM;
        bool astarNearForbidden = RadarR1_AStarNearForbiddenBoundary();
        bool astarHomeFinalLeg =
            astarHomeLeg && (((uint8_t) (g_path.current_index + 1U)) >= g_path.len);
        int16_t astarHomeDistCm = astarHomeLeg ?
            RadarR1_AStarHomeDistanceCm() : 32767;
        int16_t absFilteredYawErr = Abs16(errYaw);
        int16_t absRawYawErr = Abs16(astarRawYawErr);
        if (astarHomeLeg && astarNearForbidden) {
            bigYawDeg = RADAR_R1_ASTAR_HOME_NEAR_FORBID_BIG_YAW_DEG;
            midYawDeg = RADAR_R1_ASTAR_HOME_NEAR_FORBID_MID_YAW_DEG;
            midForwardPwm = RADAR_R1_ASTAR_HOME_NEAR_FORBID_FORWARD_PWM;
            weakForwardPwm = RADAR_R1_ASTAR_HOME_NEAR_FORBID_FORWARD_PWM;
            weakTurnMaxPwm = RADAR_R1_ASTAR_HOME_NEAR_FORBID_TURN_MAX_PWM;
        }
        if (astarHomeFinalLeg &&
            (astarHomeDistCm <= RADAR_R1_ASTAR_HOME_FINAL_REVERSE_DIST_CM) &&
            (absFilteredYawErr >= RADAR_R1_ASTAR_HOME_FINAL_REVERSE_YAW_DEG)) {
            int16_t homeYaw = ApproxAtan2Deg(
                (int32_t) g_mission.map.home_map_frame.y_cm - g_poseMap.y_cm,
                (int32_t) g_mission.map.home_map_frame.x_cm - g_poseMap.x_cm);
            int16_t reverseYaw = NormalizeDeg((int16_t) (homeYaw + 180));
            int16_t reverseErrYaw =
                NormalizeDeg((int16_t) (reverseYaw - g_poseMap.yaw_deg));

            desiredYaw = reverseYaw;
            errYaw = reverseErrYaw;
            if (Abs16(reverseErrYaw) >
                RADAR_R1_ASTAR_HOME_FINAL_REVERSE_ALIGN_DEG) {
                forward = 0;
                turn = (reverseErrYaw >= 0) ?
                    RADAR_R1_ASTAR_HOME_FINAL_REVERSE_ALIGN_PWM :
                    -RADAR_R1_ASTAR_HOME_FINAL_REVERSE_ALIGN_PWM;
                astarAlignCmd = true;
            } else {
                forward = -RADAR_R1_ASTAR_HOME_FINAL_REVERSE_PWM;
                turn = Clamp32((int32_t) reverseErrYaw *
                                   RADAR_R1_A_FORBID_WEAK_TURN_GAIN,
                               -RADAR_R1_ASTAR_HOME_FINAL_REVERSE_TURN_MAX_PWM,
                               RADAR_R1_ASTAR_HOME_FINAL_REVERSE_TURN_MAX_PWM);
                astarWeakFollowCmd = true;
            }
        } else if (astarTargetFineActive) {
            /*
             * Inside the reliable parking box, do not use normal waypoint
             * pursuit anymore.  From logs, the old fine mode could keep normal
             * momentum and run away after stop_src=5.  Limit fine correction to
             * two explicit cycles: align yaw, wait for a stable yaw window, then
             * creep once with lower-than-current forward PWM.  If the bonus box
             * is still not reached after two cycles, freeze at the reliable stop
             * instead of circling away from the QR.
             */
            if (g_radarR1AstarFineTryCount >=
                RADAR_R1_A_FORBID_TARGET_FINE_MAX_TRIES) {
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                NavSpeedLoop_Reset(nowMs);
                Motor_Stop();
                if (Mission_ShouldReturnAfterTarget()) {
                    /*
                     * Fine-stop used both allowed correction cycles.  Freeze at
                     * the reliable QR box, wait for radar pose to settle, then
                     * return home instead of waiting for another key press.
                     */
                    g_arrivedTargetMs = nowMs;
                    g_navState = NAV_ARRIVED_TARGET;
                } else {
                    g_radarR1RouteDone = true;
                    g_uiCarRunEnabled = false;
                    RadarR1_OfflineLogFreeze(nowMs);
                    ClearTarget();
                    Navigation_ResetPlan();
                    g_navState = NAV_WAIT_TARGET;
                }
                AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), errYaw);
                return;
            }

            if (g_radarR1AstarFinePhase == 0U) {
                int32_t fineTurnLimit =
                    (Abs16(errYaw) > RADAR_R1_A_FORBID_TARGET_FINE_BIG_ERR_DEG) ?
                    RADAR_R1_A_FORBID_TARGET_FINE_BIG_TURN_MAX_PWM :
                    RADAR_R1_A_FORBID_TARGET_FINE_TURN_MAX_PWM;
                forward = 0;
                turn = 0;
                if (Abs16(errYaw) <=
                    RADAR_R1_A_FORBID_TARGET_FINE_APPROACH_YAW_DEG) {
                    /*
                     * In the reliable target box, coordinates matter more than
                     * pointing exactly at the target center.  If the heading is
                     * not dangerous, allow a short fine approach instead of
                     * spinning in place forever at the QR-code edge.
                     */
                    if (g_radarR1AstarFineStableSinceMs == 0U) {
                        g_radarR1AstarFineStableSinceMs = nowMs;
                    }
                    if ((Abs16(errYaw) <=
                         RADAR_R1_A_FORBID_TARGET_FINE_ALIGN_DEG) &&
                        ((nowMs - g_radarR1AstarFineStableSinceMs) >=
                        RADAR_R1_A_FORBID_TARGET_FINE_STABLE_MS)) {
                        g_radarR1AstarFinePhase = 1U;
                        g_radarR1AstarFineDriveStartMs = nowMs;
                        g_radarR1SlewLeft = 0;
                        g_radarR1SlewRight = 0;
                        NavSpeedLoop_Reset(nowMs);
                    } else if (Abs16(errYaw) >
                               RADAR_R1_A_FORBID_TARGET_FINE_ALIGN_DEG) {
                        g_radarR1AstarFinePhase = 1U;
                        g_radarR1AstarFineDriveStartMs = nowMs;
                        g_radarR1SlewLeft = 0;
                        g_radarR1SlewRight = 0;
                        NavSpeedLoop_Reset(nowMs);
                    }
                } else {
                    forward = 0;
                    turn = Clamp32((int32_t) errYaw *
                                       RADAR_R1_A_FORBID_WEAK_TURN_GAIN,
                                   -fineTurnLimit, fineTurnLimit);
                    g_radarR1AstarFineStableSinceMs = 0U;
                }
            } else {
                forward = RADAR_R1_A_FORBID_TARGET_FINE_DRIVE_PWM;
                turn = Clamp32((int32_t) errYaw *
                                   RADAR_R1_A_FORBID_WEAK_TURN_GAIN,
                               -RADAR_R1_A_FORBID_TARGET_FINE_APPROACH_TURN_MAX_PWM,
                               RADAR_R1_A_FORBID_TARGET_FINE_APPROACH_TURN_MAX_PWM);
                if ((nowMs - g_radarR1AstarFineDriveStartMs) >=
                    RADAR_R1_A_FORBID_TARGET_FINE_DRIVE_MS) {
                    g_radarR1AstarFineTryCount++;
                    g_radarR1AstarFinePhase = 0U;
                    g_radarR1AstarFineStableSinceMs = 0U;
                    g_radarR1AstarFineDriveStartMs = 0U;
                    g_radarR1SlewLeft = 0;
                    g_radarR1SlewRight = 0;
                    NavSpeedLoop_Reset(nowMs);
                    Motor_Stop();
                    AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), errYaw);
                    return;
                }
            }
        } else if (absFilteredYawErr > bigYawDeg) {
            /*
             * If the car did not keep up with the slewed heading, behave like
             * the stable A-code turn: stop forward motion and correct angle
             * slowly.  This prevents the previous "forward while 40~60 deg
             * wrong" loop into the forbidden zone.
            */
            forward = 0;
            turn = (errYaw >= 0) ? bigTurnPwm : -bigTurnPwm;
            astarAlignCmd = true;
        } else if ((absFilteredYawErr > midYawDeg) ||
                   (absRawYawErr > 70)) {
            forward = midForwardPwm;
            turn = Clamp32((int32_t) errYaw * RADAR_R1_A_FORBID_WEAK_TURN_GAIN,
                           -weakTurnMaxPwm, weakTurnMaxPwm);
        } else {
            forward = weakForwardPwm;
            turn = Clamp32((int32_t) errYaw * RADAR_R1_A_FORBID_WEAK_TURN_GAIN,
                           -weakTurnMaxPwm, weakTurnMaxPwm);
        }
        /*
         * Fine-stop already owns its PWM budget.  Do not run the normal A*
         * near-target minimum-PWM lift here, otherwise the intended fine creep
         * (85 PWM in this build) is raised back to RADAR_R1_MIN_PWM and the car
         * looks like it "directly stops/runs through" instead of doing two
         * small correction attempts inside the target box.
         */
        if (!astarTargetFineActive) {
            if (errDist < 40) {
                forward = (forward * RADAR_R1_A_FORBID_NEAR_SCALE_PCT) / 100;
            }
            if ((g_path.source == PATH_SRC_ASTAR) &&
                (RadarR1_AStarTargetDistanceCm() <=
                 RADAR_R1_A_FORBID_TARGET_SLOW_DIST_CM)) {
                forward = (forward *
                           RADAR_R1_A_FORBID_TARGET_SLOW_SCALE_PCT) / 100;
                if ((forward > 0) && (forward < RADAR_R1_MIN_PWM)) {
                    forward = RADAR_R1_MIN_PWM;
                }
            }
        }
        astarWeakFollowCmd = true;
    } else
#endif
    if (((g_path.source == PATH_SRC_ASTAR) ||
         (g_path.source == PATH_SRC_ASTAR_HOME)) &&
        (Abs16(errYaw) > RADAR_R1_A_FORBID_ALIGN_DEG)) {
        forward = 0;
        turn = (errYaw >= 0) ? RADAR_R1_A_FORBID_TURN_PWM :
                               -RADAR_R1_A_FORBID_TURN_PWM;
        astarAlignCmd = true;
    } else
#endif
    if (Abs16(errYaw) > 120) {
        forward = 0;
        turn = Clamp32((int32_t) errYaw * RADAR_R1_SPIN_GAIN,
                       -RADAR_R1_SPIN_MAX_PWM, RADAR_R1_SPIN_MAX_PWM);
    } else {
        forward = errDist * MOTOR_FORWARD_GAIN;
        if (forward > maxPwm) {
            forward = maxPwm;
        }
        if (Abs16(errYaw) > 70) {
            forward = RADAR_R1_MIN_PWM;
        } else if (Abs16(errYaw) > RADAR_R1_A_FORBID_DRIVE_DEG) {
            forward = (RADAR_R1_MIN_PWM + maxPwm) / 2;
        } else if ((forward > 0) && (forward < RADAR_R1_MIN_PWM)) {
            forward = RADAR_R1_MIN_PWM;
        }
        turn = Clamp32((int32_t) errYaw * RADAR_R1_TURN_GAIN,
                       -RADAR_R1_TURN_MAX_PWM, RADAR_R1_TURN_MAX_PWM);
    }
    }
#else
    forward = errDist * MOTOR_FORWARD_GAIN;
    if (forward > maxPwm) {
        forward = maxPwm;
    }
    if (Abs16(errYaw) > TURN_IN_PLACE_DEG) {
        forward = 0;
    } else if ((forward > 0) && (forward < MOTOR_PWM_MIN)) {
        forward = MOTOR_PWM_MIN;
    }

    turn = errYaw * MOTOR_TURN_GAIN;
    turn = Clamp32(turn, -maxPwm, maxPwm);
#endif
    left = forward - turn;
    right = forward + turn;

#if RADAR_R1_NAV_TEST_ENABLE
#if RADAR_R1_FIXED_ROUTE_TEST
    if (!fixedStraightReady &&
        (forward == 0) &&
        (turn > 0) &&
        (right > 0) &&
        !RadarR1_FixedRouteIsFinalTurn(g_path.current_index)) {
        /*
         * Left turn in-place: the field symptom is that the right wheel
         * drives the turn faster than the loaded/slow left side can follow.
         * Scale only the right-forward side during left turns, giving the
         * left wheel more time to react.  Right turns keep the normal pair so
         * the final X leg can expose asymmetry in the next log.
         */
        right = (right * RADAR_R1_LEFT_TURN_RIGHT_SCALE_PCT) / 100;
    }
#endif
    {
        int32_t minPwm = RADAR_R1_MIN_PWM;
#if RADAR_R1_FIXED_ROUTE_TEST
        if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) && !fixedStraightReady) {
            minPwm = RADAR_R1_TURN_MIN_PWM;
        }
        if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
            fixedStraightReady && RadarR1_RuntimeBActive() &&
            (g_path.current_index == 3U)) {
            minPwm = RADAR_R1_TURN_MIN_PWM;
        }
#endif
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
        if (astarTargetFineActive) {
            minPwm = 0;
        } else if (astarAlignCmd || astarWeakFollowCmd) {
            minPwm = RADAR_R1_TURN_MIN_PWM;
        }
#endif
        if ((left > 0) && (left < minPwm)) {
            left = minPwm;
        } else if ((left < 0) && (left > -minPwm)) {
            left = -minPwm;
        }

        if ((right > 0) && (right < minPwm)) {
            right = minPwm;
        } else if ((right < 0) && (right > -minPwm)) {
            right = -minPwm;
        }
    }
#else
    if ((left > 0) && (left < MOTOR_PWM_MIN)) {
        left = MOTOR_PWM_MIN;
    } else if ((left < 0) && (left > -MOTOR_PWM_MIN)) {
        left = -MOTOR_PWM_MIN;
    }

    if ((right > 0) && (right < MOTOR_PWM_MIN)) {
        right = MOTOR_PWM_MIN;
    } else if ((right < 0) && (right > -MOTOR_PWM_MIN)) {
        right = -MOTOR_PWM_MIN;
    }
#endif

    /*
     * R1 uses a stage-specific safety ceiling. The previous code capped
     * forward and turn separately but allowed their sum to exceed the R1
     * limit, which showed up in leidar.log as wheel commands above max.
     */
    left = Clamp32(left, -maxPwm, maxPwm);
    right = Clamp32(right, -maxPwm, maxPwm);

#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
    if (((g_path.source == PATH_SRC_ASTAR) ||
         (g_path.source == PATH_SRC_ASTAR_HOME)) &&
        RadarR1_AStarNearForbiddenBoundary() &&
        (left > 0) && (right > 0)) {
        int8_t forbidSide = RadarR1_AStarNearestForbiddenSide();
        int32_t forbidDistCm = RadarR1_AStarNearestForbiddenDistanceCm();
        int32_t protectMinPwm = (g_path.source == PATH_SRC_ASTAR_HOME) ?
            RADAR_R1_ASTAR_HOME_NEAR_FORBID_MIN_PWM : RADAR_R1_MIN_PWM;
        int32_t escapePwm = (g_path.source == PATH_SRC_ASTAR_HOME) ?
            RADAR_R1_ASTAR_HOME_FORBID_PROTECT_ESCAPE_PWM :
            RADAR_R1_FORBID_PROTECT_ESCAPE_PWM;

        left = (left * RADAR_R1_FORBID_PROTECT_SCALE_PCT) / 100;
        right = (right * RADAR_R1_FORBID_PROTECT_SCALE_PCT) / 100;

        if (left < protectMinPwm) {
            left = protectMinPwm;
        }
        if (right < protectMinPwm) {
            right = protectMinPwm;
        }

        /*
         * Protection band must not only slow down.  Add a small differential
         * escape push on the wheel nearest the forbidden edge, so the chassis
         * turns gently away instead of stopping beside the boundary.  If the
         * forbidden edge is nearly straight ahead/behind and has no left/right
         * side, follow the current yaw-correction direction rather than driving
         * straight deeper into the band.
         */
        if (forbidSide == 0) {
            forbidSide = (errYaw <= 0) ? 1 : -1;
        }
        if ((g_path.source == PATH_SRC_ASTAR_HOME) &&
            (forbidDistCm <= RADAR_R1_FORBID_DANGER_DIST_CM)) {
            if (forbidSide > 0) {
                left = RADAR_R1_FORBID_DANGER_OUTER_PWM;
                right = RADAR_R1_FORBID_DANGER_INNER_PWM;
            } else {
                left = RADAR_R1_FORBID_DANGER_INNER_PWM;
                right = RADAR_R1_FORBID_DANGER_OUTER_PWM;
            }
        } else {
            if (forbidSide > 0) {
                left += escapePwm;
            } else if (forbidSide < 0) {
                right += escapePwm;
            }
        }
        left = Clamp32(left, -maxPwm, maxPwm);
        right = Clamp32(right, -maxPwm, maxPwm);
        g_radarR1ProtectFault = 5U;
    } else if (g_radarR1ProtectFault == 5U) {
        g_radarR1ProtectFault = 0U;
    }
#endif

#if RADAR_R1_NAV_TEST_ENABLE
    if ((RADAR_R1_FORWARD_LEFT_BIAS_PWM != 0) &&
        (left > 0) && (right > 0)) {
        /*
         * Full-battery field runs still show a slight left-drift tendency on
         * long A-star/return forward segments.  Add only a forward-motion trim to
         * the left wheel; in-place turns and reverse/escape states keep their
         * previous balance.
         */
        left += RADAR_R1_FORWARD_LEFT_BIAS_PWM;
        left = Clamp32(left, -maxPwm, maxPwm);
    }
#if RADAR_R1_FIXED_ROUTE_TEST
    if (fixedStraightReady &&
        (left > 0) && (right > 0) &&
        ((g_radarR1SlewLeft <= 0) || (g_radarR1SlewRight <= 0))) {
        /*
         * Preserve the good L295/R300 straight bias, but do not enter that
         * straight command from a spin state with one wheel still ramping
         * through zero.  The latest straight reference log showed the first
         * short section after a turn drifting left while the two wheel commands
         * were crossing from opposite signs to forward at different times.
         * Resetting the slew state makes both wheels restart from the same
         * zero baseline before the normal per-update ramp takes over.
         */
        g_radarR1SlewLeft = 0;
        g_radarR1SlewRight = 0;
    }
#endif
    /*
     * Keep first real R1 motion conservative.  This also prevents a single
     * noisy yaw sample from instantly reversing the wheel pair.
     */
    left = Clamp32(left,
                   (int32_t) g_radarR1SlewLeft - RADAR_R1_CMD_SLEW_PER_UPDATE,
                   (int32_t) g_radarR1SlewLeft + RADAR_R1_CMD_SLEW_PER_UPDATE);
    right = Clamp32(right,
                    (int32_t) g_radarR1SlewRight - RADAR_R1_CMD_SLEW_PER_UPDATE,
                    (int32_t) g_radarR1SlewRight + RADAR_R1_CMD_SLEW_PER_UPDATE);
    left = Clamp32(left, -maxPwm, maxPwm);
    right = Clamp32(right, -maxPwm, maxPwm);
    if (fixedStraightReady &&
        g_radarR1PostTurnRampActive &&
        (g_path.current_index == 1U) &&
        (left > 0) && (right > 0) &&
        ((nowMs - g_radarR1PostTurnRampStartMs) < RADAR_R1_POST_TURN_RAMP_MS)) {
        /*
         * The latest dual-loop log showed the first post-left-turn straight
         * command can still be logged around 120/120 because the global slew
         * limiter restarts from zero.  That is below/near the loaded left-wheel
         * breakaway region, so the car waits for the right side and then the
         * pose outer loop has to recover with a large head swing.  Only for the
         * first y+ leg startup window, lift both wheels just above the practical
         * dead zone, with a slightly higher left floor.  This avoids changing
         * the already acceptable right-turn x+ leg.
         */
        if (left < RADAR_R1_POST_TURN_START_MIN_L) {
            left = RADAR_R1_POST_TURN_START_MIN_L;
        }
        if (right < RADAR_R1_POST_TURN_START_MIN_R) {
            right = RADAR_R1_POST_TURN_START_MIN_R;
        }
        left = Clamp32(left, -maxPwm, maxPwm);
        right = Clamp32(right, -maxPwm, maxPwm);
    }
    if (g_uiCarRunEnabled) {
        g_radarR1SlewLeft = (int16_t) left;
        g_radarR1SlewRight = (int16_t) right;
    } else {
        g_radarR1SlewLeft = 0;
        g_radarR1SlewRight = 0;
    }
#endif

#if RADAR_R1_NAV_TEST_ENABLE
#if NAV_SPEED_LOOP_ENABLE
    {
        bool navSpeedCloseAllowed =
            NavSpeedLoop_CanCloseFromCmd((int16_t) left, (int16_t) right);
#if RADAR_R1_FIXED_ROUTE_TEST
        if (g_path.source == PATH_SRC_FIXED_WAYPOINTS) {
            /*
             * Keep the encoder loop out of the transient path-following work.
             * The latest log showed the S-curve is dominated by yaw/line
             * recovery after a turn; during that window the pose outer loop
             * must own the wheel split.  The speed loop is only allowed once
             * we are on a same-direction straight command, the post-turn
             * yaw-only window is over, and yaw error is already small.
             */
            navSpeedCloseAllowed = navSpeedCloseAllowed &&
                fixedStraightReady &&
                !g_radarR1PostTurnRampActive &&
                (Abs16(errYaw) <= 12);
        }
#endif
        if (navSpeedCloseAllowed) {
#if RADAR_R1_FIXED_ROUTE_TEST
            if ((g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
                fixedStraightReady &&
                (g_path.current_index >= 1U)) {
                int16_t lineBias = RadarR1_LineErrAccumSpeedBiasPwm(
                    g_path.current_index, lineErrForControl, errYaw, nowMs);
                /*
                 * For the final X leg, positive lineErr means target Y
                 * is above the car; this needs a slight right-turn bias, same
                 * wheel split direction as the +Y leg accumulator.
                 */
                left += lineBias;
                right -= lineBias;
                left = Clamp32(left, -maxPwm, maxPwm);
                right = Clamp32(right, -maxPwm, maxPwm);
            } else {
                RadarR1_ResetLineErrAccum();
            }
#endif
            NavSpeedLoop_SetTargetFromPwm((int16_t) left, (int16_t) right);
            if (g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) {
                NavSpeedLoop_Update(nowMs, true);
            } else {
                /*
                 * Phase 8C is a dry-run planning test: keep motor_out=0, but
                 * log the outer pose controller's theoretical wheel commands.
                 * Calling NavSpeedLoop_Update(false) resets the dry output to
                 * 0, hiding whether MCU planning would drive/turn correctly.
                 */
                Motor_SetOutputDry((int16_t) left, (int16_t) right);
            }
        } else {
#if RADAR_R1_FIXED_ROUTE_TEST
            RadarR1_ResetLineErrAccum();
#endif
            NavSpeedLoop_Reset(nowMs);
            if (g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) {
                Motor_SetOutput((int16_t) left, (int16_t) right);
            } else {
                Motor_SetOutputDry((int16_t) left, (int16_t) right);
            }
        }
    }
#else
    if (g_uiCarRunEnabled && (RADAR_R1_MOTOR_OUTPUT_ENABLE != 0)) {
        Motor_SetOutput((int16_t) left, (int16_t) right);
    } else {
        Motor_SetOutputDry((int16_t) left, (int16_t) right);
    }
#endif
#else
    Motor_SetOutput((int16_t) left, (int16_t) right);
#endif
    AppendLog(nowMs, errDist, errYaw);
}

#if !SPEED_C4_TEST_ENABLE
static void MotorDirectionTest_Update(uint32_t nowMs)
{
    bool key4Pressed;
    bool key3Pressed;
    bool key2Pressed;
    bool key1Pressed;
    int16_t left = 0;
    int16_t right = 0;

    g_devStage = DEV_STAGE_C0_MOTOR_DIR;
    g_navState = NAV_MOTOR_TEST;
    g_faultCode = FAULT_NONE;

    key4Pressed = Key4_IsPressed();
    key3Pressed = Key3_IsPressed();
    key2Pressed = Key2_IsPressed();
    key1Pressed = Key1_IsPressed();

    if (key4Pressed && !g_key4LastPressed && !g_motionTestPulse) {
        g_motionTestPulse = true;
        g_motionTestStartMs = nowMs;
        ++g_motionTestRunId;
        g_motionTestTrim = -1;
        left = g_motorTestPwm;
        right = 0;
        Encoder_ResetCounts();
        MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_START, left, right);
        g_motorLogLastStep = 0xFFFFFFFFU;
    }
    g_key4LastPressed = key4Pressed;

    if (key3Pressed && !g_key3LastPressed && !g_motionTestPulse) {
        g_motionTestPulse = true;
        g_motionTestStartMs = nowMs;
        ++g_motionTestRunId;
        g_motionTestTrim = 1;
        left = 0;
        right = g_motorTestPwm;
        Encoder_ResetCounts();
        MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_START, left, right);
        g_motorLogLastStep = 0xFFFFFFFFU;
    }
    g_key3LastPressed = key3Pressed;

    if (key2Pressed && !g_key2LastPressed && !g_motionTestPulse) {
        g_motionTestPulse = true;
        g_motionTestStartMs = nowMs;
        ++g_motionTestRunId;
        g_motionTestTrim = 0;
        left = g_motorTestPwm;
        right = g_motorTestPwm;
        Encoder_ResetCounts();
        MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_START, left, right);
        g_motorLogLastStep = 0xFFFFFFFFU;
    }
    g_key2LastPressed = key2Pressed;

    if (key1Pressed && !g_key1LastPressed && !g_motionTestPulse) {
        MotionC35_LogExport(nowMs);
        g_motorLogLastStep = 0xFFFFFFFFU;
    }
    g_key1LastPressed = key1Pressed;

    if (g_motionTestPulse && ((nowMs - g_motionTestStartMs) >= MotionTest_CurrentDurationMs())) {
        g_motionTestPulse = false;
        MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_STOP, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight());
        Motor_Stop();
        g_motorLogLastStep = 0xFFFFFFFFU;
    }

    if (g_motionTestPulse) {
        if (g_motionTestTrim < 0) {
            left = g_motorTestPwm;
            right = 0;
        } else if (g_motionTestTrim > 0) {
            left = 0;
            right = g_motorTestPwm;
        } else {
            left = g_motorTestPwm;
            right = g_motorTestPwm;
        }
        Motor_SetOutput(left, right);
    } else {
        Motor_Stop();
    }

    if ((g_motorLogLastStep != (uint32_t) (g_motionTestTrim + 1)) ||
        (g_motionTestPulse && ((nowMs - g_motorLogLastMs) >= 100U))) {
        g_motorLogLastStep = (uint32_t) (g_motionTestTrim + 1);
        g_motorLogLastMs = nowMs;
        if (g_motionTestPulse) {
            MotionC35_LogAppend(nowMs, MOTION_C35_EVENT_RUN, MotorDriver_GetLastLeft(), MotorDriver_GetLastRight());
        }
        Motor_LogChain(nowMs, 0U);
        Oled_ShowMotorSpeed(MotorDriver_GetLastLeft(), MotorDriver_GetLastRight(), g_motionTestPulse);
    }

    AppendLog(nowMs, 0, 0);
}
#endif

static void Navigation_Update(uint32_t nowMs)
{
    bool havePose = PoseIsFresh(nowMs);
    bool haveTarget = TargetIsFresh(nowMs);
    bool needTarget = (g_mission.leg != MISSION_LEG_TO_HOME);

    Mission_UpdatePoseTargetError();

    if (!havePose) {
        g_poseMap.valid = false;
        g_faultCode = FAULT_POSE_TIMEOUT;
    }
    if (!haveTarget && needTarget) {
        g_targetPoint.valid = false;
        if (g_navState != NAV_WAIT_TARGET) {
            g_faultCode = FAULT_TARGET_TIMEOUT;
        }
    }

    switch (g_navState) {
    case NAV_WAIT_CONFIG:
        Motor_Stop();
        if (Mission_ConfigIsValid()) {
            g_faultCode = FAULT_NONE;
            g_navState = haveTarget ? (havePose ? NAV_READY : NAV_WAIT_POSE) : NAV_WAIT_TARGET;
        } else {
            g_faultCode = FAULT_CONFIG_INVALID;
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_WAIT_TARGET:
        Motor_Stop();
        g_faultCode = FAULT_NONE;
        if (!Mission_ConfigIsValid()) {
            g_navState = NAV_WAIT_CONFIG;
            g_faultCode = FAULT_CONFIG_INVALID;
            AppendLog(nowMs, 0, 0);
            break;
        }
        if (haveTarget) {
            g_navState = havePose ? (g_uiCarRunEnabled ? NAV_PLAN_TO_TARGET : NAV_READY) : NAV_WAIT_POSE;
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_WAIT_POSE:
        Motor_Stop();
        if (!Mission_ConfigIsValid()) {
            g_navState = NAV_WAIT_CONFIG;
            g_faultCode = FAULT_CONFIG_INVALID;
        } else if (!haveTarget && needTarget) {
            g_navState = NAV_WAIT_TARGET;
            g_faultCode = FAULT_TARGET_TIMEOUT;
        } else if (havePose) {
            g_faultCode = FAULT_NONE;
            if (g_uiCarRunEnabled) {
                g_navState = (g_mission.leg == MISSION_LEG_TO_HOME) ?
                    NAV_PLAN_HOME : NAV_PLAN_TO_TARGET;
            } else {
                g_navState = NAV_READY;
            }
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_READY:
        Motor_Stop();
        if (!Mission_ConfigIsValid()) {
            g_navState = NAV_WAIT_CONFIG;
            g_faultCode = FAULT_CONFIG_INVALID;
        } else if (!havePose) {
            g_navState = NAV_WAIT_POSE;
            g_faultCode = FAULT_POSE_TIMEOUT;
        } else if (!haveTarget && needTarget) {
            g_navState = NAV_WAIT_TARGET;
            g_faultCode = FAULT_TARGET_TIMEOUT;
        } else if (g_uiCarRunEnabled) {
            g_navState = (g_mission.leg == MISSION_LEG_TO_HOME) ?
                NAV_PLAN_HOME : NAV_PLAN_TO_TARGET;
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_PLAN_TO_TARGET:
    case NAV_PLAN_HOME:
        Motor_Stop();
        if (!Mission_ConfigIsValid()) {
            g_navState = NAV_WAIT_CONFIG;
            g_planFailCode = PLAN_FAIL_NONE;
            g_faultCode = FAULT_CONFIG_INVALID;
            Navigation_ResetPlan();
            AppendLog(nowMs, 0, 0);
            break;
        }
        if (!haveTarget && (g_navState == NAV_PLAN_TO_TARGET)) {
            g_navState = NAV_WAIT_TARGET;
            g_planFailCode = PLAN_FAIL_NONE;
            g_faultCode = FAULT_TARGET_TIMEOUT;
            Navigation_ResetPlan();
            AppendLog(nowMs, 0, 0);
            break;
        }
        if (!havePose) {
            g_navState = NAV_WAIT_POSE;
            g_faultCode = FAULT_POSE_TIMEOUT;
            AppendLog(nowMs, 0, 0);
            break;
        }
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_ZERO_AT_FIRST_POSE
        if (!g_radarR1OriginLocked) {
            g_navState = NAV_WAIT_POSE;
            g_faultCode = FAULT_POSE_TIMEOUT;
            AppendLog(nowMs, 0, 0);
            break;
        }
#endif
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
        /*
         * Fixed-route tests must be judged as one continuous action chain.
         * If a short pose/target timeout temporarily sends the state machine
         * back to PLAN_TO_TARGET, do not rebuild the route from the current
         * pose; that was seen as wp=2 falling back to wp=1 and it moves the
         * planned right-turn corner.  PB7/Navigation_ResetPlan() is the only
         * normal way to start a new fixed-route geometry.
         */
        if (!g_p7MissionModeEnable &&
            (g_path.source == PATH_SRC_FIXED_WAYPOINTS) &&
            (g_path.len > 0U) &&
            (g_path.current_index > 0U) &&
            !g_radarR1RouteDone) {
            g_lastPlan.ok = true;
            g_lastPlan.fail_code = PLAN_FAIL_NONE;
            g_lastPlan.path_len = g_path.len;
        } else
        if (!g_p7MissionModeEnable && RadarR1_BuildFixedRoute(&g_path)) {
            g_lastPlan.ok = true;
            g_lastPlan.fail_code = PLAN_FAIL_NONE;
            g_lastPlan.path_len = g_path.len;
        } else {
            int16_t savedObstacleMarginCm = g_mission.map.obstacle_margin_cm;
            bool homeAStarPlan = (g_mission.leg == MISSION_LEG_TO_HOME);

#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            if (homeAStarPlan &&
                (g_mission.map.obstacle_margin_cm <
                 RADAR_R1_ASTAR_HOME_PLAN_MARGIN_CM)) {
                g_mission.map.obstacle_margin_cm =
                    RADAR_R1_ASTAR_HOME_PLAN_MARGIN_CM;
            }
#endif
            GridMap_Rebuild();
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE && RADAR_R1_A_FORBID_ORTHO_ROUTE_ENABLE
            if (RadarR1_BuildAForbidOrthoRoute(
                    (Point2D){g_poseMap.x_cm, g_poseMap.y_cm},
                    Mission_CurrentGoal(),
                    &g_path)) {
                g_lastPlan.ok = true;
                g_lastPlan.fail_code = PLAN_FAIL_NONE;
                g_lastPlan.path_len = g_path.len;
            } else
#endif
            {
            g_lastPlan = PlanPathAStar((Point2D){g_poseMap.x_cm, g_poseMap.y_cm},
                                       Mission_CurrentGoal(),
                                       &g_path);
#if RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            if (!g_lastPlan.ok && homeAStarPlan &&
                (g_mission.map.obstacle_margin_cm != savedObstacleMarginCm)) {
                g_mission.map.obstacle_margin_cm = savedObstacleMarginCm;
                GridMap_Rebuild();
                g_lastPlan = PlanPathAStar(
                    (Point2D){g_poseMap.x_cm, g_poseMap.y_cm},
                    Mission_CurrentGoal(),
                    &g_path);
            }
#endif
            if (g_lastPlan.ok) {
                g_path.source = (g_mission.leg == MISSION_LEG_TO_HOME) ?
                    PATH_SRC_ASTAR_HOME : PATH_SRC_ASTAR;
            }
            }
            if (g_mission.map.obstacle_margin_cm != savedObstacleMarginCm) {
                g_mission.map.obstacle_margin_cm = savedObstacleMarginCm;
                GridMap_Rebuild();
            }
        }
#else
        GridMap_Rebuild();
        g_lastPlan = PlanPathAStar((Point2D){g_poseMap.x_cm, g_poseMap.y_cm},
                                   Mission_CurrentGoal(),
                                   &g_path);
#endif
        g_planFailCode = g_lastPlan.fail_code;
        if (g_lastPlan.ok) {
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_FIXED_ROUTE_TEST
            if ((g_path.source != PATH_SRC_FIXED_WAYPOINTS) ||
                (g_path.current_index == 0U)) {
                g_path.current_index = (g_path.len > 1U) ? 1U : 0U;
            }
#else
            g_path.source = PATH_SRC_ASTAR;
            g_path.current_index = (g_path.len > 1U) ? 1U : 0U;
#endif
            g_faultCode = FAULT_NONE;
            g_navState = (g_mission.leg == MISSION_LEG_TO_HOME) ?
                NAV_GO_HOME : NAV_GO_TARGET;
        } else {
            g_faultCode = FAULT_PLAN_FAILED;
            g_navState = NAV_ERROR;
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_GO_TARGET:
    case NAV_GO_HOME:
        if (!haveTarget && (g_navState == NAV_GO_TARGET)) {
            g_navState = NAV_WAIT_TARGET;
            g_faultCode = FAULT_TARGET_TIMEOUT;
            Navigation_ResetPlan();
            Motor_Stop();
            AppendLog(nowMs, 0, 0);
            break;
        }
        if (!havePose) {
            g_navState = NAV_WAIT_POSE;
            g_faultCode = FAULT_POSE_TIMEOUT;
            Motor_Stop();
            AppendLog(nowMs, 0, 0);
            break;
        }
        TrackCurrentWaypoint(nowMs);
        break;

    case NAV_ARRIVED_TARGET:
        Motor_Stop();
        if ((nowMs - g_arrivedTargetMs) >= RADAR_R1_TARGET_TO_HOME_SETTLE_MS) {
#if RADAR_R1_NAV_TEST_ENABLE && RADAR_R1_A_FORBID_ASTAR_TEST_ENABLE
            if ((RADAR_R1_PRE_RETURN_FINE_ENABLE != 0) &&
                !g_radarR1PreReturnFineUsed &&
                (g_mission.leg == MISSION_LEG_TO_TARGET) &&
                RadarR1_AStarTargetZoneReached() &&
                !RadarR1_AStarTargetBonusZoneReached()) {
                /*
                 * Return-home starts from the QR stop pose.  If the first
                 * target stop is only inside the reliable 20x20 box but not
                 * the bonus 10x10 box, spend exactly one extra fine attempt
                 * after a radar-settle pause, then return.  This avoids
                 * starting the home leg from a visibly bad QR-side pose while
                 * preventing endless target chasing.
                 */
                g_radarR1PreReturnFineUsed = true;
                g_radarR1StopSource = 5U;
                g_radarR1AstarFineTryCount =
                    (RADAR_R1_A_FORBID_TARGET_FINE_MAX_TRIES > 0U) ?
                    (uint8_t) (RADAR_R1_A_FORBID_TARGET_FINE_MAX_TRIES - 1U) :
                    0U;
                g_radarR1AstarFinePhase = 0U;
                g_radarR1AstarFineStableSinceMs = 0U;
                g_radarR1AstarFineDriveStartMs = 0U;
                g_radarR1SlewLeft = 0;
                g_radarR1SlewRight = 0;
                NavSpeedLoop_Reset(nowMs);
                g_navState = NAV_GO_TARGET;
                AppendLog(nowMs, RadarR1_AStarTargetDistanceCm(), 0);
                break;
            }
#endif
            g_mission.leg = MISSION_LEG_TO_HOME;
            Navigation_ResetPlan();
            g_navState = NAV_PLAN_HOME;
        }
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_DONE:
        Motor_Stop();
        AppendLog(nowMs, 0, 0);
        break;

    case NAV_ERROR:
    default:
        Motor_Stop();
        if (!haveTarget && needTarget) {
            g_navState = NAV_WAIT_TARGET;
            g_planFailCode = PLAN_FAIL_NONE;
            g_faultCode = FAULT_NONE;
        } else if (havePose) {
            /*
             * If the first return-home plan fails or times out once, keep the
             * mission on the home leg.  The old fallback always jumped back to
             * PLAN_TO_TARGET, which can leave the car "busy running" at the QR
             * after C/A fine-stop instead of retrying the return-to-start path.
             */
            g_navState = (g_mission.leg == MISSION_LEG_TO_HOME) ?
                NAV_PLAN_HOME : NAV_PLAN_TO_TARGET;
        }
        AppendLog(nowMs, 0, 0);
        break;
    }

    Motor_UpdateStatusLeds();
}

int main(void)
{
    uint32_t lastLoopMs = 0U;

    SYSCFG_DL_init();
    DebugUart_InitPa10Pa11();
    Navigation_Init();
#if WIRELESS_W0_TEST_ENABLE
#if WIRELESS_W0_DEBUG_LOG_ENABLE
    WirelessW0_PrintHeader();
#endif
    WirelessW0_SetEvent('B', WIRELESS_PARSE_NONE);
    WirelessW0_ShowOled(g_tickMs);
#elif RADAR_R1_NAV_TEST_ENABLE
    RadarR1_PrintHeader();
    RadarR1_ShowOled(g_tickMs, true);
#elif RADAR_R0_TEST_ENABLE
    RadarR0_PrintHeader();
    RadarR0_ShowOled(g_tickMs);
#else
#if SPEED_C4_TEST_ENABLE
    DebugUart_WriteString("speed_c4.header,time_ms,run_id,pulse_index,pulse_total,elapsed_ms,mode,control,phase,target_l,target_r,speed_l,speed_r,err_l,err_r,pwm_l,pwm_r,count_l,count_r,sync,norm_err,yaw_delta,i_l,i_r,sat_l,sat_r,left_ab,right_ab,event\r\n");
#if SPEED_C4_OPEN_LOOP_DIAG
    DebugUart_WriteString("speed_c4.boot active_low c4.11_window_open PB0=595_800_6s PB25=590_800_6s PB7=600_800_6s PB6=export warmup=1000ms rscale=640/1000 no_pid\r\n");
#else
    DebugUart_WriteString("speed_c4.boot active_low c5.1_disturb PB0=base_pid_2s PB25=left_dead_600ms PB7=right_dead_600ms PB6=export target=800 ff=600/800 low_target=450 low_pwm=340/450 rscale=640/1000 kpdiv=70 ki=0 sync_dist=50 limit=25 sample=200ms\r\n");
#endif
    SpeedC4_LogAppend(g_tickMs, SPEED_C4_EVENT_BOOT, 0, 0, 0, 0, 0, 0, 0, 0U, 0U);
#else
    DebugUart_WriteString("encoder_c36.header,time_ms,run_id,pwm,mode,pwm_l,pwm_r,duration_ms,enc_l_delta,enc_r_delta,speed_l,speed_r,left_ab,right_ab,event\r\n");
    DebugUart_WriteString("encoder_c36.boot active_low PB25=right_1s PB0=left_1s PB7=both_1s PB6=export pwm=300\r\n");
    MotionC35_LogAppend(g_tickMs, MOTION_C35_EVENT_BOOT, 0, 0);
#endif
#endif
#if !RADAR_R1_NAV_TEST_ENABLE && !RADAR_R0_TEST_ENABLE && !WIRELESS_W0_TEST_ENABLE
    Gray_Log(g_tickMs);
#endif

    while (1) {
        uint32_t nowMs = g_tickMs;
        if ((nowMs - lastLoopMs) >= LOOP_PERIOD_MS) {
            lastLoopMs = nowMs;
#if WIRELESS_W0_TEST_ENABLE
            WirelessW0_Update(nowMs);
#elif RADAR_R1_NAV_TEST_ENABLE
            RadarR1_Update(nowMs);
#elif RADAR_R0_TEST_ENABLE
            RadarR0_Update(nowMs);
#else
            if (MOTOR_DIR_TEST_ENABLE != 0) {
#if SPEED_C4_TEST_ENABLE
                SpeedC4_Update(nowMs);
#else
                MotorDirectionTest_Update(nowMs);
#endif
            } else {
                Navigation_Update(nowMs);
            }
#endif
        }
    }
}





