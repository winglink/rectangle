//
// Created by wing on 15/6/10.
//

//
// Created by wing on 15/6/10.
//

//
// Created by wing on 15/6/9.
//
#include "Copter.h"
void rectangle_update();

// newflightmode_init - initialise flight mode
bool Copter::rectangle_init(bool ignore_checks)
{

    printf("rectangle_init()\r\n");

    if(position_ok()||ignore_checks) {
        pos_control.init_xy_controller();
        pos_control.set_speed_xy(300);
        pos_control.set_accel_xy(60);
        pos_control.set_speed_z(250, 150);
        pos_control.set_accel_z(100);
        pos_control.calc_leash_length_xy();
        pos_control.calc_leash_length_z();
        return true;
    }
    else
        return false;

}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
void Copter::rectangle_run()
{
    printf("rectangle_run()\r\n");
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete || !motors.get_interlock()) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.set_alt_target_to_current_alt();
        return;
    }
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
    }
    rectangle_update();
    // convert pilot input into desired vehicle angles or rotation rates
    //   g.rc_1.control_in : pilots roll input in the range -4500 ~ 4500
    //   g.rc_2.control_in : pilot pitch input in the range -4500 ~ 4500
    //   g.rc_3.control_in : pilot's throttle input in the range 0 ~ 1000
    //   g.rc_4.control_in : pilot's yaw input in the range -4500 ~ 4500

    // call one of attitude controller's attitude control functions like
    //   attitude_control.angle_ef_roll_pitch_rate_yaw(roll angle, pitch angle, yaw rate);

    // call position controller's z-axis controller or simply pass through throttle
    //   attitude_control.set_throttle_out(desired throttle, true);

}
void Copter::rectangle_update(){
    float dt=pos_control.time_since_last_xy_update();
    float tot_x,tot_y,tot_z;
    float dx=5,dy=5,dz=5;
    Vector3f curr_pos =inertial_nav.get_position();

    Vector3f target;
    target.x+=dx;
    tot_x+=dx;
}