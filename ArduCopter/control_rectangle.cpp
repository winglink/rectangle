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

// newflightmode_init - initialise flight mode
bool Copter::rectangle_init(bool ignore_checks)
{

    printf("rectangle_init()\r\n");

    if(position_ok()||ignore_checks) {
        rectangle.init();
        rectangle.pos_point();//计算航迹点
        rectangle.useful_vector();
        return true;
    }
    else
        return false;

}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
void Copter::rectangle_run()
{
    /*
    if(!((Ii++)%)) {
        printf("it's in rectangle_run()\r\n");
        if(Ii>30000)
            Ii=0;
    }*/
    float target_yaw_rate = 0;
    //float target_climb_rate = 0;
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || ap.throttle_zero) {
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
      //  target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
    }
     rectangle.update();
     pos_control.update_z_controller();
     attitude_control.angle_ef_roll_pitch_yaw(rectangle.get_roll(),rectangle.get_pitch(),rectangle.get_yaw(),true);

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
void Copter::rectangle_nav() {
     if(control_mode!=CIRCLE){
         return;
     }
     rectangle.rec_nav();




}
