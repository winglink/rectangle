//
// Created by wing on 15/6/10.
//
#include <Rectangle.h>
#include <AP_Math.h>
#include "Rectangle.h"
#include "../libraries/AC_AttitudeControl/AC_PosControl.h"

Rectangle::Rectangle(const AP_InertialNav& inav,const AP_AHRS& ahrs,AC_PosControl& pos_control) :
       dx(5.0f);
       dy(5.0f);
       dz(5.0f);
    tot_x(0.0f);
    tot_y(0.0f);
    tot_z(0.0f);
{}

void Rectangle::init(){
    _pos_control.init_xy_controller();
    _pos_control.set_speed_xy(300);
    _pos_control.set_accel_xy(60);
    _pos_control.set_speed_z(250, 150);
    _pos_control.set_accel_z(100);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

}

void Rectangle::update(){
        float dt=pos_control.time_since_last_xy_update();
        Vector3f curr_pos =inertial_nav.get_position();
        Vector3f target;
        target.x=curr_pos.x+dx;
        tot_x+=dx;
       _pos_control.set_pos_target(target);
       _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY,1.0f);

    }





