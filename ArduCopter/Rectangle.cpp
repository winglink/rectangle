//
// Created by wing on 15/6/10.
//
#include <Rectangle.h>
#include <AP_HAL.h>
#include "Rectangle.h"

extern const AP_HAL::HAL& hal;

Rectangle::Rectangle(const AP_InertialNav& inav,const AP_AHRS& ahrs,AC_PosControl& pos_control,const AC_AttitudeControl& attitude_control ) :
    _yaw(0.0f),
      No(0),
    _wp_last_update(0),
    _slow_down_dist(0.0f),
    _track_length(0.0f),
    _limited_speed_xy_cms(0.0f),
    _track_desired(0.0f),
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control),
    _track_speed(0.0f),
    _track_accel(0.0f),
    _track_leash_length(0.0f)


{
    _flags.reached_destination=false;
    _flags.new_wp_destination=false;
    _flags.recalc_wp_leash=false;

}

void Rectangle::init(){
    _pos_control.init_xy_controller();
    _pos_control.set_speed_xy(_wp_speed_cms);
    _pos_control.set_accel_xy(_wp_accel_cms);
    _pos_control.set_speed_z(-_wp_speed_down_cms,_wp_speed_up_cms);
    _pos_control.set_accel_z(_wp_accel_z_cms);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

}

void Rectangle::update(){
        float dt=_pos_control.time_since_last_xy_update();
        if(dt>=_pos_control.get_dt_xy()){
            if(dt>=0.2f){
                dt=0.0f;
            }
        }
       cal_target_point(dt);
       if(_flags.new_wp_destination){
           _flags.new_wp_destination=false;
           _pos_control.freeze_ff_xy();
       }
       _pos_control.freeze_ff_z();
       _pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY,1.0f);
       _wp_last_update = hal.scheduler->millis();

    }
void Rectangle::pos_point() {
     Vector3f curr_pos = _inav.get_position();

             wp_point[0](curr_pos.x+1000,curr_pos.y,curr_pos.z);
             wp_point[1](curr_pos.x+1000,curr_pos.y+1000,curr_pos.z);
             wp_point[2](curr_pos.x,curr_pos.y+600,curr_pos.z);

              set_wp_destination(wp_point[0]);

}
void Rectangle::set_wp_destination(const Vector3f& destination){
    Vector3f  origin;
    if((hal.scheduler->millis()-_wp_last_update)<1000){
        origin=_pos_control.get_pos_target();
    }else {
          _pos_control.get_stopping_point_xy(origin);
          _pos_control.get_stopping_point_z(origin);
    }
    set_wp_origin_and_destination(origin,destination);
}
void Rectangle::set_wp_origin_and_destination(const Vector3f &origin,const Vector3f &destination){
           _origin=origin;
           _destination=destination;
      Vector3f pos_delta=_destination-_origin;
           _track_length=pos_delta.length();
      if (is_zero(_track_length)) {
        // avoid possible divide by zero
           _pos_delta_unit.x = 0;
           _pos_delta_unit.y = 0;
           _pos_delta_unit.z = 0;
      }else{
           _pos_delta_unit = pos_delta/_track_length;
      }
      calculate_leash_length();
      if(_track_length>=200){
           _yaw=get_bearing_cd(_origin,_destination);
      }
      else{
          _yaw=_attitude_control.angle_ef_targets().z;
      }
      _pos_control.set_pos_target(origin);
      _track_desired=0;
      _flags.reached_destination= false;
      _flags.new_wp_destination=true;

      const Vector3f &curr_vel=_inav.get_velocity();
      float speed_along_track=curr_vel.x*_pos_delta_unit.x+curr_vel.y*_pos_delta_unit.y+curr_vel.z*_pos_delta_unit.z;
      _limited_speed_xy_cms=constrain_float(speed_along_track,0,_wp_speed_cms);
}

float Rectangle::get_bearing_cd(const Vector3f &origin,const Vector3f &destination){
    float bearing=9000+atan2f(-(destination.x-origin.x),destination.y-origin.y)*5729.57795f;
    if(bearing<0){
        bearing+=36000;
    }
    return bearing;
}
float Rectangle::get_slow_down_speed(float dist_from_dest_cm,float accel_cmss){
          if(dist_from_dest_cm<=0)
              return WPNAV_WP_TRACK_SPEED_MIN;
          float target_speed=safe_sqrt(dist_from_dest_cm*4.0f*accel_cmss);
          if(target_speed<WPNAV_WP_TRACK_SPEED_MIN){
              return WPNAV_WP_TRACK_SPEED_MIN;
          }else{
              return target_speed;
          }
}
void Rectangle::calculate_leash_length() {
    float pos_delta_unit_xy=pythagorous2(_pos_delta_unit.x,_pos_delta_unit.y);
    float pos_delta_unit_z=fabsf(_pos_delta_unit.z);
    float speed_z;
    float leash_z;
    if(_pos_delta_unit.z>=0.0f){
        speed_z=_wp_speed_up_cms;
        leash_z=_pos_control.get_leash_up_z();
    }else{
        speed_z=_wp_speed_down_cms;
        leash_z=_pos_control.get_leash_down_z();
    }
    if(is_zero(pos_delta_unit_z)&&is_zero(pos_delta_unit_xy)){
         _track_accel=0;
         _track_speed=0;
         _track_leash_length=WPNAV_LEASH_LENGTH_MIN;
    }else if(is_zero(_pos_delta_unit.z)){
         _track_accel=_wp_accel_cms/pos_delta_unit_xy;
         _track_speed=_wp_speed_cms/pos_delta_unit_xy;
         _track_leash_length=_pos_control.get_leash_xy()/pos_delta_unit_xy;
    }else if(is_zero(pos_delta_unit_xy)){
        _track_accel=_wp_accel_z_cms/pos_delta_unit_z;
        _track_speed=speed_z/pos_delta_unit_z;
        _track_leash_length=leash_z/pos_delta_unit_z;
    }else {
        _track_accel=min(_wp_accel_z_cms/pos_delta_unit_z,_wp_accel_cms/pos_delta_unit_xy);
        _track_speed=min(speed_z/pos_delta_unit_z,_wp_speed_cms/pos_delta_unit_xy);
        _track_leash_length=min(leash_z/pos_delta_unit_z,_pos_control.get_leash_xy()/pos_delta_unit_xy);
    }
    calc_slow_down_distance(_track_speed,_track_accel);
    _flags.recalc_wp_leash=false;
}
void Rectangle::calc_slow_down_distance(float speed_cms, float accel_cms) {
       if(accel_cms<=0.0f){
           _slow_down_dist=0.0f;
           return;
       }
       _slow_down_dist=speed_cms*speed_cms/(4.0f*accel_cms);

}
void Rectangle::cal_target_point(float dt) {
    Vector3f  track_error;
    float     track_covered;  //track distance
    float     track_leash_slack;
    float     track_desired_max;
    bool      reach_leash_limit=false;
    Vector3f  curr_pos=_inav.get_position();
    Vector3f  curr_delta=curr_pos-_origin;
              track_covered=curr_delta.x*_pos_delta_unit.x+curr_delta.y*_pos_delta_unit.y+curr_delta.z*_pos_delta_unit.z;
    Vector3f  track_covered_pos=_pos_delta_unit*track_covered;
              track_error=curr_delta-track_covered_pos;  //目前位置误差
    float track_error_xy=pythagorous2(track_error.x,track_error.y);
    float track_error_z=fabsf(track_error.z);

    float leash_xy=_pos_control.get_leash_xy();  //printf()  ???  valve
    float leash_z;
          if(track_error.z>=0){
              leash_z=_pos_control.get_leash_up_z(); //printf()??
          }else{
              leash_z=_pos_control.get_leash_down_z(); //printf()??
          }
          track_leash_slack=min(_track_leash_length*(leash_z-track_error_z)/leash_z,_track_leash_length*(leash_xy-track_error_xy)/leash_xy);
          if(track_leash_slack<0){
               track_desired_max=track_covered;
          }else{
               track_desired_max=track_covered+track_leash_slack;
          }
          if(_track_desired>track_desired_max){
                reach_leash_limit=true;
          }
    const Vector3f &curr_vel=_inav.get_velocity();
    float speed_along_track=curr_vel.x*_pos_delta_unit.x+curr_vel.y*_pos_delta_unit.y+curr_pos.z*_pos_delta_unit.z;
    float linear_velocity=_wp_speed_cms;
    float kp=_pos_control.get_pos_xy_kP();//printf();
          if(kp>=0.0f){
              linear_velocity=_track_accel/kp;
          }
          if(speed_along_track<-linear_velocity){
              _limited_speed_xy_cms=0;
          }
          else {
                  if(dt>0&&!reach_leash_limit)
              {
                  _limited_speed_xy_cms+=2*_track_accel*dt; //change _track_accel
              }
              _limited_speed_xy_cms=constrain_float(_limited_speed_xy_cms,0.0f,_track_speed);

              float dis_to_dest=_track_length-_track_desired;
              if(dis_to_dest<=_slow_down_dist)
                  _limited_speed_xy_cms=min(_limited_speed_xy_cms,get_slow_down_speed(dis_to_dest,_track_accel));  //chage _track_accel
              if(fabs(speed_along_track)<linear_velocity) {
                  _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, speed_along_track - linear_velocity,
                                                          speed_along_track + linear_velocity);
              }
          }

          if(!reach_leash_limit) {
              _track_desired += _limited_speed_xy_cms * dt;

              if (_track_desired > track_desired_max) {
                  _track_desired = track_desired_max;
                  _limited_speed_xy_cms -= 2.0f * _track_accel * dt;
                  if (_limited_speed_xy_cms < 0.0f) {
                      _limited_speed_xy_cms = 0.0f;
                  }
              }
          }

         _track_desired=constrain_float(_track_desired,0,_track_length);

         _pos_control.set_pos_target(_origin+_pos_delta_unit*_track_desired);

         if(!_flags.reached_destination){
            if(_track_desired>=_track_length){
                Vector3f dist_to_dest=curr_pos-_destination;
                if(dist_to_dest.length()<=_wp_radius_cm){
                    _flags.reached_destination=true;
                }
        }
    }
}

void Rectangle::rec_nav() {

         if(_flags.reached_destination==true){
             /*
            set_wp_origin_and_destination(_pos_control.get_pos_target(),wp_point[No++]);
            if(No==3){
                No=0;
            }
              */
              printf("reach destination\r\n");
              set_wp_destination(wp_point[1]);
             printf("second destination\r\n");
         }

}




