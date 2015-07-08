//
// Created by wing on 15/6/10.
//

#ifndef ARDUPILOT_RECTANGLE_H
#define ARDUPILOT_RECTANGLE_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_PosControl.h>
#include <AC_AttitudeControl.h>
#include <stdio.h>

#define WPNAV_WP_TRACK_SPEED_MIN         50.0f
#define WPNAV_LEASH_LENGTH_MIN          100.0f
#define COPTER_DIST                     200.0f
class  Rectangle
{
public:
    Rectangle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control,const AC_AttitudeControl& attitude_control);
    void update();
    void init();
    void pos_point();
    void cal_target_point(float dt);
    float get_bearing_cd(const Vector3f &origin,const Vector3f &destination);
    float get_slow_down_speed(float dist_from_dest_cm,float accel_cmss);
    void set_wp_origin_and_destination(const Vector3f &origin,const Vector3f &destination);
    float get_roll() const {return _pos_control.get_roll();}
    float get_pitch() const {return _pos_control.get_pitch();}
    float get_yaw() const {return _yaw;}
    void  rec_nav();
    void set_wp_destination(const Vector3f& destination);
    void calculate_leash_length();
    void calc_slow_down_distance(float speed_cms,float accel_cms);
    void useful_vector();
    void set_pos_point(const Vector3f& curr,int Nd);
private:
    const AP_InertialNav&  _inav;
    const AP_AHRS&         _ahrs;
    AC_PosControl&         _pos_control;
    const AC_AttitudeControl& _attitude_control;

    int   No;
    Vector3f wp_point[4];

    Vector3f x_unit;
    Vector3f y_unit;
    Vector3f z_unit;
    int  status;
    float    x_length;   //wp[1]-wp[0]
    float    y_length;   //wp[2]-wp[1]
    float    z_length;   //wp[3]-wp[0]
    float    m_length;   //wp[2]-wp[3]
    float    total_length;   //横向距离统计
    float    lati_length;    //纵向距离

    Vector3f _origin;
    Vector3f _destination;
    Vector3f _pos_delta_unit;
    uint32_t _wp_last_update;
    float    _track_length;
    float    _yaw;
    float    _track_desired;
    float    _limited_speed_xy_cms;

    const  float  _wp_speed_cms=500;          // maximum horizontal speed in cm/s during missions
    const  float  _wp_accel_cms=100;
    const  float  _wp_speed_down_cms=150;
    const  float  _wp_speed_up_cms=250;
    const  float  _wp_accel_z_cms=100;


    // point variable
    float  _track_speed;
    float  _track_leash_length;
    float  _track_accel;  //  change ??
    float  _slow_down_dist;
    float  _wp_radius_cm=200.0f;
    struct rectangle_flags{
      uint8_t  reached_destination   :1;
      uint8_t  new_wp_destination    :1;
      uint8_t  recalc_wp_leash       :1;
    }_flags;

    enum flight_status{
       A_STATUS,
       B_STATUS,
       C_STATUS,
       D_STATUS
    };



};

#endif