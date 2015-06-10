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


class  Rectangle
{
public:
    Rectangle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control);
    void update();
    void init();
private:
    const AP_InertialNav&  _inav;
    const AP_AHRS&         _ahrs;
    AC_PosControl&         _pos_control;


    float tot_x,tot_y,tot_z;
    float dx,dy,dz;


};

#endif