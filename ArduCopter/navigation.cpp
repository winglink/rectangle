// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
    rectangle_nav();
}

// calc_position - get lat and lon positions from inertial nav library
void Copter::calc_position()
{
    // pull position from interial nav library
    current_loc.lng = inertial_nav.get_longitude();
    current_loc.lat = inertial_nav.get_latitude();
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
void Copter::calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_distance()
{
    // get target from loiter or wpinav controller
    if (control_mode == LOITER || control_mode == CIRCLE) {
        wp_distance = wp_nav.get_loiter_distance_to_target();
    }else if (control_mode == AUTO || control_mode == RTL || (control_mode == GUIDED && guided_mode == Guided_WP)) {
        wp_distance = wp_nav.get_wp_distance_to_destination();
    }else{
        wp_distance = 0;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    if (control_mode == LOITER || control_mode == CIRCLE) {
        wp_bearing = wp_nav.get_loiter_bearing_to_target();
    } else if (control_mode == AUTO || control_mode == RTL || (control_mode == GUIDED && guided_mode == Guided_WP)) {
        wp_bearing = wp_nav.get_wp_bearing_to_destination();
    } else {
        wp_bearing = 0;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
void Copter::calc_home_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();

    // calculate home distance and bearing
    if (position_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        Vector3f home = pv_location_to_vector(ahrs.get_home());
        home_bearing = pv_get_bearing_cd(curr,home);

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
void Copter::run_autopilot()
{
    if (control_mode == AUTO) {
        // update state of mission
        // may call commands_process.pde's start_command and verify_command functions
        mission.update();
    }
}
