#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <random>
#include "Map.h"
#include "Point.h"

struct rotors_t {
    float r1;
    float r2;
    float r3;
    float r4;
};


class UAV{
private:
    rotors_t rotors;
    point_t loc;
    point_t vel;
    point_t acc;

    point_t angle; // angle.x : pitch, angle.y : yaw, angle.z : roll
    point_t ang_vel;
    point_t ang_acc;

    point_t init_point;
    
    point_t prev_measured_vel;
    point_t prev_measured_loc;

    std::vector<point_t> history;

    float radius;
    float mass;
    float dt;
    float size;

    int point_counts;
    
    std::vector<point_t> obstacles;

    void recalc_pos();
    void recalc_vel();

    void recalc_angle();
    void recalc_angular();

    void apply_drag();
    void apply_gravity();

public:
    UAV(float x_n, float y_n, float z_n, float mass_n, float dt_n, float size_n);
    float get_radius() const;
    point_t get_pos() const;
    point_t get_vel() const;
    point_t get_acc() const;
    point_t get_angle() const;
    point_t get_ang_vel() const;
    point_t get_init() const;
    point_t get_up() const;
    point_t get_forward() const;
    void apply_thrust();
    void apply_2d_force(rotors_t rotor_f);
    void apply_arbitrary_force(point_t force);
    void apply_force(rotors_t new_rotors);
    void change_heading(float angle);
    const std::vector<point_t> *get_history() const;
    void sense_obstacles(const Map &map);
    const std::vector<point_t> *get_obstacles() const;
    int target_inside_obstacle(const std::vector<block_t> *blocks, point_t target);
    
    point_t get_measured_pos();
    point_t get_measured_vel();
    point_t get_measured_acc() const;
};
