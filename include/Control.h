#pragma once
#include <vector>
#include "UAV.h"
#include "Point.h"
#include "Map.h"

class Control
{
private:
    std::vector<point_t> points; 
    std::vector<point_t>::iterator target;
    void init_path();
    void run_fake_control(UAV &uav);

public:
    Control(float x, float y, float z);
    ~Control();
    void generate_path(const Map &map, const UAV &uav);
    void init(const Map &map, const UAV &uav);
    const std::vector<point_t> *get_points() const;
    void run_control(UAV &uav);
    void next_point();
    point_t get_target() const;
    void test_path();
    void stabilize_angle(UAV &uav);
};