#include "Control.h"

Control::Control(float x, float y, float z) {}

void Control::init(const Map &map, const UAV &uav) {
    generate_path(map, uav);
    target = points.begin();
}

Control::~Control()
{
}

void Control::next_point() {
    if (next(target) != points.end()) {
        target++;
    } else {
        target = points.begin();
    }
}

void Control::run_fake_control(UAV &uav) {
    point_t dist = *target - uav.get_pos();
    float c = 1;
    uav.apply_arbitrary_force(c * dist);
}

void Control::stabilize_angle(UAV &uav) {
    point_t ang_diff = -1 * uav.get_angle();
    point_t ang_vel = uav.get_ang_vel();

    float pitch_diff = ang_diff.x; // r1, r3
    float roll_diff = ang_diff.z; // r2, r4

    float pitch_vel = ang_vel.x; // r1, r3
    float roll_vel = ang_vel.z; // r2, r4

    float c = 5;
    float d = 10;
    float r1 = c * pitch_diff - d * fabs(pitch_vel); 
    float r3 = c * -pitch_diff - d * fabs(pitch_vel); 

    float r2 = c * roll_diff - d * fabs(roll_vel); 
    float r4 = c * -roll_diff - d * fabs(roll_vel);
    uav.apply_force({r1, r2, r3, r4});
}

void Control::run_control(UAV &uav) {
    run_fake_control(uav);
    stabilize_angle(uav);
}

void Control::init_path() {
    for (float x = -1000; x < 1000; x++) {
        float y = 200 * sin(x / 100.) + 300;
        float z = 200 * cos(x / 100);
        points.push_back(point_t(x, y, z));
    }
}

void print_path(std::vector<point_t> &vec) {
    int i = 0;
    for (point_t &point : vec) {
        i++;
        printf("Point %d: { %.2f, %.2f }\n", i, point.x, point.z);
    }
}

void Control::generate_path(const Map &map, const UAV &uav)
{
    point_t init_point = uav.get_init();
    const std::vector<point_t> *boundary = map.get_boundary();
    const std::vector<block_t> *blocks = map.get_blocks();

    // Add points to temporary vector
    std::vector<point_t> search_points;

    point_t point_min = map.get_min_boundary();
    point_t point_max = map.get_max_boundary();

    float radius = uav.get_radius();

    for (float x = point_min.x; x < point_max.x; x += radius)
    {
        for (float y = point_min.y; y < point_max.y; y += radius)
        {
            if (map.is_inside_boundary(x, y) && !map.is_inside_block(x, y))
            {
                search_points.push_back(point_t(x, 0, y));
            }
        }
    }

    // Find closest point to initial point
    point_t closest = init_point;
    float min_dist = __FLT_MAX__;
    auto closest_index = search_points.begin();
    for (auto i = search_points.begin(); i != search_points.end(); ++i) {
        float dist = init_point.dist(*i);
        if (dist < min_dist) {
            min_dist = dist;
            closest = *i;
            closest_index = i;
        }
    }

    // Make path from points with greedy traveling salesman
    points.clear();
    point_t curr_point = closest;
    search_points.erase(closest_index);

    while (!search_points.empty()) {
        points.push_back(curr_point);
        float min_dist = __FLT_MAX__;
        auto min_point = search_points.begin();
        for (auto j = search_points.begin(); j != search_points.end(); ++j) {
            float dist = curr_point.dist(*j);
            if (dist < min_dist) {
                min_dist = dist;
                min_point = j;
            }
        }
        curr_point = *min_point;
        search_points.erase(min_point);
    }

    extern const int DISPLAY;
    if (DISPLAY == 0) {
        print_path(points);
    }
}

//test path for mapping
void Control::test_path() {
    for (float x = -500; x < 500; x++) {
        float y = 0;
        float z = 0;
        points.push_back(point_t(x, y, z));
    }
}

const std::vector<point_t> *Control::get_points() const{
    return &points;
}

point_t Control::get_target() const {
    return *target;
}
