#include "UAV.h"
#define CONV M_PI / 180

UAV::UAV(float x_n, float y_n, float z_n, float mass_n, float dt_n, float size_n) {
    loc = { x_n, y_n, z_n };
    history.push_back(loc);
    mass = mass_n;
    dt = dt_n;

    acc = point_t(0, 0, 0);
    vel = point_t(0, 0, 0);

    init_point = point_t(0, 0, 0);
    radius = 50;
    size = size_n;

    point_counts = 0;
    rotors = rotors_t { 0, 0, 0, 0 };
}

point_t UAV::get_forward() const {
    auto [x, y, z] = CONV * get_angle();
    auto c = [](auto a){return cosf(a);};
    auto s = [](auto a){return sinf(a);};

    float for_rot_x = s(y);
    float for_rot_y = -c(y)*s(x);
    float for_rot_z = c(x)*c(y);

    return (point_t){for_rot_x, for_rot_y, for_rot_z};
}

point_t UAV::get_up() const {
    auto [x, y, z] = CONV * get_angle();
    auto c = [](auto a){return cosf(a);};
    auto s = [](auto a){return sinf(a);};

    float rot_x = -c(y)*s(z);
    float rot_y = c(x)*c(z)-s(x)*s(y)*s(z);
    float rot_z = c(z)*s(x)+c(x)*s(y)*s(z);

    return (point_t){rot_x, rot_y, rot_z};
}

const std::vector<point_t> *UAV::get_history() const {
    return &history;
}

float UAV::get_radius() const {
    return radius;
}

point_t UAV::get_init() const {
    return init_point;
}

void UAV::recalc_angle()
{
    angle = angle + (ang_vel * dt);
}

void UAV::recalc_angular()
{
    ang_vel = ang_vel + (ang_acc * dt);
}

void UAV::apply_thrust() {
    float height = get_pos().y;
    float c = 100;
    point_t force;
    auto [rot_x, rot_y, rot_z] = get_up();
    force.x = c * rot_x * (10 - height);
    force.y = c * rot_y * (10 - height);
    force.z = c * rot_z * (10 - height);

    acc = force / mass;
}

void UAV::apply_2d_force(rotors_t rotor_f) {
    point_t torque;
    auto [forward, right, backward, left] = rotor_f;
    torque = {backward - forward, 0, left - right};
    ang_acc = torque / mass;
    // apply_arbitrary_force(get_forward());
    apply_thrust();
    recalc_angle();
    apply_drag();
    recalc_angular();
    apply_gravity();
    recalc_vel();
    recalc_pos();
}

void UAV::apply_arbitrary_force(point_t force) {
    acc = force / mass;
    recalc_angle();
    apply_drag();
    recalc_angular();
    apply_gravity();
    recalc_vel();
    recalc_pos();
}

void UAV::apply_force(rotors_t new_rotors) {
    rotors = new_rotors;

    point_t torque;

    float tip_const = .1;
    float yaw_const = .01;

    /*
     *    r1
     * r4    r2
     *    r3
     */

    torque.x = size * tip_const * (rotors.r1 - rotors.r3);
    torque.y = yaw_const * (rotors.r1 - rotors.r2 + rotors.r3 - rotors.r4);
    torque.z = size * tip_const * (rotors.r2 - rotors.r4);

    ang_acc = torque / mass; 


    float rotor_sum = rotors.r1 + rotors.r2 + rotors.r3 + rotors.r4;
    point_t force;

    extern const int DISPLAY;
    if (DISPLAY == 0) {
        printf("angles: { %.2f, %.2f, %.2f }\n", CONV*angle.x, CONV*angle.y, CONV*angle.z);
    }
    
    auto [rot_x, rot_y, rot_z] = get_up();
    force.x = rot_x * rotor_sum;
    force.y = rot_y * rotor_sum;
    force.z = rot_z * rotor_sum;

    acc = force / mass;

    recalc_angle();
    apply_drag();
    recalc_angular();
    apply_gravity();
    recalc_vel();
    recalc_pos();
    
    //point_t measured_acc = get_measured_acc();
    //point_t measured_vel = get_measured_vel();
    //point_t measured_pos = get_measured_pos();
}

point_t UAV::get_pos() const
{
    return loc;

}
point_t UAV::get_acc() const
{
    return acc;
}

point_t UAV::get_vel() const
{
    return vel;
}

point_t UAV::get_angle() const
{
    return angle;
}

point_t UAV::get_ang_vel() const
{
    return ang_vel;
}

void UAV::recalc_vel() {
    point_t new_vel = vel + (acc * dt);
    vel = new_vel;
}

void UAV::recalc_pos() {
    point_t new_loc = loc + (vel * dt);
    loc = new_loc;
    loc.y = loc.y > 0 ? loc.y : 0;
    history.push_back(loc);
}

void UAV::apply_drag() {
    float nu = .1;
    acc.x += vel.x < 0 ? (nu * pow(vel.x,2)) / mass : - (nu * pow(vel.x,2)) / mass;
    acc.y += vel.y < 0 ? (nu * pow(vel.y,2)) / mass : - (nu * pow(vel.y,2)) / mass;
    acc.z += vel.z < 0 ? (nu * pow(vel.z,2)) / mass : - (nu * pow(vel.z,2)) / mass;

    float r_nu = .05;
    ang_acc.x += ang_vel.x < 0 ? (r_nu * pow(ang_vel.x,2)) / mass : - (r_nu * pow(ang_vel.x,2)) / mass;
    ang_acc.y += ang_vel.y < 0 ? (r_nu * pow(ang_vel.y,2)) / mass : - (r_nu * pow(ang_vel.y,2)) / mass;
    ang_acc.z += ang_vel.z < 0 ? (r_nu * pow(ang_vel.z,2)) / mass : - (r_nu * pow(ang_vel.z,2)) / mass;
}

void UAV::apply_gravity() {
    acc.y -= loc.y > 0 ? 10 : 0;
}

void UAV::change_heading(float angle) {

}

const std::vector<point_t> *UAV::get_obstacles() const {
    return &obstacles;
}

void UAV::sense_obstacles(const Map &map) {
    float angle_resolution = M_PI / 90.0;
    float distance_resolution = 2.5;
    float sight = 80.0;
    float lat_max_angle = M_PI / 6.0;
    float lat_min_angle = -M_PI / 6.0;
    const std::vector<block_t>* blocks = map.get_blocks();
    point_t target;
    // obstacles.clear();
    for (auto long_angle = 0.0; long_angle < M_PI * 2.0; long_angle = long_angle + angle_resolution) {
        for (auto lat_angle = lat_min_angle; lat_angle < lat_max_angle; lat_angle = lat_angle + angle_resolution) {
            float u_x = cos(long_angle) * cos(lat_angle), u_y = sin(lat_angle), u_z = sin(long_angle) * cos(lat_angle);
            point_t unit_dist = point_t(u_x, u_y, u_z);
            for (float distance = 0.0; distance < sight; distance += distance_resolution) {
                target = loc + distance * unit_dist;
                //target.x = loc.x + distance * u_x;
                //target.y = loc.y + distance * u_y;
                //target.z = loc.z + distance * u_z;
                int block_index = target_inside_obstacle(blocks, target);
                if (block_index >= 0 && -1 == target_inside_obstacle(blocks, loc)) {
                    ++point_counts;
                    if (point_counts % 500 == 0)
                        obstacles.push_back(target - distance_resolution * unit_dist);
                    break;
                }
            }
        }
    }
}


int UAV::target_inside_obstacle(const std::vector<block_t>* blocks, point_t target)
{
    for (int i = 0; i < blocks->size(); i++) {
        block_t block = blocks->at(i);
        float dist_x, dist_y, dist_z;
        dist_x = target.x - block.cen_x;
        dist_y = target.y - block.cen_y;
        dist_z = target.z - block.cen_z;
        if (dist_y >= 0 && dist_y <= block.height && fabs(dist_x) <= .5 * block.side_len && fabs(dist_z) <= .5 * block.side_len)
            return i;
    }
    return -1;
}

point_t UAV::get_measured_pos()
{
    const double mean = 0, dev = 0.005; // mean and standard dev
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, dev);
    point_t measured_vel = get_measured_vel();
    point_t measured_loc = prev_measured_loc + (vel - measured_vel) * dt + point_t(dist(generator), dist(generator), dist(generator));
    prev_measured_loc = measured_loc;
    return measured_loc;
}

point_t UAV::get_measured_vel()
{
    const double mean = 0, dev = 0.001; // mean and standard dev
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, dev);
    point_t measured_acc = get_measured_acc();
    point_t measured_vel = prev_measured_vel + (acc - measured_acc) * dt + point_t(dist(generator), dist(generator), dist(generator));
    prev_measured_vel = measured_vel;
    return measured_vel;
}

point_t UAV::get_measured_acc() const
{
    const double mean = 0, dev = 0.0001; // mean and standard dev
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, dev);
    point_t measured_acc = acc + point_t(dist(generator), dist(generator), dist(generator));
    return measured_acc;
}
