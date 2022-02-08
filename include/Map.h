#pragma once
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>

#include "Point.h"

typedef struct block {
    float cen_x;
    float cen_y;
    float cen_z;
    float height;
    float side_len;
} block_t ;

class Map {
private:
    float map_scale;
    float grid_scale;
    float axis_scale;
    int size;
    std::vector<block_t> blocks;
    
    std::vector<point_t> boundary_shape;
    std::vector<point_t> terrain;
    float b_xmin, b_ymin, b_xmax, b_ymax; // min and max of boundary
    void init_terrain();

public:
    void init_blocks();
    void load_map_from_file(const std::string &filename);
    void set_minmax(float b_xmin_n, float b_ymin_n, float b_xmax_n, float b_ymax_n);
    void set_boundary_shape(std::vector<point_t> new_shape);
    bool is_inside_block(float testx, float testy) const;
    bool is_inside_boundary(float testx, float testy) const;
    void get_scale(float &map_scale, float &grid_scale, float &axis_scale) const;
    point_t get_min_boundary() const;
    point_t get_max_boundary() const;
    void add_block(block_t block);
    const std::vector<block_t> *get_blocks() const;
    const std::vector<point_t> *get_boundary() const;
    Map(float map_s, float grid_s, float axis_s, std::string filename);
};
