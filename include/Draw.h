#pragma once
#define GL_SILENCE_DEPRECATION
#include "fssimplewindow.h"
#include <cmath>
#include <limits>
#include "DrawingUtilNG.h"
#include "GraphicFont.h"
#include "UAV.h"
#include "Map.h"
#include "Camera.h"
#include "Control.h"
#include "Point.h"
#include <algorithm>

typedef struct system
{
    Camera camera;
    UAV uav;
    Map map;
    Control control;
    bool need_blocks;
    bool control_on;
    bool user_define_mode;
    bool quad;
    std::vector<point_t> user_input;
} system_t;

namespace Draw
{
    void draw_uav(const UAV &uav);
    void draw_terrain(const system_t &sys);
    void draw_display(const system_t &sys, ComicSansFont &font, int wid, int hei);
    void draw_control(const Control &control, const UAV &uav);

    void takein_points(system_t &sys); // take in user input points, and draw
    void write_user_input_to_file(system_t &sys); // write user input vector to file "input.shape"
    
}; // namespace Draw

// UAV drawing
void draw_airblade(double cen_x, double cen_y, double cen_z, double height, double side_len);
void draw_body(double height, double side_len);

// Terrain drawing
void draw_blocks(const Map &map, float map_scale);
void draw_block(block_t block);
void draw_boundary_2D(const Map &map);
void draw_path_points_2D(const Control &control);
void draw_point_cloud(const UAV &uav, const Control &control);

// Display drawing
void draw_position_info(const Camera &camera, const UAV &uav, ComicSansFont &comicsans);
float draw_text_line(std::string title, std::string vars, float y_pos, ComicSansFont &font);
void draw_grid_lines(int grid_scale);
void draw_axes_lines(int axis_scale);
void draw_maprectangle(point_t, float scale);
void draw_mapbase(const Control &control, float output_scale);

// Control drawing
void draw_path(const std::vector<point_t> *points);
void draw_force(const UAV &uav);
void draw_scanner(const Camera &camera, double scanner_scale);

// Output Map Drawing
void draw_mapbase(const system_t &sys, float output_scale);

void draw_maprectangle(point_t point, float scale);

// What to do if I have to use both Control and Map?
void draw_flaggedpoints(const Control &control, const Map &map, const Camera &camera, float output_scale);
