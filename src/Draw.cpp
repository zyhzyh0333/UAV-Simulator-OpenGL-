#include "Draw.h"
#include <iostream>

// declare a global variable to store user_input points
std::vector<point_t> user_input;
int mouse_click_count = 0;

//draw a small circle around the mouse position
void mouse_draw(int locX, int locY) {
    const float PI = 3.1415926;
    int wid, hei;
    FsGetWindowSize(wid, hei);

    // Set up 2D drawing
    glLineWidth(2);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0, (float)wid - 1, (float)hei - 1, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);

//    glBegin(GL_POINTS);

    glBegin(GL_LINE_LOOP);
    float rad = 5;
    for (int i = 0; i < 64; i++) {
        float angle = (float)i * PI/32.0;
        float x = float(locX) + cos(angle)*rad;
        float y = float(locY) + sin(angle)*rad;
        glVertex2f(x, y);
    }
    glEnd();


}

void connect_user_points(std::vector<point_t> &vec) {
    glLineWidth(3);
    glBegin(GL_LINE_STRIP);
    for (auto & point : vec) {
        glVertex2f(point.x, point.y);
    }
    glEnd();
    glLineWidth(1);
}

//// change the center point of user input boundary to (0,0)
//void adjust_points(std::vector<point_t> &vec, float scale = 1.0) {
//    float cen_x = 0.0, cen_y = 0.0;
//    int num_points = vec.size();
//    for (auto & point : vec) {
//        cen_x += point.x;
//        cen_y += point.y;
//    }
//    cen_x /= num_points;
//    cen_y /= num_points;
//    // translate to (0,0), then scale
//    for (auto & point : vec) {
//        point.x -= cen_x;
//        point.y -= cen_y;
//        
//        point.x *= scale;
//        point.y *= scale;
//    }
//}

void Draw::write_user_input_to_file(system_t &sys) {
    using namespace std;

    std::vector<float> tmp_x;
    std::vector<float> tmp_y;


    std::ofstream myfile;
    myfile.open("include/user_define.shape");
    if (!myfile.is_open()){
        printf("Error writing to file!\n");
    };
    for (auto & point : user_input) {

        myfile << point.x <<" "<<point.y<<endl;
        tmp_x.push_back(point.x);
        tmp_y.push_back(point.y);
    }
    if (tmp_x.size() > 1) {
        auto [x_min, x_max] = minmax_element(tmp_x.begin(), tmp_x.end());
        auto [y_min, y_max] = minmax_element(tmp_y.begin(), tmp_y.end());
        sys.map.set_minmax(*x_min, *y_min, *x_max, *y_max);
    }

    myfile.close();
}

void Draw::takein_points(system_t &sys) {
    int mouseEvent, leftButton, middleButton, rightButton;
    int locX, locY;
    point_t tmp_point;

    mouseEvent = FsGetMouseEvent(leftButton, middleButton,
        rightButton, locX, locY);
//    std::cout<<locX<<std::endl;
    mouse_draw(locX, locY);

    if (mouse_click_count != 0) {
        glBegin(GL_LINE_STRIP);
        glVertex2f(user_input.back().x, user_input.back().y);
        glVertex2f(locX, locY);
        glEnd();
    }

    if (mouseEvent == FSMOUSEEVENT_LBUTTONDOWN) {
        mouse_click_count++;
//        std::cout<<locX<<" "<<locY<<std::endl;
        tmp_point.x = (float) locX;
        tmp_point.y = (float) locY;
        user_input.push_back(tmp_point);

        // make the boundary's center (0,0)
//        adjust_points(user_input, 1.0);
    }
    connect_user_points(user_input);
}

void Draw::draw_uav(const UAV &uav) {
    glPushMatrix();

    double height = 10.0;
    double side_len = 20.0;
    double s_len = side_len/4.0;
    glColor3ub(1, 1, 1);
    point_t uav_pos = uav.get_pos();
    float x = uav_pos.x;
    float y = uav_pos.y;
    float z = uav_pos.z;

    glTranslatef(x, y, z);
    point_t uav_angle = uav.get_angle();
    glRotatef(uav_angle.x, 1, 0, 0);
    glRotatef(uav_angle.y, 0, 1, 0);
    glRotatef(uav_angle.z, 0, 0, 1);
    
    // draw four airblades and one body
    draw_airblade(-s_len, y, -s_len, height, side_len);
    draw_airblade(-s_len, y, s_len, height, side_len);
    draw_airblade(s_len, y, -s_len, height, side_len);
    draw_airblade(s_len, y, s_len, height, side_len);
    glColor3ub(127, 127, 127);
    draw_body(height, side_len);

    glPopMatrix();
}

void Draw::draw_terrain(const system_t &sys) {
    float map_scale, grid_scale, axis_scale;
    sys.map.get_scale(map_scale, grid_scale, axis_scale);
    // 3D drawing from here
    glColor3ub(0, 0, 255);
    draw_grid_lines(grid_scale);
    // draw axes (x is red, y is green, z is blue)
    draw_axes_lines(axis_scale);
    draw_boundary_2D(sys.map);
    if (sys.need_blocks) {
        draw_blocks(sys.map, map_scale);
    } else {
        draw_point_cloud(sys.uav, sys.control);
    }
}

void draw_point_cloud(const UAV &uav, const Control &control) {
    const std::vector<point_t> *obstacles = uav.get_obstacles();
    const point_t point = control.get_target();
    float r = 0.5;
    
//    point_t point = (*obstacles).back();
//    std::cout<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
//    DrawingUtilNG::drawCube(point.x - r, point.y - r, point.z - r, point.x + r, point.y + r, point.z + r);
    
  
    for (auto& point : *obstacles) {
//        std::cout<<point.x<<" "<<point.y<<" "<<point.z<<std::endl;
        DrawingUtilNG::drawCube(point.x - r, point.y - r, point.z - r, point.x + r, point.y + r, point.z + r);
    }
}

void Draw::draw_control(const Control &control, const UAV &uav) {
    draw_path(control.get_points());
    draw_force(uav);
}

void Draw::draw_display(const system_t &sys, ComicSansFont &font, int wid, int hei) {
    // Set up 2D drawing
    glLineWidth(1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0, (float)wid - 1, (float)hei - 1, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);

    draw_position_info(sys.camera, sys.uav, font);
    float output_scale = 1;
    draw_mapbase(sys, output_scale);
    //draw_scanner(camera, 0.5);
}

void draw_path(const std::vector<point_t> *points) {
    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < points->size(); i++) {
        glVertex3f(points->at(i).x, 0, points->at(i).z);
    }
    glEnd();
}

void draw_minimap(int map_size) {
    float half_map = map_size / 2.;
    glColor3f(1, 1, 1);
    glRectf(-half_map, -half_map, half_map, half_map);
    glColor3f(0, 0, 1);
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    glVertex2f(- half_map, - half_map);
    glVertex2f(half_map, - half_map);
    glVertex2f(half_map, half_map);
    glVertex2f(-half_map, half_map);
    glVertex2f(-half_map, -half_map);
    glEnd();
}

void draw_map_uav(float x, float y) {
    glColor3f(1, 0, 0);
    glPointSize(5);
    glBegin(GL_POINTS);
    glVertex2f(x, y);
    glEnd();
}

void draw_map_paths(const std::vector<point_t> *points, 
                    const std::vector<point_t> *boundary, 
                    const std::vector<point_t> *history,
                    float radius)
{
    glLineWidth(.1);

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < points->size(); i++) {
        point_t curr_point = points->at(i);
        glVertex2f(curr_point.x, curr_point.z);
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < boundary->size(); i++) {
        point_t curr_point = boundary->at(i);
        glVertex2f(curr_point.x, curr_point.y);
    }
    glEnd();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(.5, .5, 0, .025);
    float radianConvert = atan(1.) / 45.;
    for (int i = 0; i < history->size(); i++) {
        point_t curr_point = history->at(i);
        glBegin(GL_POLYGON);
        for (int i = 0; i < 360; i += 10) {
		    float angle = i * radianConvert;
		    float x = cos(angle) * radius + curr_point.x;
		    float y = sin(angle) * radius + curr_point.z;
		    glVertex2f(x, y);
        }
        glEnd();
	}
}

// mapbase is the output map background that indicate the positions of the flagged points.
void draw_mapbase(const system_t &sys, float output_scale) {
    const std::vector<point_t> *points = sys.control.get_points();
    const std::vector<point_t> *boundary = sys.map.get_boundary();
    const std::vector<point_t> *history = sys.uav.get_history();

    // Switch to minimap scale
    int wid, hei;
    FsGetWindowSize(wid, hei);
    int map_size = 200;
    glPushMatrix();
    glTranslatef(wid - map_size / 2, hei - map_size / 2, 0);
    draw_minimap(map_size);

    point_t point_min = sys.map.get_min_boundary();
    point_t point_max = sys.map.get_max_boundary();
    float buffer = 300;

    float diff_x = point_max.x - point_min.x + buffer;
    float diff_y = point_max.y - point_min.y + buffer;
    float center_x = (point_max.x + point_min.x) / 2.;
    float center_y = (point_max.y + point_min.y) / 2.;
    float scale = std::min(map_size / diff_x, map_size / diff_y);

    glScalef(scale, scale, 0);
    glTranslatef(- point_max.x / 2., - point_max.y / 2., 0);

    draw_map_paths(points, boundary, history, sys.uav.get_radius());
    point_t uav_pos = sys.uav.get_pos();
    float uav_x = uav_pos.x;
    float uav_y = uav_pos.y;
    float uav_z = uav_pos.z;
    draw_map_uav(uav_x, uav_z);
    glPopMatrix();
}


//Not too sure
void draw_flaggedpoints(const Control &control, const Map &map, const Camera &camera, float output_scale){

}

void draw_airblade(double cen_x, double cen_y, double cen_z, double height, double side_len) {
    double radius = side_len/4;
    double thickness = height/10;
    cen_y = 0;
    for (float j = 0.0; j < thickness; j += 0.01) {
        float x, y, angle;
        float radianConvert = atan(1.) / 45.;
        radius = fabs(radius);

        // adapt the number of segments based on radius size
        int stepSize = 1;
        if (radius < 10)
            stepSize = 3;
        else if (radius < 200)
            stepSize = round((3. - 1.) / (10. - 200.) * (radius - 200.) + 1.);

        stepSize *= 6;  // always want stepSize to be a factor of 360

        glBegin(GL_POLYGON);
        for (int i = 0; i < 360; i += stepSize) {
            angle = i * radianConvert;
            x = cos(angle) * radius + cen_x;
            y = sin(angle) * radius + cen_z;
            glVertex3d(x, cen_y + j,y);
        }
        // std::cout<<"X: "<<x<<" Y: "<<y<<std::endl;
        glEnd();
    }
}

float draw_text_line(std::string title, std::string vars, float y_pos, ComicSansFont &font) {
    font.setColorHSV(0, 1, 1);
    font.drawText(title, 10, y_pos, .25);
    y_pos += 20;
    font.setColorHSV(300, 1, .5);
    font.drawText(vars, 10, y_pos, .15);
    y_pos += 50;
    return y_pos;
}

void draw_position_info(const Camera &camera, const UAV &uav, ComicSansFont &font) {
    float y_pos = 60;
    std::string data;

    point_t uav_pos = uav.get_pos();
    float x = uav_pos.x;
    float y = uav_pos.y;
    float z = uav_pos.z;
    data = "X=" + std::to_string(x) + " Y=" + std::to_string(y) + " Z=" + std::to_string(z);
    y_pos = draw_text_line("Position", data, y_pos, font);

    float h_r, p_r, b_r;
    camera.get_angle(h_r, p_r, b_r);
    double h = h_r * 45. / atan(1.);
    double p = p_r * 45. / atan(1.);
    data = "h=" + std::to_string((int)h % 360) + " deg, p=" + std::to_string((int)p % 360) + " deg";
    y_pos = draw_text_line("Direction", data, y_pos, font);

    point_t uav_vel = uav.get_vel();
    data = "X=" + std::to_string(uav_vel.x) + " Y=" + std::to_string(uav_vel.y) + " Z=" + std::to_string(uav_vel.z);
    y_pos = draw_text_line("Velocity", data, y_pos, font);

    point_t uav_acc = uav.get_acc();
    data = "X=" + std::to_string(uav_acc.x) + " Y=" + std::to_string(uav_acc.y) + " Z=" + std::to_string(uav_acc.z);
    y_pos = draw_text_line("Acceleration", data, y_pos, font);
}

void draw_body(double height, double side_len) {
    double half_side = side_len/4.0;
    double h1 = height/20.0;
    double h2 = height/4.0;
    DrawingUtilNG::drawCube(- half_side, - h1, - half_side, half_side, - h2, half_side);
}

void draw_boundary_2D(const Map &map) {
    const std::vector<point_t> *boundary = map.get_boundary();
    glColor3i(255, 0, 0);
    
    glBegin(GL_LINE_LOOP);
    for (auto& currNode : *boundary) {
        glVertex3f(currNode.x, 0,currNode.y);
    }
    glEnd();
}

void draw_path_points_2D(const Control &control) {
    const std::vector<point_t> *pathPoints = control.get_points();
    // draw something, just show there's a path point
    glBegin(GL_LINE_STRIP);
    for (auto& curr: *pathPoints) {    
        // std::cout<<cur.x<<" "<<cur.y<<std::endl;
        // draw_body(cur.x, 0, cur.y, 10, 20);
        glVertex3f(curr.x, 0,curr.y);
    }
    glEnd();
}

void draw_blocks(const Map &map, float map_scale) {
    const std::vector<block_t> *blocks = map.get_blocks();
    for (int i = 0; i < blocks->size(); i++) {
        draw_block(blocks->at(i));
    }
}

void draw_block(block_t block) {
    float cen_x = block.cen_x;
    float cen_y = block.cen_y;
    float cen_z = block.cen_z;
    float height = block.height;
    float side_len = block.side_len;

    float x1 = cen_x - 0.5 * side_len;
    float x2 = cen_x + 0.5 * side_len;
    float z1 = cen_z - 0.5 * side_len;
    float z2 = cen_z + 0.5 * side_len;
    float y1 = cen_y;
    float y2 = cen_y + height;
    glColor3ub(255, 255, 255);
    DrawingUtilNG::drawCube(x1, y1, z1, x2, y2, z2);
    // black  lines
    glColor3ub(0, 0, 0);
    glBegin(GL_LINES);
    
    // bottom
    glVertex3d(x1, y1, z1);
    glVertex3d(x2, y1, z1);
    
    glVertex3d(x2, y1, z1);
    glVertex3d(x2, y1, z2);
    
    glVertex3d(x2, y1, z2);
    glVertex3d(x1, y1, z2);
    
    glVertex3d(x1, y1, z2);
    glVertex3d(x1, y1, z1);
    
    // top
    glVertex3d(x1, y2, z1);
    glVertex3d(x2, y2, z1);
    
    glVertex3d(x2, y2, z1);
    glVertex3d(x2, y2, z2);
    
    glVertex3d(x2, y2, z2);
    glVertex3d(x1, y2, z2);
    
    glVertex3d(x1, y2, z2);
    glVertex3d(x1, y2, z1);
    
    // side
    glVertex3d(x1, y1, z1);
    glVertex3d(x1, y2, z1);
    
    glVertex3d(x2, y1, z1);
    glVertex3d(x2, y2, z1);
    
    glVertex3d(x2, y2, z2);
    glVertex3d(x2, y1, z2);
    
    glVertex3d(x1, y1, z2);
    glVertex3d(x1, y2, z2);
    

    
    glEnd();   
}

void draw_grid_lines(int grid_scale) {
    glBegin(GL_LINES);
    int x;
    int line_length = 1500 * grid_scale;
    for (x = -line_length; x <= line_length; x += line_length/25)
    {
        glVertex3i(x, 0, -line_length);
        glVertex3i(x, 0, line_length);
        glVertex3i(-line_length, 0, x);
        glVertex3i(line_length, 0, x);
    }
    glEnd();
}

void draw_axes_lines(int axis_scale) {
    glLineWidth(8);
    glBegin(GL_LINES);
    int axis_length = 1500 * axis_scale;
    
    glColor3ub(255, 0, 0);
    glVertex3i(-axis_length, 0, 0);
    glVertex3i(axis_length, 0, 0);
    
    glColor3ub(0, 255, 0);
    glVertex3i(0, -axis_length, 0);
    glVertex3i(0, axis_length, 0);
    
    glColor3ub(0, 0, 255);
    glVertex3i(0, 0, -axis_length);
    glVertex3i(0, 0, axis_length);
    
    glEnd();
    glLineWidth(1);
}

// not used
void draw_scanner(const Camera &camera, double scanner_scale) {
    double rad = 30;
    double deg2red = M_PI/180.0;
    float x, y, z;
    float h, p, b;
    camera.get_position(x, y, z);
    camera.get_angle(h, p, b);
    double x1 = x-15; // make scanner middle
    double y1 = y;
    double z1 = z-20;
    double x2 = x1 - rad*sin(h);
    double y2 = y1 + rad*sin(p);
    double z2 = z1 + rad*cos(h);
    
    DrawingUtilNG::drawCube(x1, y1, z1, x2, y2, z2);
    glColor3ub(255, 0, 255);
    glLineWidth(8);
    glBegin(GL_LINES);
    glVertex3d(x1, y1, z1);
    glVertex3d(x2, y2, z2);
    glEnd();
}

void draw_force(const UAV &uav) {
    glLineWidth(5);
    glColor4f(1, 0, 0, 1);
    auto [x, y, z] = uav.get_pos();
    auto [x_f, y_f, z_f] = uav.get_pos() + 10 * uav.get_up();
    glBegin(GL_LINES);
    glVertex3f(x,y,z);
    glVertex3f(x_f,y_f,z_f);
    glEnd();
}
