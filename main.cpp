#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>

#define GL_SILENCE_DEPRECATION
#include "fssimplewindow.h"
#include "GraphicFont.h"
#include "DrawingUtilNG.h"

#include "Camera.h"
#include "UAV.h"
#include "Map.h"
#include "Control.h"
#include "Draw.h"
#include "Point.h"

// DISPLAY 1 opens display window,
// DISPLAY 0 runs system without window
extern const int DISPLAY = 1;
#define LEN 10

system_t sys_init();
int run_system(system_t sys);
bool process_input(system_t &sys);
int display_frame(system_t &sys, ComicSansFont &font);

system_t sys_init()
{
    float map_scale = 1;
    float grid_scale = 1;
    float axis_scale = 1;
    UAV uav = UAV(0, 0, 0, 10, 1 / 10., 4);
    Camera camera = Camera(-.35);
    Map map = Map(map_scale, grid_scale, axis_scale, "include/Elephant.shape");
    Control control = Control(0, 0, 0);
    system_t sys = (system_t){camera, uav, map, control, true, false, false, true};
    return sys;
}

void process_2d_input(system_t &sys)
{
    float forward = 0;
    float right = 0;
    float backward = 0;
    float left = 0;
    float force = 100;
    if (FsGetKeyState(FSKEY_W))
    {
        forward += force;
    }
    if (FsGetKeyState(FSKEY_D))
    {
        right += force;
    }
    if (FsGetKeyState(FSKEY_S))
    {
        backward += force;
    }
    if (FsGetKeyState(FSKEY_A))
    {
        left += force;
    }
    sys.uav.apply_2d_force((rotors_t){forward, right, backward, left});
    sys.control.stabilize_angle(sys.uav);
}

void process_quad_input(system_t &sys)
{
    float r1 = 0;
    float r2 = 0;
    float r3 = 0;
    float r4 = 0;
    float f = 100;
    if (FsGetKeyState(FSKEY_Q))
    {
        r1 += f;
    }
    if (FsGetKeyState(FSKEY_W))
    {
        r2 += f;
    }
    if (FsGetKeyState(FSKEY_A))
    {
        r3 += f;
    }
    if (FsGetKeyState(FSKEY_S))
    {
        r4 += f;
    }
    if (FsGetKeyState(FSKEY_SPACE))
    {
        r1 += f;
        r2 += f;
        r3 += f;
        r4 += f;
    }
    sys.uav.apply_force({r1, r2, r3, r4});
}

bool process_input(system_t &sys)
{
    FsPollDevice();

    int key = FsInkey();
    switch (key)
    {
    case FSKEY_ESC:
        return false;
        break;
    case FSKEY_T:
        sys.need_blocks = !sys.need_blocks;
        break;
    case FSKEY_C:
        sys.control_on = !sys.control_on;
        break;
    case FSKEY_M:
        sys.user_define_mode = !sys.user_define_mode;
        break;
    case FSKEY_P:
        sys.quad = !sys.quad;
        break;
    case FSKEY_ENTER: // reload the map, quit user define mode
        sys.user_define_mode = !sys.user_define_mode;
        sys.map.load_map_from_file("include/user_define.shape");
        sys.map.init_blocks();
        sys.control.generate_path(sys.map, sys.uav);
        break;
    }

    float x_v, y_v, z_v;
    sys.camera.get_forward(x_v, y_v, z_v);

    if (sys.quad) {
        process_quad_input(sys);
    } else {
        process_2d_input(sys);
    }

    // Camera operations
    if (FsGetKeyState(FSKEY_LEFT))
    {
        sys.camera.change_heading(M_PI / 180);
        sys.uav.change_heading(M_PI / 180);
    }
    if (FsGetKeyState(FSKEY_RIGHT))
    {
        sys.camera.change_heading(-M_PI / 180);
        sys.uav.change_heading(-M_PI / 180);
    }
    if (FsGetKeyState(FSKEY_UP))
    {
        sys.camera.change_pitch(-M_PI / 180);
    }
    if (FsGetKeyState(FSKEY_DOWN))
    {
        sys.camera.change_pitch(M_PI / 180);
    }
    if (FsGetKeyState(FSKEY_G))
    {
        sys.camera.change_dist(1 / 1.05);
    }
    if (FsGetKeyState(FSKEY_N))
    {
        sys.camera.change_dist(1.05);
    }

    return true;
}

int run_system(system_t sys)
{
    bool cont = true;
    sys.control.init(sys.map, sys.uav);
    ComicSansFont font = ComicSansFont();
    if (DISPLAY == 1)
    {
        font.init();
    }

    int i = 0;
    while (cont)
    {
        if (DISPLAY == 0 && i > LEN)
        {
            cont = false;
        }

        if (sys.control_on)
        {
            sys.control.run_control(sys.uav);
        }
        sys.uav.sense_obstacles(sys.map); // start sensing obstacles

        if (sys.uav.get_pos().dist(sys.control.get_target()) < sys.uav.get_radius())
        {
            sys.control.next_point();
        }

        if (DISPLAY == 1)
        {
            sys.camera.init();
            sys.camera.follow(sys.uav);
            cont = process_input(sys);
            display_frame(sys, font);
            // FsSleep(10);
        }
        else
        {
            point_t p_t = sys.control.get_target();
            point_t uav_pos = sys.uav.get_pos();
            printf("\nIteration %d\n", i);
            printf("Target {x: %.2f, y: %.2f, z: %.2f}\n", p_t.x, p_t.y, p_t.z);
            printf("UAV {x: %.2f, y: %.2f, z: %.2f}\n", uav_pos.x, uav_pos.y, uav_pos.z);
        }

        i++;
    }
    return 0;
}

int display_frame(system_t &sys, ComicSansFont &font)
{
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    int wid, hei;
    FsGetWindowSize(wid, hei);
    glViewport(0, 0, wid, hei);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1, 1);

    // Draw::draw_uav(sys.uav);
    // Draw::draw_terrain(sys);
    // Draw::draw_control(sys.control, sys.uav);
    // Draw::draw_display(sys, font, wid, hei);

    if (sys.user_define_mode == false)
    {
        Draw::draw_uav(sys.uav);
        Draw::draw_terrain(sys);
        Draw::draw_control(sys.control, sys.uav);
        Draw::draw_display(sys, font, wid, hei);
    }
    else
    {
        Draw::takein_points(sys);
        // this will open/close the file hundreds of times, but it works...
        Draw::write_user_input_to_file(sys);
    }

    FsSwapBuffers();
    return 0;
}

int main(int argc, char *argv[])
{
    if (DISPLAY == 1)
    {
        FsOpenWindow(16, 16, 800, 600, 1);
    }

    system_t sys = sys_init();
    run_system(sys);
    return 0;
}
