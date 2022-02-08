#pragma once
#include <cmath>
#include "fssimplewindow.h"
#include "UAV.h"

#define MIN_DIST .5
#define MAX_DIST 4000

class Camera
{
private:
	float x, y, z;  // target (what the camera is looking at)
	float h, p, b;  // camera orientation (heading, pitch, bank)
    float dist;
	float fov, nearZ, farZ;

    void get_transformation(float &x_c, float &y_c, float &z_c);
    void init_projection();
    void init_rotate();

public:
	Camera(float p_n);
    void init();
    void follow(const UAV &uav);
	void get_forward(float& vx, float& vy, float& vz) const;

    void change_heading(float m_heading);
    void change_pitch(float m_pitch);
    void change_dist(float m_dist);

    void get_position(float &x, float &y, float &z) const;
    void get_angle(float &h, float &p, float &b) const;
};