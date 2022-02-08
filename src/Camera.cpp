#include "Camera.h"
//Test22

//Test2
Camera::Camera(float p_n)
{
    h = 0;
    p = p_n;
    b = 0;

    x = 0;
    y = 0;
    z = 0;

    fov = M_PI / 6.0;  // 30 degree
	nearZ = 0.1;
	farZ = 5000.0;
    dist = 120.0;
}

void Camera::init() {
    init_projection();
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    
    init_rotate();
    
    float x_c, y_c, z_c;
    get_transformation(x_c, y_c, z_c);
    glTranslatef(x_c, y_c, z_c);
}

void Camera::init_projection()
{
	int wid, hei;
	double aspect;

	FsGetWindowSize(wid, hei);
	aspect = (double)wid / (double)hei;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fov * 180.0 / M_PI, aspect, nearZ, farZ);
}

void Camera::get_transformation(float &x_c, float &y_c, float &z_c)
{
    float vx, vy, vz;
    get_forward(vx, vy, vz);
    x_c = -(x - vx * dist);
    y_c = -(y - vy * dist);
    z_c = -(z - vz * dist);
}

void Camera::init_rotate()
{
    glRotatef(-b * 180.0 / M_PI, 0.0, 0.0, 1.0);
	glRotatef(-p * 180.0 / M_PI, 1.0, 0.0, 0.0);
	glRotatef(-h * 180.0 / M_PI, 0.0, 1.0, 0.0);
}

void Camera::get_forward(float& vx, float& vy, float& vz) const
{
	vx = -cos(p) * sin(h);
	vy = sin(p);
	vz = -cos(p) * cos(h);
}

void Camera::change_heading(float m_heading) {
    h += m_heading;
}

void Camera::change_pitch(float m_pitch) {
    p += m_pitch;
}

void Camera::change_dist(float m_dist) {
    float new_dist = dist * m_dist;
    if (MIN_DIST < new_dist && new_dist < MAX_DIST) {
        dist = new_dist;
    }
}

void Camera::follow(const UAV &uav) {
    float x_u, y_u, z_u;
    point_t uav_pos = uav.get_pos();
    x = uav_pos.x;
    y = uav_pos.y;
    z = uav_pos.z;
}

void Camera::get_position(float &x_g, float &y_g, float &z_g) const {
    x_g = x;
    y_g = y;
    z_g = z;
}

void Camera::get_angle(float &h_g, float &p_g, float &b_g) const {
    h_g = h;
    p_g = p;
    b_g = b;
}
