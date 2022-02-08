#include "Point.h"

point_t::point_t(float x_n, float y_n, float z_n)
{
    x = x_n;
    y = y_n;
    z = z_n;
}

point_t::point_t()
{
    x = 0;
    y = 0;
    z = 0;
}

float point_t::dist(point_t p2)
{
    return pow(pow(p2.x - x, 2) + pow(p2.y - y, 2) + pow(p2.z - z, 2), .5);
}

point_t operator*(const point_t p, float c)
{
    point_t p_new;
    p_new.x = p.x * c;
    p_new.y = p.y * c;
    p_new.z = p.z * c;
    return p_new;
}

point_t operator*(float c, const point_t p)
{
    point_t p_new;
    p_new.x = p.x * c;
    p_new.y = p.y * c;
    p_new.z = p.z * c;
    return p_new;
}

point_t operator/(const point_t p, float c)
{
    return point_t(p.x / c, p.y / c, p.z / c);
}

point_t operator+(const point_t p1, const point_t p2)
{
    return point_t(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

point_t operator-(const point_t p1, const point_t p2)
{
    point_t p_new;
    p_new.x = p1.x - p2.x;
    p_new.y = p1.y - p2.y;
    p_new.z = p1.z - p2.z;
    return p_new;
}