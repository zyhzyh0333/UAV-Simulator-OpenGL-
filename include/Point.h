#pragma once
#include <cmath>

class point_t
{
public:
    point_t(float x_n, float y_n, float z_n);
    point_t();
    float x;
    float y;
    float z;
    float dist(point_t p2);
};

point_t operator*(point_t p, float c);
point_t operator*(float c, const point_t p);
point_t operator/(const point_t p, float c);
point_t operator+(const point_t p1, const point_t p2);
point_t operator-(const point_t p1, const point_t p2);