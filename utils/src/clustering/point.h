#ifndef __POINT__
#define __POINT__

struct Point
{
    float X;
    float Y;

    Point(const float &X, const float &Y);
    float distance(const Point &other);
};

#endif