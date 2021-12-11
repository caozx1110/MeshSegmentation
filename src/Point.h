#pragma once
#include <math.h>
class Point
{
public:
    Point(double px = 0, double py = 0, double pz = 0, int pr = 255, int pg = 255, int pb = 255);
    Point(const Point& p);
    Point& operator=(const Point& p);
    ~Point();
    void SetColor(int pr, int pg, int pb);
    //计算两点距离
    double GetDisFrom(const Point p);
    //减，向量也看成是一个以0为起始点的Point为终点的量
    Point operator-(const Point& p);
    //点乘
    double operator*(const Point& p);
    //叉乘
    Point operator^(const Point& p);
    //数乘
    Point operator*(const double k);
    //模长
    double Length();

private:
    friend class Image3D;
    double x;
    double y;
    double z;
    int r;
    int g;
    int b;
};

