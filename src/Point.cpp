#include "Point.h"

Point::Point(double px, double py, double pz, int pr, int pg, int pb)
{
    x = px;
    y = py;
    z = pz;
    r = pr;
    g = pg;
    b = pb;
}

Point::Point(const Point& p)
{
    x = p.x;
    y = p.y;
    z = p.z;
    r = p.r;
    g = p.g;
    b = p.b;
}

Point& Point::operator=(const Point& p)
{
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;
    this->r = p.r;
    this->g = p.g;
    this->b = p.b;
    
    return *this;
}

Point::~Point()
{
}

void Point::SetColor(int pr, int pg, int pb)
{
    r = pr;
    g = pg;
    b = pb;
}
//计算两点距离
double Point::GetDisFrom(const Point p)
{
    return sqrt((this->x - p.x) * (this->x - p.x) + (this->y - p.y) * (this->y - p.y) + (this->z - p.z) * (this->z - p.z));
}
//减法
Point Point::operator-(const Point& p)
{
    double tempx = this->x - p.x;
    double tempy = this->y - p.y;
    double tempz = this->z - p.z;
    
    return Point(tempx, tempy, tempz);
}
//点乘
double Point::operator*(const Point& p)
{
    return x * p.x + y * p.y + z * p.z;
}
//叉乘
Point Point::operator^(const Point& p)
{
    return Point((y * p.z - z * p.y), (z * p.x - x * p.z), (x * p.y - y * p.x));
}
//数乘
Point Point::operator*(const double k)
{
    return Point(k * x, k * y, k * z);
}

double Point::Length()
{
    return sqrt(x * x + y * y + z * z);
}

