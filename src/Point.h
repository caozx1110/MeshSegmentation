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
    //�����������
    double GetDisFrom(const Point p);
    //��������Ҳ������һ����0Ϊ��ʼ���PointΪ�յ����
    Point operator-(const Point& p);
    //���
    double operator*(const Point& p);
    //���
    Point operator^(const Point& p);
    //����
    Point operator*(const double k);
    //ģ��
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

