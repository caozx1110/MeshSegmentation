#pragma once
//面片
class Mesh
{
public:
    //四件套
    Mesh(int p1, int p2, int p3);
    Mesh(const Mesh& msh);
    Mesh& operator=(const Mesh& msh);
    ~Mesh();
    //定义Mesh的相等意味着两者共边
    bool operator==(const Mesh& msh);
private:
    //友元类
    friend class Image3D;
    //三个顶点对应PointList的下标
    int PointIdx[3];
};

