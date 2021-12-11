#include "Mesh.h"
//四件套
Mesh::Mesh(int p1, int p2, int p3)
{
    PointIdx[0] = p1;
    PointIdx[1] = p2;
    PointIdx[2] = p3;
}

Mesh::Mesh(const Mesh& msh)
{
    PointIdx[0] = msh.PointIdx[0];
    PointIdx[1] = msh.PointIdx[1];
    PointIdx[2] = msh.PointIdx[2];
}

Mesh& Mesh::operator=(const Mesh& msh)
{
    this->PointIdx[0] = msh.PointIdx[0];
    this->PointIdx[1] = msh.PointIdx[1];
    this->PointIdx[2] = msh.PointIdx[2];

    return *this;
}

Mesh::~Mesh()
{
}
//定义两个面片相等意味着两者共边，也就是有两个点相同
bool Mesh::operator==(const Mesh& msh)
{
    int flag = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (this->PointIdx[i] == msh.PointIdx[j])
            {
                flag++;
            }
        }
    }
    if (flag == 2)
    {
        return true;
    }
    else
    {
        return false;
    }
}
