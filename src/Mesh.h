#pragma once
//��Ƭ
class Mesh
{
public:
    //�ļ���
    Mesh(int p1, int p2, int p3);
    Mesh(const Mesh& msh);
    Mesh& operator=(const Mesh& msh);
    ~Mesh();
    //����Mesh�������ζ�����߹���
    bool operator==(const Mesh& msh);
private:
    //��Ԫ��
    friend class Image3D;
    //���������ӦPointList���±�
    int PointIdx[3];
};

