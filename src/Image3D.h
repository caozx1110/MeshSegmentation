#pragma once
//pi
#define PI 3.1415926535898
//�Ƚ�double�������С��
//#define MIN 1e-7
//�Ǿ��밼�����
#define ETA 0.1
//��ؾ����Ȩ����
#define DELTA 0.6
//ģ���ָ���ֵ
#define EPSILON 0.1
//��������
#define ITERTIMES 10

#include "Point.h"
#include "Mesh.h"
#include <vector>
#include <queue>
#include <stack>
#include <string>
#include <utility>
#include <fstream>
#include <iostream>
#include <ctime>
using namespace std;

//���ȼ�������pair<int,double>�ıȽϷ�ʽ,second(Ȩֵ����С�ķ�����ǰ��
struct Node {
    int idx;
    double weight;
    friend bool operator < (Node A, Node B)
    {
        return  A.weight > B.weight;
    }
    Node(int i = 0, double w = 0) : idx(i), weight(w)
    {
    }
};
//ö���ͱ���
enum Color
{
    RED,        //��������A����ɫ1
    BLUE,       //��������B����ɫ2
    GREEN,
    UNCERTAIN   //��ȷ����ģ���ָ���
};

class Image3D
{
public:
    //�ļ���
    Image3D(string FileName);
    Image3D(const Image3D& Img);
    Image3D& operator=(const Image3D& Img);
    ~Image3D();
    //��obj�ļ��ж�ȡ��Ϣ
    bool LoadFromFile(string FileName);
    //����������Ƭ�Ĳ�ؾ���
    double Geod(const Mesh msh1, const Mesh msh2);
    //����������Ƭ�ĽǾ���
    double AngDist(const Mesh msh1, const Mesh msh2);
    //�����żͼ
    void FormDualGraph();
    //Dijkstra������±�Ϊbegin��Mesh��ʼ����end���������·
    void Dijkstra(int begin);
    //Dijkstra����ÿ����֮�����̾���
    void FormDistGraph();
    //�ҵ�������
    void IniREP();
    //Sum(PA(fi) * Dist(f, fi)),�������е㵽�õ�ļ�PAȨ�ľ���֮��
    double PADistSum(int idx);
    //Sum(PB(fi) * Dist(f, fi)),�������е㵽�õ�ļ�PBȨ�ľ���֮��
    double PBDistSum(int idx);
    //����PList
    void UpdatePList();
    //����REP
    void UpdateREP();
    //����ȷ�����Ӻ͸���
    void FormCluster();
    //ģ���ָ�
    void FuzzySeg();
    //���ɳ�ʼ����ͼ
    void FormCapGraph();
    //��������ͼ��BFS��ͨ·��Cap-������������
    bool UpdateCapGraph();
    //��ϸ�ָ�
    void ExactSeg();
    //д���ļ�
    void WriteIntoFile(string OldFileName);

private:
    //���������ɫ�洢����
    vector<Point> PointList;
    //��Ƭ�����������±�洢����
    vector<Mesh> MeshList;
    //��Ƭ��
    int MeshNum;
    //��żͼ���洢��Ӧ�±����Ƭ���ڵ�������Ƭ�±꼰Ȩֵ���ڽӱ���ʽ,pair<int, double>�����ڵ��±꼰��Ӧ��Ȩֵ
    vector<vector<Node> > DualGraph;
    //AngDist��,�Ǿ���(��ϸ�ָ�Ҫ�ã�
    vector<vector<double> > AngDistList;
    //�Ǿ���ƽ��ֵ����ϸ�ָ�Ҫ�ã�
    double AvgAngDist;
    //��żͼ������������������,DistGraph[i][j]��ʾ�±�Ϊi����Ƭ��j����̾���
    vector<vector<double> > DistGraph;
    //��̾�����Զ��һ����Ƭ���±�Ϊ��ʼ����
    pair<int, int> REP;
    //�ֱ���������ĸ���
    vector<pair<double, double> > PList;
    //����Ƭ�����ĸ�����
    vector<Color> ColorList;
    //����ͼ
    vector<vector<Node> > CapGraph;
    //��߽磬��Ӧ�±��Ƿ�Ϊ��߽�
    vector<int> RedBound;
    //���߽磬��Ӧ�±��Ƿ�Ϊ���߽�
    vector<int> BlueBound;
    //һ����߽磬�±�
    int ARedBound;
    //һ�����߽磬�±�
    int ABlueBound;
};

