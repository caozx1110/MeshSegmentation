#pragma once
//pi
#define PI 3.1415926535898
//比较double相等所用小量
//#define MIN 1e-7
//角距离凹面参数
#define ETA 0.1
//测地距离加权参数
#define DELTA 0.6
//模糊分割阈值
#define EPSILON 0.1
//迭代次数
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

//优先级队列中pair<int,double>的比较方式,second(权值）最小的放在最前面
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
//枚举型变量
enum Color
{
    RED,        //聚类中心A，颜色1
    BLUE,       //聚类中心B，颜色2
    GREEN,
    UNCERTAIN   //不确定，模糊分割用
};

class Image3D
{
public:
    //四件套
    Image3D(string FileName);
    Image3D(const Image3D& Img);
    Image3D& operator=(const Image3D& Img);
    ~Image3D();
    //从obj文件中读取信息
    bool LoadFromFile(string FileName);
    //计算相邻面片的测地距离
    double Geod(const Mesh msh1, const Mesh msh2);
    //计算相邻面片的角距离
    double AngDist(const Mesh msh1, const Mesh msh2);
    //构造对偶图
    void FormDualGraph();
    //Dijkstra计算从下标为begin的Mesh开始，到end结束的最短路
    void Dijkstra(int begin);
    //Dijkstra计算每两点之间的最短距离
    void FormDistGraph();
    //找到最大距离
    void IniREP();
    //Sum(PA(fi) * Dist(f, fi)),其他所有点到该点的加PA权的距离之和
    double PADistSum(int idx);
    //Sum(PB(fi) * Dist(f, fi)),其他所有点到该点的加PB权的距离之和
    double PBDistSum(int idx);
    //更新PList
    void UpdatePList();
    //更新REP
    void UpdateREP();
    //迭代确定种子和概率
    void FormCluster();
    //模糊分割
    void FuzzySeg();
    //生成初始容量图
    void FormCapGraph();
    //更新容量图，BFS找通路，Cap-允许的最大流量
    bool UpdateCapGraph();
    //精细分割
    void ExactSeg();
    //写入文件
    void WriteIntoFile(string OldFileName);

private:
    //点的坐标颜色存储向量
    vector<Point> PointList;
    //面片的三个顶点下标存储向量
    vector<Mesh> MeshList;
    //面片数
    int MeshNum;
    //对偶图，存储相应下标的面片相邻的三个面片下标及权值，邻接表形式,pair<int, double>存相邻的下标及相应的权值
    vector<vector<Node> > DualGraph;
    //AngDist表,角距离(精细分割要用）
    vector<vector<double> > AngDistList;
    //角距离平均值（精细分割要用）
    double AvgAngDist;
    //对偶图中任意两点的最近距离,DistGraph[i][j]表示下标为i的面片到j的最短距离
    vector<vector<double> > DistGraph;
    //最短距离最远的一对面片的下标为初始种子
    pair<int, int> REP;
    //分别属于两类的概率
    vector<pair<double, double> > PList;
    //各面片属于哪个聚类
    vector<Color> ColorList;
    //容量图
    vector<vector<Node> > CapGraph;
    //红边界，对应下标是否为红边界
    vector<int> RedBound;
    //蓝边界，对应下标是否为蓝边界
    vector<int> BlueBound;
    //一个红边界，下标
    int ARedBound;
    //一个蓝边界，下标
    int ABlueBound;
};

