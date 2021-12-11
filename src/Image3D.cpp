#include "Image3D.h"

//四件套
Image3D::Image3D(string FileName)
{
    if (LoadFromFile(FileName))
    {
        FormDualGraph();
        FormDistGraph();
        FormCluster();
        FuzzySeg();
        ExactSeg();
        WriteIntoFile(FileName);
    }
}

Image3D::Image3D(const Image3D& Img)
{
    PointList = Img.PointList;
    MeshList = Img.MeshList;
    DualGraph = Img.DualGraph;
}

Image3D& Image3D::operator=(const Image3D& Img)
{
    this->PointList = Img.PointList;
    this->MeshList = Img.MeshList;
    this->DualGraph = Img.DualGraph;

    return *this;
}

Image3D::~Image3D()
{
}

//从obj文件中读取点和面的信息
bool Image3D::LoadFromFile(string FileName)
{
    cout << "Loading info from " << FileName << ".obj ..." << endl;
    ifstream MyFile(FileName + ".obj", ios::in);
    //缓存begin
    string temp;
    double x;
    double y;
    double z;
    int p1;
    int p2;
    int p3;
    //缓存first
    if (!MyFile.is_open())
    {
        cout << "Fail to load info from " << FileName << ".obj ..." << endl;
        return false;
    }
    else
    {
        while (MyFile >> temp)
        { 
            if (temp == string("v"))
            {
                MyFile >> x >> y >> z;
                //存入点的坐标
                PointList.push_back(Point(x, y, z));
            }
            if (temp == string("f"))
            {
                MyFile >> p1 >> p2 >> p3;
                //点的下标减一， 使其与PointList的下标相对应
                MeshList.push_back(Mesh(p1 - 1, p2 - 1, p3 - 1));
            }
        }
    }
    MeshNum = MeshList.size();

    MyFile.close();
    cout << "Load info from " << FileName << ".obj successfully ..." << endl;
    return true;
}

//相邻面片的测地距离
double Image3D::Geod(const Mesh msh1, const Mesh msh2)
{
    ////在两面片相等时
    ////a1,a2为一个三角形非共边的边长
    //double a1;
    //double a2;
    ////b1,b2为另一个三角形非共边边长
    //double b1;
    //double b2;
    ////共边边长
    //double c;

    //m[0],[1]为一个三角形的共边的两端点的下标,m[2]为非共边顶点的下标，n为另一三角形的
    int m[3] = { 0 };
    int n[3] = { 0 };
    int flag = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (msh1.PointIdx[i] == msh2.PointIdx[j])
            {
                m[flag] = i;
                n[flag] = j;
                flag++;
            }
        }
    }
    //利用和为3
    m[2] = 3 - m[0] - m[1];
    n[2] = 3 - n[0] - n[1];
    ////各边边长，三角函数计算
    //c = PointList[msh1.PointIdx[m[0]]].GetDisFrom(PointList[msh1.PointIdx[m[1]]]);
    //a1 = PointList[msh1.PointIdx[m[2]]].GetDisFrom(PointList[msh1.PointIdx[m[0]]]);
    //a2 = PointList[msh1.PointIdx[m[2]]].GetDisFrom(PointList[msh1.PointIdx[m[1]]]);
    //b1 = PointList[msh2.PointIdx[n[2]]].GetDisFrom(PointList[msh2.PointIdx[n[0]]]);
    //b2 = PointList[msh2.PointIdx[n[2]]].GetDisFrom(PointList[msh2.PointIdx[n[1]]]);

    //double cosa = (a1 * a1 + c * c - a2 * a2) / (2 * a1 * c);
    //double sina = sqrt(1 - cosa * cosa);
    //double ha = a1 * sina;
    //double da = a1 * cosa;
    //double cosb = (b1 * b1 + c * c - b2 * b2) / (2 * b1 * c);
    //double sinb = sqrt(1 - cosb * cosb);
    //double hb = b1 * sinb;
    //double db = b1 * cosb;

    //向量计算
    Point c0 = PointList[msh2.PointIdx[n[0]]];
    Point c1 = PointList[msh1.PointIdx[m[1]]];
    Point a = PointList[msh1.PointIdx[m[2]]];
    Point b = PointList[msh2.PointIdx[n[2]]];
    double da = ((a - c0) * (c1 - c0)) / ((c1 - c0).Length());
    double db = ((b - c0) * (c1 - c0)) / ((c1 - c0).Length());
    double la = (a - c0).Length();
    double lb = (b - c0).Length();
    double ha = sqrt(la * la - da * da);
    double hb = sqrt(lb * lb - db * db);

    //计算得出非共边顶点的测地距离，除以3即为重心的测地距离（相似）
    double geod = sqrt((da - db) * (da - db) + (ha + hb) * (ha + hb)) / 3;

    return geod;
}
//角距离
double Image3D::AngDist(const Mesh msh1, const Mesh msh2)
{
    //m[0],[1]为一个三角形的共边的两端点的下标,m[2]为非共边顶点的下标，n为另一三角形的
    int m[3] = { 0 };
    int n[3] = { 0 };
    int flag = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (msh1.PointIdx[i] == msh2.PointIdx[j])
            {
                m[flag] = i;
                n[flag] = j;
                flag++;
            }
        }
    }
    //利用和为3
    m[2] = 3 - m[0] - m[1];
    n[2] = 3 - n[0] - n[1];

    //向外法向量
    Point NorVec1 = (PointList[msh1.PointIdx[1]] - PointList[msh1.PointIdx[0]]) ^ (PointList[msh1.PointIdx[2]] - PointList[msh1.PointIdx[1]]);
    Point NorVec2 = (PointList[msh2.PointIdx[1]] - PointList[msh2.PointIdx[0]]) ^ (PointList[msh2.PointIdx[2]] - PointList[msh2.PointIdx[1]]);
    //二面角
    double costheta = (NorVec1 * NorVec2) / (NorVec1.Length() * NorVec2.Length());

    //参数eta
    double eta;
    if (NorVec1 * (PointList[msh2.PointIdx[n[2]]] - PointList[msh2.PointIdx[n[0]]]) >= 0)
    {
        //凹面或平面
        eta = 1;
    }
    else
    {
        //凸面
        eta = ETA;
    }

    double angdist = eta * (1 - costheta);

    return angdist;
}

//构造对偶图
//待优化***************************************************************
void Image3D::FormDualGraph()
{
    cout << "Forming DualGraphy ..." << endl;
    DualGraph.resize(MeshNum);

    //蛮力算法,O(n^2)
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < MeshNum; j++)
        {
            if (MeshList[i] == MeshList[j])
            {
                DualGraph[i].push_back(Node(j, 0));
            }
        }
    }

    //测地距离
    vector<vector<double> > GeodList;
    GeodList.resize(MeshNum);
    //角距离
    //vector<vector<double> > AngDistList;
    AngDistList.resize(MeshNum);

    //构建测地距离及角距离的表
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            GeodList[i].push_back(Geod(MeshList[i], MeshList[DualGraph[i][j].idx]));
            AngDistList[i].push_back(AngDist(MeshList[i], MeshList[DualGraph[i][j].idx]));
        }
    }
    //平均值计算
    double AvgGeod = 0;
    AvgAngDist = 0;

    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            AvgGeod += GeodList[i][j];
            AvgAngDist += AngDistList[i][j];
        }
    }

    //平均值
    AvgGeod = AvgGeod / (MeshNum * 3.0);
    AvgAngDist = AvgAngDist / (MeshNum * 3.0);

    //给对偶表的权值赋值
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            DualGraph[i][j].weight = (DELTA * GeodList[i][j]) / AvgGeod + ((1 - DELTA) * AngDistList[i][j]) / AvgAngDist;
        }
    }

    cout << "Form DualGraph successfully ..." << endl;
}
//begin开始到各点的最短距离
//可通过对称性减少一半计算量
void Image3D::Dijkstra(int begin)
{
    vector<bool> visited(MeshNum, false);
    Node Top(begin, 0); //起点
    Node Neigh;

    priority_queue<Node> Que;
    Que.push(Top);           //把顶点放入优先级队列

    while (!Que.empty()) {
        Top = Que.top();
        Que.pop(); //提取优先级最高顶点
        
        DistGraph[begin][Top.idx] = Top.weight < DistGraph[begin][Top.idx] ? Top.weight : DistGraph[begin][Top.idx];

        if (visited[Top.idx])
        {
            continue;  // 若该顶点被访问过，则返回
        }
        visited[Top.idx] = true; // 设置该顶点访问标记
        for (int i = 0; i < 3; i++) //3为固定值
        {
            Neigh.idx = DualGraph[Top.idx][i].idx; // 取出第i个邻域顶点的秩
            Neigh.weight = DualGraph[Top.idx][i].weight + Top.weight; // 对应权重修改
            if (!visited[Neigh.idx])    // 若该邻域顶点未被访问，则放入队列
            {
                Que.push(Neigh);
            }
        }
    }
}
//计算每两点（Mesh）之间的最短距离
void Image3D::FormDistGraph()
{
    cout << "Forming DistGraph ..." << endl;

    //resize
    DistGraph.resize(MeshNum);
    for (int i = 0; i < MeshNum; i++)
    {
        DistGraph[i].resize(MeshNum);
        for (int j = 0; j < MeshNum; j++)
        {
            DistGraph[i][j] = INFINITY;
        }
    }
    //dijkstra
    for (int i = 0; i < MeshNum; i++)
    {
        Dijkstra(i);
    }

    cout << "Form DistGraph successfully ..." << endl;
}
//初始种子
void Image3D::IniREP()
{
    double MaxDist = 0;
    //距离最大的两个点
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = i + 1; j < MeshNum; j++)
        {
            if (DistGraph[i][j] > MaxDist)
            {
                MaxDist = DistGraph[i][j];
                REP.first = i;
                REP.second = j;
            }
        }
    }
}
//Sum(PA(fi) * Dist(f, fi)),其他所有点到该点的加PA权的距离之和
double Image3D::PADistSum(int idx)
{
    double Sum = 0;
    for (int i = 0; i < MeshNum; i++)
    {
        Sum += PList[i].first * DistGraph[i][idx];
    }
    return Sum;
}
//Sum(PB(fi) * Dist(f, fi)),其他所有点到该点的加PB权的距离之和
double Image3D::PBDistSum(int idx)
{
    double Sum = 0;
    for (int i = 0; i < MeshNum; i++)
    {
        Sum += PList[i].second * DistGraph[i][idx];
    }
    return Sum;
}
//更新PList
void Image3D::UpdatePList()
{
    for (int i = 0; i < MeshNum; i++)
    {
        PList[i].first = DistGraph[i][REP.second] / (DistGraph[i][REP.first] + DistGraph[i][REP.second]);
        PList[i].second = DistGraph[i][REP.first] / (DistGraph[i][REP.first] + DistGraph[i][REP.second]);
    }
}
//更新REP
void Image3D::UpdateREP()
{
    double MinPADist = INFINITY;
    double MinPBDist = INFINITY;
    for (int i = 0; i < MeshNum; i++)
    {
        double padistsum = PADistSum(i);
        double pbdistsum = PBDistSum(i);
        if (padistsum < MinPADist)
        {
            MinPADist = padistsum;
            REP.first = i;
        }
        if (pbdistsum < MinPBDist)
        {
            MinPBDist = pbdistsum;
            REP.second = i;
        }
    }
}
//迭代确定种子和概率
void Image3D::FormCluster()
{
    PList.resize(MeshNum);
    IniREP();
    pair<int, int> tempREP;

    cout << "Forming Cluster seed ..." << endl;

    for (int i = 0; i < ITERTIMES; i++)
    {
        tempREP = REP;
        //for debug
        //cout << REP.first << " " << REP.second << endl;
        UpdatePList();
        UpdateREP();
        //REP不变，停止迭代
        if (tempREP == REP)
        {
            break;
        }
    }
    cout << "Form Cluster seed successfully ..." << endl;
}
//模糊分割，找到边界
void Image3D::FuzzySeg()
{
    cout << "Fuzzy Segmentation starts ..." << endl;
    ColorList.resize(MeshNum);
    //模糊区分
    for (int i = 0; i < MeshNum; i++)
    {
        if (PList[i].first > 0.5 + EPSILON)
        {
            ColorList[i] = RED;
        }
        else if (PList[i].second > 0.5 + EPSILON)
        {
            ColorList[i] = BLUE;
        }
        else
        {
            ColorList[i] = UNCERTAIN;
        }
    }

    //找边界点
    RedBound.resize(MeshNum);
    BlueBound.resize(MeshNum);
    for (int i = 0; i < MeshNum; i++)
    {
        RedBound[i] = 0;
        BlueBound[i] = 0;
    }
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            //红
            if (ColorList[i] == RED && ColorList[DualGraph[i][j].idx] != RED)
            {
                RedBound[i]++;
                ARedBound = i;
            }
        }
        for (int j = 0; j < 3; j++)
        {
            //蓝
            if (ColorList[i] == BLUE && ColorList[DualGraph[i][j].idx] != BLUE)
            {
                BlueBound[i]++;
                ABlueBound = i;
            }
        }
    }
    cout << "Fuzzy Segmentation successfully ..." << endl;
}
//生成容量图
void Image3D::FormCapGraph()
{
    //resize
    CapGraph.resize(MeshNum);

    double tempCap;
    //构建
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            //假如有一边属于S或T
            if ((ColorList[i] != UNCERTAIN && !RedBound[i] && !BlueBound[i]) || (ColorList[DualGraph[i][j].idx] != UNCERTAIN) && !RedBound[DualGraph[i][j].idx] && !BlueBound[DualGraph[i][j].idx])
            {
                CapGraph[i].push_back(Node(DualGraph[i][j].idx, INFINITY));
            }
            //模糊区域内部
            else
            {
                tempCap = 1 / (1 + (AngDistList[i][j] / AvgAngDist));
                CapGraph[i].push_back(Node(DualGraph[i][j].idx, tempCap));
            }
        }
    }
}
//更新容量图
bool Image3D::UpdateCapGraph()
{
    //广度优先搜索队列
    queue<int> QueBFS;
    //是否visited
    vector<bool> isvisited(MeshNum, false);
    //前驱
    vector<int> Pre(MeshNum, -1);

    double MinCap = INFINITY;
    int Flag = 0;   //判断是否找到通路


    QueBFS.push(ARedBound);
    isvisited[ARedBound] = true;
    int Top;
    int temp;

    while (!QueBFS.empty())
    {
        Top = QueBFS.front();
        QueBFS.pop();

        ////前驱找最小容量
        //if (Pre[Top] != -1)
        //{
        //    int i;
        //    for (i = 0; i < 3; i++)
        //    {
        //        if (CapGraph[Pre[Top]][i].idx == Top)
        //        {
        //            break;
        //        }
        //    }
        //    MinCap = MinCap < CapGraph[Pre[Top]][i].weight ? MinCap : CapGraph[Pre[Top]][i].weight;
        //}

        //判断是否找到终点
        if (ColorList[Top] == BLUE)
        {
            Flag = 1;
            break;
        }
        //push
        for (int i = 0; i < 3; i++)
        {
            if (!isvisited[CapGraph[Top][i].idx] && CapGraph[Top][i].weight > 0)
            {
                QueBFS.push(CapGraph[Top][i].idx);
                //设置前驱
                Pre[CapGraph[Top][i].idx] = Top;
                //设置访问状态
                isvisited[CapGraph[Top][i].idx] = true;
            }
        }
    } 

    if (Flag)   //容量--
    {
        //找到最小cap
        temp = Top;
        while (Pre[temp] != -1)
        {
            int i;
            for (i = 0; i < 3; i++)
            {
                if (CapGraph[Pre[temp]][i].idx == temp)
                {
                    break;
                }
            }
            MinCap = MinCap < CapGraph[Pre[temp]][i].weight ? MinCap : CapGraph[Pre[temp]][i].weight;
            temp = Pre[temp];
        }
        //容量-MinCap
        temp = Top;
        while (Pre[temp] != -1)
        {
            int i;
            for (i = 0; i < 3; i++)
            {
                if (CapGraph[Pre[temp]][i].idx == temp)
                {
                    break;
                }
            }
            CapGraph[Pre[temp]][i].weight -= MinCap;
            temp = Pre[temp];
        }
        return true;
    }
    else
    {
        return false;
    }
}
//精细分割
void Image3D::ExactSeg()
{
    cout << "Exact Segmentation starts ..." << endl;
    //生成CapGraph
    FormCapGraph();
    while (UpdateCapGraph())
    {
    }
    
    stack<int> StaDFS;
    vector<bool> isvisited(MeshNum, false);

    //涂红
    StaDFS.push(ARedBound);
    isvisited[ARedBound] = true;
    //深度遍历
    while (!StaDFS.empty())
    {
        int Top = StaDFS.top();
        StaDFS.pop();
        
        for (int i = 0; i < 3; i++)
        {
            if (!isvisited[CapGraph[Top][i].idx] && CapGraph[Top][i].weight > 0)
            {
                StaDFS.push(CapGraph[Top][i].idx);
                isvisited[CapGraph[Top][i].idx] = true;
                if (ColorList[CapGraph[Top][i].idx] == UNCERTAIN)
                {
                    ColorList[CapGraph[Top][i].idx] = RED;
                }
            }
        }
    }

    //涂蓝
    for (int i = 0; i < MeshNum; i++)
    {
        isvisited[i] = false;
    }
    StaDFS.push(ABlueBound);
    isvisited[ABlueBound] = true;
    //深度遍历
    while (!StaDFS.empty())
    {
        int Top = StaDFS.top();
        StaDFS.pop();

        for (int i = 0; i < 3; i++)
        {
            if (!isvisited[CapGraph[Top][i].idx] && CapGraph[Top][i].weight > 0)
            {
                StaDFS.push(CapGraph[Top][i].idx);
                isvisited[CapGraph[Top][i].idx] = true;
                if (ColorList[CapGraph[Top][i].idx] == UNCERTAIN)
                {
                    ColorList[CapGraph[Top][i].idx] = BLUE;
                }
            }
        }
    }

    ////观看最小割
    //for (int i = 0; i < MeshNum; i++)
    //{
    //    for (int j = 0; j < 3; j++)
    //    {
    //        if (CapGraph[i][j].weight == 0)
    //        {
    //            ColorList[CapGraph[i][j].idx] = GREEN;
    //        }
    //    }
    //}
    cout << "Exact Segmentation successfully ..." << endl;
}

void Image3D::WriteIntoFile(string OldFileName)
{
    string NewFileName = OldFileName + string("_2Seg.obj");

    ofstream InFile(NewFileName);

    //设定点的颜色
    for (int i = 0; i < MeshNum; i++)
    {
        //红色
        if (ColorList[i] == RED)
        {
            for (int j = 0; j < 3; j++)
            {
                PointList[MeshList[i].PointIdx[j]].SetColor(255, 0, 0);
            }
        }
        //蓝色
        else if (ColorList[i] == BLUE)
        {
            for (int j = 0; j < 3; j++)
            {
                PointList[MeshList[i].PointIdx[j]].SetColor(0, 0, 255);
            }
        }
        ////观看最小割
        //else if (ColorList[i] == GREEN)
        //{
        //    for (int j = 0; j < 3; j++)
        //    {
        //        PointList[MeshList[i].PointIdx[j]].SetColor(0, 255, 0);
        //    }
        //}
    }

    //写入文件
    for (int i = 0; i < PointList.size(); i++)
    {
        InFile << "v " << PointList[i].x << " " << PointList[i].y << " " << PointList[i].z << " " << PointList[i].r << " " << PointList[i].g << " " << PointList[i].b << endl;
    }
    for (int i = 0; i < MeshNum; i++)
    {
        InFile << "f " << MeshList[i].PointIdx[0] + 1 << " " << MeshList[i].PointIdx[1] + 1 << " " << MeshList[i].PointIdx[2] + 1 << endl;
    }

    InFile.close();
    cout << "Write into " << NewFileName << " successfully !" << endl;
}
