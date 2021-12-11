#include "Image3D.h"

//�ļ���
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

//��obj�ļ��ж�ȡ��������Ϣ
bool Image3D::LoadFromFile(string FileName)
{
    cout << "Loading info from " << FileName << ".obj ..." << endl;
    ifstream MyFile(FileName + ".obj", ios::in);
    //����begin
    string temp;
    double x;
    double y;
    double z;
    int p1;
    int p2;
    int p3;
    //����first
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
                //����������
                PointList.push_back(Point(x, y, z));
            }
            if (temp == string("f"))
            {
                MyFile >> p1 >> p2 >> p3;
                //����±��һ�� ʹ����PointList���±����Ӧ
                MeshList.push_back(Mesh(p1 - 1, p2 - 1, p3 - 1));
            }
        }
    }
    MeshNum = MeshList.size();

    MyFile.close();
    cout << "Load info from " << FileName << ".obj successfully ..." << endl;
    return true;
}

//������Ƭ�Ĳ�ؾ���
double Image3D::Geod(const Mesh msh1, const Mesh msh2)
{
    ////������Ƭ���ʱ
    ////a1,a2Ϊһ�������ηǹ��ߵı߳�
    //double a1;
    //double a2;
    ////b1,b2Ϊ��һ�������ηǹ��߱߳�
    //double b1;
    //double b2;
    ////���߱߳�
    //double c;

    //m[0],[1]Ϊһ�������εĹ��ߵ����˵���±�,m[2]Ϊ�ǹ��߶�����±꣬nΪ��һ�����ε�
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
    //���ú�Ϊ3
    m[2] = 3 - m[0] - m[1];
    n[2] = 3 - n[0] - n[1];
    ////���߱߳������Ǻ�������
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

    //��������
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

    //����ó��ǹ��߶���Ĳ�ؾ��룬����3��Ϊ���ĵĲ�ؾ��루���ƣ�
    double geod = sqrt((da - db) * (da - db) + (ha + hb) * (ha + hb)) / 3;

    return geod;
}
//�Ǿ���
double Image3D::AngDist(const Mesh msh1, const Mesh msh2)
{
    //m[0],[1]Ϊһ�������εĹ��ߵ����˵���±�,m[2]Ϊ�ǹ��߶�����±꣬nΪ��һ�����ε�
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
    //���ú�Ϊ3
    m[2] = 3 - m[0] - m[1];
    n[2] = 3 - n[0] - n[1];

    //���ⷨ����
    Point NorVec1 = (PointList[msh1.PointIdx[1]] - PointList[msh1.PointIdx[0]]) ^ (PointList[msh1.PointIdx[2]] - PointList[msh1.PointIdx[1]]);
    Point NorVec2 = (PointList[msh2.PointIdx[1]] - PointList[msh2.PointIdx[0]]) ^ (PointList[msh2.PointIdx[2]] - PointList[msh2.PointIdx[1]]);
    //�����
    double costheta = (NorVec1 * NorVec2) / (NorVec1.Length() * NorVec2.Length());

    //����eta
    double eta;
    if (NorVec1 * (PointList[msh2.PointIdx[n[2]]] - PointList[msh2.PointIdx[n[0]]]) >= 0)
    {
        //�����ƽ��
        eta = 1;
    }
    else
    {
        //͹��
        eta = ETA;
    }

    double angdist = eta * (1 - costheta);

    return angdist;
}

//�����żͼ
//���Ż�***************************************************************
void Image3D::FormDualGraph()
{
    cout << "Forming DualGraphy ..." << endl;
    DualGraph.resize(MeshNum);

    //�����㷨,O(n^2)
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

    //��ؾ���
    vector<vector<double> > GeodList;
    GeodList.resize(MeshNum);
    //�Ǿ���
    //vector<vector<double> > AngDistList;
    AngDistList.resize(MeshNum);

    //������ؾ��뼰�Ǿ���ı�
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            GeodList[i].push_back(Geod(MeshList[i], MeshList[DualGraph[i][j].idx]));
            AngDistList[i].push_back(AngDist(MeshList[i], MeshList[DualGraph[i][j].idx]));
        }
    }
    //ƽ��ֵ����
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

    //ƽ��ֵ
    AvgGeod = AvgGeod / (MeshNum * 3.0);
    AvgAngDist = AvgAngDist / (MeshNum * 3.0);

    //����ż���Ȩֵ��ֵ
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            DualGraph[i][j].weight = (DELTA * GeodList[i][j]) / AvgGeod + ((1 - DELTA) * AngDistList[i][j]) / AvgAngDist;
        }
    }

    cout << "Form DualGraph successfully ..." << endl;
}
//begin��ʼ���������̾���
//��ͨ���Գ��Լ���һ�������
void Image3D::Dijkstra(int begin)
{
    vector<bool> visited(MeshNum, false);
    Node Top(begin, 0); //���
    Node Neigh;

    priority_queue<Node> Que;
    Que.push(Top);           //�Ѷ���������ȼ�����

    while (!Que.empty()) {
        Top = Que.top();
        Que.pop(); //��ȡ���ȼ���߶���
        
        DistGraph[begin][Top.idx] = Top.weight < DistGraph[begin][Top.idx] ? Top.weight : DistGraph[begin][Top.idx];

        if (visited[Top.idx])
        {
            continue;  // ���ö��㱻���ʹ����򷵻�
        }
        visited[Top.idx] = true; // ���øö�����ʱ��
        for (int i = 0; i < 3; i++) //3Ϊ�̶�ֵ
        {
            Neigh.idx = DualGraph[Top.idx][i].idx; // ȡ����i�����򶥵����
            Neigh.weight = DualGraph[Top.idx][i].weight + Top.weight; // ��ӦȨ���޸�
            if (!visited[Neigh.idx])    // �������򶥵�δ�����ʣ���������
            {
                Que.push(Neigh);
            }
        }
    }
}
//����ÿ���㣨Mesh��֮�����̾���
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
//��ʼ����
void Image3D::IniREP()
{
    double MaxDist = 0;
    //��������������
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
//Sum(PA(fi) * Dist(f, fi)),�������е㵽�õ�ļ�PAȨ�ľ���֮��
double Image3D::PADistSum(int idx)
{
    double Sum = 0;
    for (int i = 0; i < MeshNum; i++)
    {
        Sum += PList[i].first * DistGraph[i][idx];
    }
    return Sum;
}
//Sum(PB(fi) * Dist(f, fi)),�������е㵽�õ�ļ�PBȨ�ľ���֮��
double Image3D::PBDistSum(int idx)
{
    double Sum = 0;
    for (int i = 0; i < MeshNum; i++)
    {
        Sum += PList[i].second * DistGraph[i][idx];
    }
    return Sum;
}
//����PList
void Image3D::UpdatePList()
{
    for (int i = 0; i < MeshNum; i++)
    {
        PList[i].first = DistGraph[i][REP.second] / (DistGraph[i][REP.first] + DistGraph[i][REP.second]);
        PList[i].second = DistGraph[i][REP.first] / (DistGraph[i][REP.first] + DistGraph[i][REP.second]);
    }
}
//����REP
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
//����ȷ�����Ӻ͸���
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
        //REP���䣬ֹͣ����
        if (tempREP == REP)
        {
            break;
        }
    }
    cout << "Form Cluster seed successfully ..." << endl;
}
//ģ���ָ�ҵ��߽�
void Image3D::FuzzySeg()
{
    cout << "Fuzzy Segmentation starts ..." << endl;
    ColorList.resize(MeshNum);
    //ģ������
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

    //�ұ߽��
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
            //��
            if (ColorList[i] == RED && ColorList[DualGraph[i][j].idx] != RED)
            {
                RedBound[i]++;
                ARedBound = i;
            }
        }
        for (int j = 0; j < 3; j++)
        {
            //��
            if (ColorList[i] == BLUE && ColorList[DualGraph[i][j].idx] != BLUE)
            {
                BlueBound[i]++;
                ABlueBound = i;
            }
        }
    }
    cout << "Fuzzy Segmentation successfully ..." << endl;
}
//��������ͼ
void Image3D::FormCapGraph()
{
    //resize
    CapGraph.resize(MeshNum);

    double tempCap;
    //����
    for (int i = 0; i < MeshNum; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            //������һ������S��T
            if ((ColorList[i] != UNCERTAIN && !RedBound[i] && !BlueBound[i]) || (ColorList[DualGraph[i][j].idx] != UNCERTAIN) && !RedBound[DualGraph[i][j].idx] && !BlueBound[DualGraph[i][j].idx])
            {
                CapGraph[i].push_back(Node(DualGraph[i][j].idx, INFINITY));
            }
            //ģ�������ڲ�
            else
            {
                tempCap = 1 / (1 + (AngDistList[i][j] / AvgAngDist));
                CapGraph[i].push_back(Node(DualGraph[i][j].idx, tempCap));
            }
        }
    }
}
//��������ͼ
bool Image3D::UpdateCapGraph()
{
    //���������������
    queue<int> QueBFS;
    //�Ƿ�visited
    vector<bool> isvisited(MeshNum, false);
    //ǰ��
    vector<int> Pre(MeshNum, -1);

    double MinCap = INFINITY;
    int Flag = 0;   //�ж��Ƿ��ҵ�ͨ·


    QueBFS.push(ARedBound);
    isvisited[ARedBound] = true;
    int Top;
    int temp;

    while (!QueBFS.empty())
    {
        Top = QueBFS.front();
        QueBFS.pop();

        ////ǰ������С����
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

        //�ж��Ƿ��ҵ��յ�
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
                //����ǰ��
                Pre[CapGraph[Top][i].idx] = Top;
                //���÷���״̬
                isvisited[CapGraph[Top][i].idx] = true;
            }
        }
    } 

    if (Flag)   //����--
    {
        //�ҵ���Сcap
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
        //����-MinCap
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
//��ϸ�ָ�
void Image3D::ExactSeg()
{
    cout << "Exact Segmentation starts ..." << endl;
    //����CapGraph
    FormCapGraph();
    while (UpdateCapGraph())
    {
    }
    
    stack<int> StaDFS;
    vector<bool> isvisited(MeshNum, false);

    //Ϳ��
    StaDFS.push(ARedBound);
    isvisited[ARedBound] = true;
    //��ȱ���
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

    //Ϳ��
    for (int i = 0; i < MeshNum; i++)
    {
        isvisited[i] = false;
    }
    StaDFS.push(ABlueBound);
    isvisited[ABlueBound] = true;
    //��ȱ���
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

    ////�ۿ���С��
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

    //�趨�����ɫ
    for (int i = 0; i < MeshNum; i++)
    {
        //��ɫ
        if (ColorList[i] == RED)
        {
            for (int j = 0; j < 3; j++)
            {
                PointList[MeshList[i].PointIdx[j]].SetColor(255, 0, 0);
            }
        }
        //��ɫ
        else if (ColorList[i] == BLUE)
        {
            for (int j = 0; j < 3; j++)
            {
                PointList[MeshList[i].PointIdx[j]].SetColor(0, 0, 255);
            }
        }
        ////�ۿ���С��
        //else if (ColorList[i] == GREEN)
        //{
        //    for (int j = 0; j < 3; j++)
        //    {
        //        PointList[MeshList[i].PointIdx[j]].SetColor(0, 255, 0);
        //    }
        //}
    }

    //д���ļ�
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
