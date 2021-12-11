//author: CaO
//date: 2021/1/12
//describe: 基于模糊聚类和最小割的层次化三维网格分割算法
#include "Image3D.h"

int main()
{
    //clock_t time_start = clock();

    //Image3D myImage(string("chair"));
    //Image3D myImage(string("dinosaur.2k"));
    //Image3D myImage(string("horse.2k"));
    string FileName;
    cout << "Please enter filename (without .obj): " << endl;
    cin >> FileName;
    Image3D myImage(FileName);

    //clock_t time_end = clock();
    //cout << "Time:" << (time_end - time_start) / CLOCKS_PER_SEC << "s" << endl;

    return 0;
}