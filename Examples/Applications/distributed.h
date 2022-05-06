//
// Created by lz on 22-5-2.
//

#ifndef ORB_SLAM3_DISTRIBUTED_H
#define ORB_SLAM3_DISTRIBUTED_H
#include <System.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
//#include "Map.h"

using namespace std;
class distributedSLAMSystem{
private:
    int subThreadsNum;
    vector<ORB_SLAM3::System> Subsystems;
    ORB_SLAM3::Map GlobalMap;

};

#endif //ORB_SLAM3_DISTRIBUTED_H
