#ifndef GPSDATA_H
#define GPSDATA_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <vector>
#include <memory> 


using namespace std;
struct gpsData
{
    double timestamp;
    double x, y, z;
    std::string state;
};

struct orbData
{
    double timestamp;
    double x, y, z, q0, q1, q2, w;
};

bool gps_file(std::string tempStrname, vector<shared_ptr<gpsData>> &dataTemp);
bool orb_file(std::string tempStrname, vector<shared_ptr<orbData>> &dataTemp);

template <class T>
void convertFromString(T &value, const std::string &s)
{
    std::stringstream ss(s);
    ss >> value;
}

bool gps_file(std::string tempStrname, vector<shared_ptr<gpsData>> &dataTemp)
{
    ifstream tempStr;

    tempStr.open(tempStrname.c_str());
    if (!tempStr.is_open())
    {
        std::cout << "打开GPS文件失败" << std::endl;
    };

    string s;
    while (tempStr >> s)
    {

 
        shared_ptr<gpsData> gpsTemp(new gpsData);
        convertFromString(gpsTemp->timestamp, s);
        tempStr >> s;
        convertFromString(gpsTemp->x, s);
        tempStr >> s;
        convertFromString(gpsTemp->y, s);
        tempStr >> s;
        convertFromString(gpsTemp->z, s);
        tempStr >> s;
        convertFromString(gpsTemp->state, s);
        dataTemp.push_back(gpsTemp);

    }
    tempStr.close();
    return 1;
}

bool orb_file(std::string tempStrname, vector<shared_ptr<orbData>> &dataTemp)
{
    ifstream tempStr;

    tempStr.open(tempStrname.c_str());
    if (!tempStr.is_open())
    {
        std::cout << "打开ORB文件失败" << std::endl;
    };

    string s;
    while (tempStr >> s)
    {
        shared_ptr<orbData> gpsTemp(new orbData);
        convertFromString(gpsTemp->timestamp, s);
        tempStr >> s;
        convertFromString(gpsTemp->x, s);
        tempStr >> s;
        convertFromString(gpsTemp->y, s);
        tempStr >> s;
        convertFromString(gpsTemp->z, s);
        tempStr >> s;
        convertFromString(gpsTemp->q0, s);
        tempStr >> s;
        convertFromString(gpsTemp->q1, s);
        tempStr >> s;
        convertFromString(gpsTemp->q2, s);
        tempStr >> s;
        convertFromString(gpsTemp->w, s);
        dataTemp.push_back(gpsTemp);
    }
    tempStr.close();
    return 1;
}




#endif // GPSDATA_H
