#pragma once

#include "Scene.h"


#include <vector>
using namespace std;

class Snapshot : public Scene
{
private:
    double  m_time;
    vector<float>   m_speed;


public:
    Snapshot();
    Snapshot(const int& ID);
    Snapshot(const Snapshot& snapshot);
    ~Snapshot();

    Snapshot& operator =(const Snapshot& snapshot);

    double  time() const;
    float                 speed(const int& i) const;
    const vector<float>&  speeds() const;
    vector<float>&        speeds();


    void    set_time(const double& time);
    void    set_speed(const int& i, const float& speed);
    void    set_speeds(const vector<float>& speeds);
};

