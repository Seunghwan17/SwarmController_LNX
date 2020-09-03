#include "SnapShot.h"




Snapshot::Snapshot()
: Scene()
, m_time(0.0)
{
}



Snapshot::Snapshot(const int& ID)
: Scene(ID)
, m_time(0.0)
{
}



Snapshot::Snapshot(const Snapshot& snapshot)
: Scene(snapshot)
, m_time(snapshot.m_time)
{
}



Snapshot::~Snapshot()
{
}



Snapshot& Snapshot::operator =(const Snapshot& snapshot)
{
    if (this != &snapshot) {
        Scene::operator=(snapshot);
        m_time = snapshot.m_time;
    }

    return *this;
}



double  Snapshot::time() const
{
    return m_time;
}



float                 Snapshot::speed(const int& i) const
{
    if (i >= 0 && i < m_speed.size()) {
        return m_speed[i];
    }
    else {
        return 0.0f;
    }
}



const vector<float>&  Snapshot::speeds() const
{
    return m_speed;
}



vector<float>&        Snapshot::speeds()
{
    return m_speed;
}



void    Snapshot::set_time(const double& time)
{
    m_time = time;
}

void    Snapshot::set_speed(const int& i, const float& speed)
{
    if (i >= 0 && i < m_speed.size()) {
        m_speed[i] = speed;
    }
}



void    Snapshot::set_speeds(const vector<float>& speeds)
{
    m_speed = speeds;
}



