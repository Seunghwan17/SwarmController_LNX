#include "Scene.h"



Scene::Scene()
: m_ID(-1)
, m_positions()
{
}



Scene::Scene(const int& ID)
: m_ID(ID)
, m_positions() 
{
}



Scene::Scene(const Scene& scene)
: m_ID(scene.m_ID)
, m_positions(scene.m_positions) 
{
}



Scene::~Scene()
{
}



Scene& Scene::operator =(const Scene& scene)
{
    if (this != &scene) {
        m_ID = scene.m_ID;
        m_positions = scene.m_positions;
    }

    return *this;
}



int                             Scene::ID() const
{
    return m_ID;
}


unsigned int    Scene::number_of_positions() const
{
    return m_positions.size();
}



rg_Point3D                      Scene::position(const int& i) const
{
    if (i >= 0 && i < m_positions.size()) {
        return m_positions[i];
    }
    else {
        return rg_Point3D();
    }
}



const vector<rg_Point3D>&  Scene::positions() const
{
    return m_positions;
}



vector<rg_Point3D>&  Scene::positions() 
{
    return m_positions;
}



void    Scene::set_ID(const int& ID)
{
    m_ID = ID;
}



void    Scene::set_position(const int& i, const rg_Point3D& position)
{
    if (i >= 0 && i < m_positions.size()) {
        m_positions[i] = position;
    }
}



void    Scene::set_positions(const vector<rg_Point3D>& positions)
{
    m_positions = positions;
}



void    Scene::set_scene(const int& ID, const vector<rg_Point3D>& positions)
{
    m_ID = ID;
    m_positions = positions;
}



