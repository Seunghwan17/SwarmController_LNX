#pragma once

#include "rg_Point3D.h"

#include <vector>
using namespace std;


class Scene
{
private:
    int                     m_ID;
    vector<rg_Point3D> m_positions;

public:
    Scene();
    Scene(const int& ID);
    Scene(const Scene& scene);
    virtual ~Scene();

    Scene& operator =(const Scene& scene);

    int                        ID() const;
    unsigned int               number_of_positions() const; 
    rg_Point3D                 position(const int& i) const;
    const vector<rg_Point3D>&  positions() const;
    vector<rg_Point3D>&        positions();

    void                       set_ID(const int& ID);
    void                       set_position(const int& i, const rg_Point3D& position);
    void                       set_positions(const vector<rg_Point3D>& positions);
    void                       set_scene(const int& ID, const vector<rg_Point3D>& positions);


};

