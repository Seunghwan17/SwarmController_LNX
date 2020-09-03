#pragma once

#include <QWidget>
#include "VDRCOpenGLWidget.h"
#include "SwarmSimulator.h"

class SwarmDisplayer : public VDRCOpenGLWidget
{
	Q_OBJECT

public:
	SwarmSimulator * pSimulator;
	bool m_bShowDVD = false;
	bool m_bShowTrajectory = false;
	bool m_bShowConjunctionCircle = false;

public:
	SwarmDisplayer(QWidget *parent);
	~SwarmDisplayer();

	void draw();

	void draw_balls();
	void draw_shape(const vector<rg_Point3D>& shape, const Color3f& color) const;
	void draw_circle(const rg_Point3D& center, const rg_Point3D& normal, const float& radius, const Color3f& color) const;
	rg_Point3D calculate_vector_on_plane(const rg_Point3D& vector, const rg_Point3D& normal) const;

	void draw_path(DroneBall& drone, const double& time);
};
