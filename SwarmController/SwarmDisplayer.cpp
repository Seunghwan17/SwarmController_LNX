#include "SwarmDisplayer.h"

#include "BallGeneratorCore.h"

SwarmDisplayer::SwarmDisplayer(QWidget *parent)
	: VDRCOpenGLWidget(parent)
{
	pSimulator = nullptr;
}



SwarmDisplayer::~SwarmDisplayer()
{
}



void SwarmDisplayer::draw()
{
	if (pSimulator != nullptr)
	{
		draw_balls();
		//draw_shape(pSimulator->get_shapes().at(0), GREY);
		//draw_shape(pSimulator->get_shapes().at(1), GREY);
		
		if (m_bShowDVD)
		{
			list<VEdgeCore*> edges;
			pSimulator->collect_edges_for_drawing(edges);
			for (auto& edge : edges)
			{
				draw_line(edge->getStartVVertex()->getPoint(), edge->getEndVVertex()->getPoint(), 1, BLACK);
			}
		}
	}
}



void SwarmDisplayer::draw_balls()
{
	double currentTime = pSimulator->get_dynamic_VD().get_current_time();
	//DECIMAL currentTime = pSimulator->get_time_for_simulation();
	for (vector<DroneBall>::iterator it = pSimulator->get_drones().begin(); it != pSimulator->get_drones().end(); it++)
	{
		DroneBall& currDrone = *it;
		Color3f color = currDrone.get_color();

		switch (currDrone.get_status())
		{
		case AVOIDING:
			color = COLOR_SC;
			break;
		case ARRIVAL:
		case NORMAL:
		default:
			break;
		}


		//draw_sphere((*it).calculate_center_at_time(currentTime), SELF, color);
		draw_sphere(currDrone.calculate_position_at_time(currentTime), pSimulator->get_collision_radius(), color);

		if (m_bShowTrajectory)
			draw_path(currDrone, currentTime);

		if (m_bShowConjunctionCircle)
			draw_circle(currDrone.calculate_position_at_time(currentTime), localZ, pSimulator->get_conjunction_radius(), GREEN);
	}
}



void SwarmDisplayer::draw_shape(const vector<rg_Point3D>& shape, const Color3f& color) const
{
	for (int i = 0; i < shape.size(); i++)
	{
		draw_sphere(shape.at(i), pSimulator->get_conjunction_radius(), color);
	}
}



void SwarmDisplayer::draw_circle(const rg_Point3D& center, const rg_Point3D& normal, const float& radius, const Color3f& color) const
{
	rg_Point3D vector1 = calculate_vector_on_plane(rg_Point3D(1, 0, 0), normal).getUnitVector();
	double dotProduct = normal.innerProduct(vector1);

	rg_Point3D vector2 = normal.crossProduct(vector1).getUnitVector();

	int numSegments = 16;
	for (int i = 0; i < numSegments; i++)
	{
		double angle1 = 2 * M_PI*i / numSegments;
		double angle2 = 2 * M_PI*(i+1) / numSegments;
		rg_Point3D point1 = (cos(angle1)*vector1 + sin(angle1)*vector2)*radius+center;
		rg_Point3D point2 = (cos(angle2)*vector1 + sin(angle2)*vector2)*radius + center;
		draw_line(point1, point2, 1, BLACK);
	}
}



rg_Point3D SwarmDisplayer::calculate_vector_on_plane(const rg_Point3D& vector, const rg_Point3D& normal) const
{
	rg_Point3D unitNormal = normal.getUnitVector();
	rg_Point3D vectorOnplane = vector - (vector.innerProduct(unitNormal))*unitNormal;
	return vectorOnplane;
}



void SwarmDisplayer::draw_path(DroneBall& drone, const double& time)
{
	vector<WayPoint>& wayPoints = drone.get_way_points();
	rg_Point3D lastPt = drone.calculate_position_at_time(time);
	for (int i = drone.get_WP_index()+1; i < wayPoints.size(); i++)
	{
		if (wayPoints.at(i).type == DESTINATION || wayPoints.at(i).type == PAUSE)
		{
			draw_line_stipple(lastPt, wayPoints.at(i).point, 1, BLACK);
			lastPt = wayPoints.at(i).point;
		}
	}
}
