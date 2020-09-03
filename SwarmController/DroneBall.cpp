#include "DroneBall.h"
#include <random>
#include <ctime>
#include "constForSwarmSimulator.h"



DroneBall::DroneBall()
	: DynamicBall()
{
	m_status = NORMAL;
	m_givenPriority = 0.0f;
	m_WPIndex = 0;
}



DroneBall::DroneBall(const int& ID, BALL_TYPE type, const Sphere& sphere, const rg_Point3D& velocityVector, const DRONE_STATUS& status/*=NORMAL*/, const double& time /*= 0.0*/, const float& priority /*= 0.0f*/)
	: DynamicBall(ID, type, sphere, velocityVector, time)
{
	m_status = status;
	m_givenPriority = priority;
	m_WPIndex = 0;
}



DroneBall::DroneBall(const int& ID, BALL_TYPE type, const Sphere& sphere, const rg_Point3D& velocityVector, const Color3f& color, const DRONE_STATUS& status /*= NORMAL*/, const double& time /*= 0.0*/, const float& priority /*= 0.0f*/)
	: DynamicBall(ID, type, sphere, velocityVector, color, time)
{
	m_status = status;
	m_givenPriority = priority;
	m_WPIndex = 0;
}



DroneBall::DroneBall(const DroneBall& rhs)
{
	copy(rhs);
}



DroneBall::~DroneBall()
{
}



void DroneBall::copy(const DroneBall& rhs)
{
	DynamicBall::copy(rhs);
	m_status = rhs.m_status;
	m_givenPriority = rhs.m_givenPriority;
	m_WPIndex = rhs.m_WPIndex;
	m_waypoints = rhs.m_waypoints;
}



void DroneBall::clear()
{
	DynamicBall::clear();
	m_status = NORMAL;
	m_givenPriority = 0.0;
	clear_way_points();
}



void DroneBall::clear_way_points()
{
	m_waypoints.clear();
	m_WPIndex = 0;
}



DroneBall& DroneBall::operator=(const DroneBall& rhs)
{
	if (this == &rhs)
		return *this;

	copy(rhs);
	return *this;
}



void DroneBall::add_way_point(const int& index, const WayPoint& wayPoint)
{
	m_waypoints.insert(m_waypoints.begin() + index, wayPoint);
}



rg_Point3D DroneBall::find_destination() const
{
	if (m_status == ARRIVAL)
		return get_sphere_center();
	else
		return find_next_WP().point;
}