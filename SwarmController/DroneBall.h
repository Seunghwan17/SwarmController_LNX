#pragma once

#include "DynamicBall.h"
#include "constForSwarmSimulator.h"
#include "rg_Point3D.h"


class DroneBall : public DynamicBall
{
private:
	DRONE_STATUS m_status;
	float m_priority;
	float m_givenPriority;
	vector<WayPoint> m_waypoints;
	int m_WPIndex;

public:
	DroneBall();
	DroneBall(const int& ID, BALL_TYPE type, const Sphere& sphere, const rg_Point3D& velocityVector, const DRONE_STATUS& status=NORMAL, const double& time = 0.0, const float& priority = 0.0f);
	DroneBall(const int& ID, BALL_TYPE type, const Sphere& sphere, const rg_Point3D& velocityVector, const Color3f& color, const DRONE_STATUS& status = NORMAL, const double& time = 0.0, const float& priority = 0.0f);
	DroneBall(const DroneBall& rhs);
	virtual ~DroneBall();

	DroneBall& operator = (const DroneBall& rhs);

	virtual void copy(const DroneBall& rhs);
	virtual void clear();
	void clear_way_points();

	inline void set_status(const DRONE_STATUS& status) { m_status = status; }
	inline void set_priority(const float& priority) { m_priority = priority; }
	inline void set_given_priority(const float& givenPriority) { m_givenPriority = givenPriority; }
	inline void set_WP_index(const int& index) { m_WPIndex = index; }

	inline DRONE_STATUS get_status() const { return m_status; }
	inline float get_priority() const { return m_priority; }
	inline float get_given_priority() const { return m_givenPriority; }
	inline vector<WayPoint>& get_way_points() { return m_waypoints; }
	inline int get_WP_index() const { return m_WPIndex; }

	//utility
	inline void increase_WP_index() { m_WPIndex++; }
	inline void decrease_WP_index() { m_WPIndex--; }
	
	inline const WayPoint& find_current_WP() const { return m_waypoints.at(m_WPIndex); }
	inline const WayPoint& find_next_WP() const { return m_waypoints.at(m_WPIndex + 1); }
	inline const WayPoint& find_final_WP() const { return m_waypoints.back(); }
	
	inline double calculate_arrival_time_to_next_WP(const double& currTime) const { return calculate_position_at_time(currTime).distance(find_next_WP().point) / get_velocity_vector().magnitude() + currTime; }

	void add_way_point(const int& index, const WayPoint& wayPoint);
	inline void add_way_point_at_current(const WayPoint& wayPoint) { add_way_point(m_WPIndex, wayPoint); }
	inline void add_way_point_at_next(const WayPoint& wayPoint) { add_way_point(m_WPIndex+1, wayPoint); }


	rg_Point3D find_destination() const;
};
