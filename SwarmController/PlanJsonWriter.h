#pragma once
#include <array>
#include <vector>

#include <fstream>
#include "json/json.h"
#include "constForSwarmSimulator.h"

using namespace std;
using namespace Json;

class PlanJsonWriter
{

private:
	float m_homeLat = 0.0f;
	float m_homeLon = 0.0f;
	
	float m_launchLat = 0.0f;
	float m_launchLon = 0.0f;

	int m_frame = 3;
	int m_jumpID = 1;

	array<float, 2> m_center = {0.0,};//latitude,longitude
	float m_radius = 0.0;

	array<float, 2> m_bottomleft = { 0.0f, };
	array<float, 2> m_topright = { 0.0f, };

	vector<array<float, 2>> m_polygons;//latitude, longitude

	bool m_circleFenceCheck = 0;
	bool m_polyFenceCheck = 0;

	

public:
	PlanJsonWriter();
	PlanJsonWriter(float homeLat, float homeLon);
	~PlanJsonWriter();

	array<float, 2> translate_coord_to_gps(const float& x, const float& y) const;
	array<float, 2> translate_gps_to_coord(const float& latitude, const float& longitude) const;

	inline double meter_per_latitude() const { return 111132.92 - 559.82*cos(2 * m_homeLat) + 1.175*cos(4 * m_homeLat) - 0.0023*cos(6 * m_homeLat); }
	inline double meter_per_longitude() const { return 111412.84*cos(m_homeLat) - 93.5*cos(3 * m_homeLat) + 0.118*cos(5 * m_homeLat); }

	inline void set_home(const float& homeLat, const float& homeLon) { m_homeLat = homeLat;  m_homeLon = homeLon; }
	inline void set_launchPoint(const float& launchLat, const float& launchLon) { m_launchLat = launchLat; m_launchLon = launchLon; }

	void set_polygons(const vector<array<float, 2>> polygons);
	inline void set_frame(const int& frame) { m_frame = frame; }

	inline void set_geoFence(const array<float, 2>& center, const float& radius) { m_center[0] = center[0]; m_center[1] = center[1]; m_radius = radius; m_circleFenceCheck = 1; }
	inline void set_geoFence(const array<float, 2>& bottomleft, const array<float, 2> topright) { m_bottomleft[0] = bottomleft[0]; m_bottomleft[1] = bottomleft[1]; m_topright[0] = topright[0]; m_topright[1] = topright[1]; m_polyFenceCheck = 1; }

	void tranlate_plan_items_to_file(const string& filePath, const rg_Point3D& takeoffPosition, const vector<PlanItem>& planItems);

	void add_circle_fence(Value& geoFence);
	void add_polygon_fence(Value& geoFence);
	void add_take_off(Value& items, const double& lat, const double& lon, const double& alt, const bool& autoContinue);
	void add_way_points(Value& items, const double& lat, const double& lon, const double& alt, const float& loiterTime, const bool& autoContinue);
	void add_change_speed(Value& items, const double& speed);
	void add_return_to_launch(Value& items);
	void add_land(Value& items, const double& lat, const double& lon, const double& alt, const bool& autoContinue);
};

