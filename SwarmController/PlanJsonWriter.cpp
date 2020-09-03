#include "PlanJsonWriter.h"
#include <iostream>



PlanJsonWriter::PlanJsonWriter()
{
}



PlanJsonWriter::PlanJsonWriter(float homeLat, float homeLon)
	:m_homeLat(homeLat)
	,m_homeLon(homeLon)
{

}



PlanJsonWriter::~PlanJsonWriter()
{
}



std::array<float, 2> PlanJsonWriter::translate_coord_to_gps(const float& x, const float& y) const
{
	float latitude = m_homeLat + (y / meter_per_latitude());
	float longitude = m_homeLon + (x / meter_per_longitude());
	return { latitude, longitude };
}

std::array<float, 2> PlanJsonWriter::translate_gps_to_coord(const float& latitude, const float& longitude) const
{
	float x = (longitude - m_homeLon)*meter_per_longitude();
	float y = (latitude - m_homeLat)*meter_per_latitude();
	return { x, y };
}

void PlanJsonWriter::set_polygons(const vector<array<float, 2>> polygons)
{
	m_polygons.resize(polygons.size());
	copy(polygons.begin(), polygons.end(), m_polygons.begin());

	for (array<float, 2>& polygon : m_polygons)
	{
		polygon = translate_coord_to_gps(polygon.at(0), polygon.at(1));
	}
}



void PlanJsonWriter::tranlate_plan_items_to_file(const string& filePath, const rg_Point3D& takeoffPosition, const vector<PlanItem>& planItems)
{
	ofstream planfile(filePath);

	array<float, 2> takeoffGps = translate_coord_to_gps(takeoffPosition.getX(), takeoffPosition.getY());
	set_home(takeoffGps.at(0), takeoffGps.at(1));

	set_frame(3);

	Value Plan;
	Plan["fileType"] = "Plan";
	Plan["groundStation"] = "QGroundControl";
	Plan["version"] = 1;

	Value geoFence;
	add_circle_fence(geoFence);
	add_polygon_fence(geoFence);

	geoFence["version"] = 2;
	Plan["geoFence"] = geoFence;

	
	Value items;

	//Step 1: add take off
	const PlanItem& takeoffItem = planItems.front();
	array<float, 2> GPSCoordTakeoff = translate_coord_to_gps(takeoffItem.point.getX(), takeoffItem.point.getY());
	add_take_off(items, GPSCoordTakeoff.at(0), GPSCoordTakeoff.at(1), takeoffItem.point.getZ(), false);
	add_change_speed(items, takeoffItem.speed);

	//Step 2: add waypoints
	for (int i = 1; i < planItems.size() - 1; i++)
	{
		const PlanItem& waypointItem = planItems.at(i);
		array<float, 2> GPSCoordForWP = translate_coord_to_gps(waypointItem.point.getX(), waypointItem.point.getY());
		add_way_points(items, GPSCoordForWP.at(0), GPSCoordForWP.at(1), waypointItem.point.getZ(), waypointItem.loiterTime, true);
		add_change_speed(items, waypointItem.speed);
	}

	//Step 3: add land
	const PlanItem& landItem = planItems.back();
	array<float, 2> GPSCoordLand = translate_coord_to_gps(landItem.point.getX(), landItem.point.getY());
	add_land(items, GPSCoordLand.at(0), GPSCoordLand.at(1), landItem.point.getZ(), true);

	Value plannedhomeposition;
	plannedhomeposition.append(GPSCoordTakeoff.at(0));
	plannedhomeposition.append(GPSCoordTakeoff.at(1));
	plannedhomeposition.append(0);

	Value mission;

	mission["cruiseSpeed"] = 15;
	mission["firmwareType"] = 12;
	mission["hoverspeed"] = BASIC_SPEED;
	mission["plannedHomePosition"] = plannedhomeposition;
	mission["vehicleType"] = 2;
	mission["version"] = 2;

	mission["items"] = items;

	//Value point;
	//Value points;
	//points.append(point);

	Value rallypoints;
	//rallypoints["points"] = points;
	rallypoints["points"] = arrayValue;
	rallypoints["version"] = 1;

	Plan["rallyPoints"] = rallypoints;

	Plan["mission"] = mission;

	StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "\t";

	unique_ptr<Json::StreamWriter> writter(builder.newStreamWriter());

	writter->write(Plan, &planfile);

	planfile.close();
}



void PlanJsonWriter::add_circle_fence(Value& geoFence)
{
	if (m_circleFenceCheck == true)
	{
		Value fenceCenter;
		fenceCenter.append(m_center[0]);
		fenceCenter.append(m_center[1]);

		Value fenceCircle;
		fenceCircle["center"] = fenceCenter;
		fenceCircle["radius"] = m_radius;

		Value circle;
		circle["inclusion"] = true;
		circle["circle"] = fenceCircle;
		circle["version"] = 1;

		Value circles;
		circles.append(circle);

		geoFence["circles"] = circles;
	}
	else
	{
		geoFence["circles"] = arrayValue;
	}
}



void PlanJsonWriter::add_polygon_fence(Value& geoFence)
{
	if (m_polyFenceCheck == true)
	{
		Value bottomleft;
		bottomleft.append(m_bottomleft[0]);
		bottomleft.append(m_bottomleft[1]);
		Value bottomright;
		bottomright.append(m_bottomleft[0]);
		bottomright.append(m_topright[1]);
		Value topright;
		topright.append(m_topright[0]);
		topright.append(m_topright[1]);
		Value topleft;
		topleft.append(m_topright[0]);
		topleft.append(m_bottomleft[1]);

		Value polygon1;
		polygon1.append(bottomleft);
		polygon1.append(bottomright);
		polygon1.append(topright);
		polygon1.append(topleft);

		Value polygons;
		polygons["inclusion"] = true;
		polygons["polygon"] = polygon1;
		polygons["version"] = 1;

		Value polygon;
		polygon.append(polygons);

		geoFence["polygons"] = polygon;
	}
	else
	{
		geoFence["polygons"] = arrayValue;
	}
}



void PlanJsonWriter::add_take_off(Value& items, const double& lat, const double& lon, const double& alt, const bool& autoContinue)
{
	Value waypoint;

	waypoint["AMSLAltAboveTerrain"] = alt;
	waypoint["Altitude"] = alt;
	waypoint["AltitudeMode"] = 1;
	waypoint["autoContinue"] = autoContinue;
	waypoint["command"] = 22;
	waypoint["doJumpId"] = m_jumpID++;
	waypoint["frame"] = m_frame;
	waypoint["type"] = "SimpleItem";

	Value params;

	params.append(0);
	params.append(0);
	params.append(0);
	params.append(Value::null);
	params.append(lat);
	params.append(lon);
	params.append(alt);

	waypoint["params"] = params;

	items.append(waypoint);
}



void PlanJsonWriter::add_way_points(Value& items, const double& lat, const double& lon, const double& alt, const float& loiterTime, const bool& autoContinue)
{
	Value waypoint;

	waypoint["AMSLAltAboveTerrain"] = alt;
	waypoint["Altitude"] = alt;
	waypoint["AltitudeMode"] = 1;
	waypoint["autoContinue"] = autoContinue;
	waypoint["command"] = 16;
	waypoint["doJumpId"] = m_jumpID++;
	waypoint["frame"] = m_frame;
	waypoint["type"] = "SimpleItem";

	Value params;
	params.append(loiterTime);
	params.append(0);
	params.append(0);
	params.append(Value::null);
	params.append(lat);
	params.append(lon);
	params.append(alt);

	waypoint["params"] = params;

	items.append(waypoint);
}



void PlanJsonWriter::add_change_speed(Value& items, const double& speed)
{
	Value waypoint;

	waypoint["autoContinue"] = true;
	waypoint["command"] = 178;
	waypoint["doJumpId"] = m_jumpID++;
	waypoint["frame"] = 2;
	waypoint["type"] = "SimpleItem";

	Value params;
	params.append(1);
	params.append(speed);
	params.append(-1);
	params.append(0);
	params.append(0);
	params.append(0);
	params.append(0);

	waypoint["params"] = params;

	items.append(waypoint);
}



void PlanJsonWriter::add_return_to_launch(Value& items)
{
	Value waypoint;

	waypoint["autoContinue"] = true;
	waypoint["command"] = 20;
	waypoint["doJumpId"] = m_jumpID++;
	waypoint["frame"] = 2;
	waypoint["type"] = "SimpleItem";

	Value params;
	params.append(0);
	params.append(0);
	params.append(0);
	params.append(0);
	params.append(0);
	params.append(0);
	params.append(0);

	waypoint["params"] = params;

	items.append(waypoint);
}



void PlanJsonWriter::add_land(Value& items, const double& lat, const double& lon, const double& alt, const bool& autoContinue)
{
	Value waypoint;

	waypoint["AMSLAltAboveTerrain"] = alt;
	waypoint["Altitude"] = alt;
	waypoint["AltitudeMode"] = 1;
	waypoint["autoContinue"] = autoContinue;
	waypoint["command"] = 21;
	waypoint["doJumpId"] = m_jumpID++;
	waypoint["frame"] = m_frame;
	waypoint["type"] = "SimpleItem";

	Value params;

	params.append(0);
	params.append(0);
	params.append(0);
	params.append(Value::null);
	params.append(lat);
	params.append(lon);
	params.append(alt);

	waypoint["params"] = params;

	items.append(waypoint);
}
