#pragma once

#include "constForDynamicVD3DLib.h"
#include <string>

using namespace std;

enum DRONE_STATUS {NORMAL, AVOIDING, ARRIVAL};
enum SWARM_CONTROL_EVENT_TYPE { SC_CONJUNCTION, SC_RESTORATION, SC_ARRIVAL, SC_NULL};
enum PATH_GENERATION_MODE {TIME_ON_TARGET, CONSTANT_SPEED};
enum AVOIDANCE_METHOD{CHANGE_SPEED, BY_PASS};
enum WAYPOINT_TYPE {INIT_LOC, PAUSE, DESTINATION, AVOIDANCE_START, AVOIDANCE_END, ROLE_CHANGE};
enum PRIORITY_TYPE {PRIORITY_IDENTICAL, PRIORITY_GIVEN, PRIORITY_DISTANCE};
enum AVOIDANCE_TYPE {ON_THE_PLANE, OFF_THE_PLANE};

#define PRINT_SCENES
#define PRINT_MISSION

const Color3f BASIC_DRONE_COLOR = BLACK;
const Color3f COLOR_SC = RED;
const Color3f COLOR_RT = BLACK;
const Color3f COLOR_RC = YELLOW;

//Parameters for time on target mode
const double MAX_SPEED = 10;
const int MAX_ALTITUDE = 4;

const double MAX_WAITING_TIME = 30;

const float ROTATION_FOR_UNIVERSITY_STADIUM = 0.188834125221208;

const float BASIC_SPEED = 1.5;
const float TAKEOFF_SPEED = 1;
const float AVOIDANCE_SPEED = 1.5;

const float INCREASE_TIME_STEP = 0.1f;

enum FESTA_COMMAND {
	UNKNOWN_COMMAND, COMMENT_SIGN, NUMBER_OF_DRONES, CONJUNCT_RADIUS, COLLISION_RADIUS,
	COORDINATE_ORIGIN_LATITUDE, COORDINATE_ORIGIN_LONGITUDE, COORDINATE_ROTATION_ANGLE, NUMBER_OF_SCENES,
	HOME_POSITION, TAKEOFF_AND_PREPARE_TO_FLY, SCENE, PREPARE_TO_LAND, LAND
};

enum SWITCHING_PLAN_TYPE { APRIORI_DEFINED_FLIGHT_PLAN, MIN_TOTAL_FLIGHT_DISTANCES, UNKNOWN_SWITCHING_PLAN };

struct WayPoint
{
	WAYPOINT_TYPE type;
	rg_Point3D point;
	double time;
	Color3f color;
};


struct PlotObject
{
	rg_Point3D point;
	float priority;
	Color3f color;
};


struct Plot
{
	string plotFileName;
	bool isReal;
	SWITCHING_PLAN_TYPE switchingPlanType;
	float switchingVelocity;
	float plotWaitingTime;
	float plotTransitionTime;
	float tiltAngle;
	vector<PlotObject> plotObjects;
	vector<int> destinationAssignMap;
};


struct PlanItem
{
	int droneID;
	rg_Point3D point;
	float speed;
	WAYPOINT_TYPE type;
	float loiterTime;
	Color3f color;
};



