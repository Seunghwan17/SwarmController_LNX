#pragma once

#include "constForSwarmSimulator.h"
#include "DynamicSimulator3D.h"
#include "DroneBall.h"
#include <list>
#include <vector>
#include <string>
#include <unordered_map>
#include <tuple>
#include "rg_Point3D.h"
//#include "EntityAccessiblePriorityQ.h"

using namespace std;

class SwarmSimulator : public DynamicSimulator3D
{
private:
	PATH_GENERATION_MODE m_pathGenerationMode = CONSTANT_SPEED;
	PRIORITY_TYPE m_priorityType = PRIORITY_GIVEN;
	AVOIDANCE_TYPE m_avoidanceType = OFF_THE_PLANE;
	vector<DroneBall> m_drones;

	Plot* m_targetPlot;
	unordered_map<int, int> m_mapForDestinationAssignment;
	
	EventPredictionQ m_contactQ;
	EventPredictionQ m_restorationQ;
	EventPredictionQ m_arrivalQ;

	unordered_map<int, int> m_mapFromIDToLatestContactKey;
	unordered_map<int, int> m_mapFromIDToLatestRestorationKey;
	unordered_map<int, int> m_mapFromIDToLatestArrivalKey;

	//EntityAccessiblePriorityQ<VFaceCore*> m_contactQ;
	//EntityAccessiblePriorityQ<DroneBall*> m_restorationQ;
	//EntityAccessiblePriorityQ<DroneBall*> m_arrivalQ;

	double m_timeForSimulation;
	int m_indexOfLastVelocityChangeEvent;
	vector<tuple<int, double, rg_Point3D>> m_velocityChangeHistory;

	//Common parameters
	double m_vehicleRadius = 0.25;
	double m_collisionRadius = 1;
	double m_conjunctionRadius = 1.5;
	double m_maxSpeedMultiplier = 1.5;
	double m_avoidanceAlt = 5.0;

	//Parameters for time on target mode
	double m_maxSpeed = 5;
	double m_targetArrivalTime = 60;

	//Parameters for contant speed mode
	double m_basicSpeed = BASIC_SPEED;
	double m_maxAvoidanceDuration = 30;

public:
	//for display
	list<DroneBall*> m_dronesOnEvasion;
	//unordered_map<DroneBall*, DECIMAL> mapForColorChangedTime;

public:
	SwarmSimulator();
	virtual ~SwarmSimulator();

	void clear();

	inline vector<DroneBall>& get_drones() { return m_drones; }
	inline const list<DroneBall*>& get_drones_on_evasion() const { return m_dronesOnEvasion; }
	inline const vector<tuple<int, double, rg_Point3D>>& get_velocity_change_history() const { return m_velocityChangeHistory; }
	inline const double& get_time_for_simulation() const { return m_timeForSimulation; }
	inline const int& get_index_of_last_velocity_change_event() const { return m_indexOfLastVelocityChangeEvent; }

	inline PATH_GENERATION_MODE get_path_generation_mode() const { return m_pathGenerationMode; }
	inline PRIORITY_TYPE get_priority_type() const { return m_priorityType; }
	inline double get_vehicle_radius() const { return m_vehicleRadius; }
	inline double get_collision_radius() const { return m_collisionRadius; }
	inline double get_conjunction_radius() const { return m_conjunctionRadius; }
	inline double get_max_speed_multiplier() const { return m_maxSpeedMultiplier; }
	inline double get_avoidance_altitude() const { return m_avoidanceAlt; }

	inline double get_max_speed() const { return m_maxSpeed; }
	inline double get_target_arrival_time() const { return m_targetArrivalTime; }

	inline double get_basic_speed() const { return m_basicSpeed; }
	inline double get_max_avoidance_duration() const { return m_maxAvoidanceDuration; }

	inline void set_target_plot(Plot& targetPlot) { m_targetPlot = &targetPlot; }

	inline void set_path_generation_mode(const PATH_GENERATION_MODE& pathGenerationMode) { m_pathGenerationMode = pathGenerationMode; }
	inline void set_priority_type(const PRIORITY_TYPE& priorityType) { m_priorityType = priorityType; }
	inline void set_vehicle_radius(const double& vehicleRadius) { m_vehicleRadius = vehicleRadius; }
	inline void set_collision_radius(const double& collisionRadius) { m_collisionRadius = collisionRadius; }
	inline void set_conjunction_radius(const double& conjunctionRadius) { m_conjunctionRadius = conjunctionRadius; }
	inline void set_max_speed_multiplier(const double& maxSpeedMultiplier) { m_maxSpeedMultiplier = maxSpeedMultiplier; }
	inline void set_avoidance_altitude(const double& avoidaceAlt) { m_avoidanceAlt = avoidaceAlt; }

	inline void set_max_speed(const double& maxSpeed) { m_maxSpeed = maxSpeed; }
	inline void set_target_arrival_time(const double& targetArrivalTime) { m_targetArrivalTime = targetArrivalTime; }
				
	inline void set_basic_speed(const double& basicSpeed) { m_basicSpeed = basicSpeed; }
	inline void set_max_avoidance_duration(const double& maxAvoidanceDuration) { m_maxAvoidanceDuration = maxAvoidanceDuration; }

	void initialize_scene_change(const vector<int>& destinationAssignMap);


	void add_arc_path(DroneBall& drone, int numArcPoints, float curvature);
	void prepare_simulation();
	void make_drone_balls(const Plot& initialPlot);
	void assign_priority();

	void initialize_drone_move();
	void update_velocity_to_arrive_at_target_time(DroneBall& drone, const double& currTime);

	int construct_initial_swarm_event_queue();
	void add_initial_contact_event();
	void add_initial_arrival_event();

	pair<SWARM_CONTROL_EVENT_TYPE, double> find_next_swarm_control_event_info();
	const PredictedEvent* find_imminent_contact_event();
	const PredictedEvent* find_imminent_restoration_event();
	const PredictedEvent* find_imminent_arrival_event();

	bool is_contact_event_valid(const PredictedEvent& event);
	bool is_restoration_event_valid(const PredictedEvent& event);
	bool is_arrival_event_valid(const PredictedEvent& event);

	void adjust_contact_event_key_map_for_face(VFaceCore* face, const bool& isBornFace);


	virtual void	 proceed_to_time_for_history_generation(const double& time);
	virtual void increase_time(const double& timeIncrement);
	void process_DVD_event();
	void process_swarm_control_event(const SWARM_CONTROL_EVENT_TYPE& eventType);

	virtual void	 go_to_time_using_history(const double& targetTime);
	virtual void increase_time_using_history(const double& timeIncrement);

	array<double, 2> calculate_contact_interval_for_face_from_now(const VFaceCore* face, const float& contactDistance, const double& currTime) const;
	array<double, 2> calculate_contact_interval_from_now(const rg_Point3D& relativePosition, const rg_Point3D& relativeVelocity, const float& contactDistance) const;

	bool add_contact_event_for_new_face(const FlipEvent3D* edgeFlipEvent);

	void avoid_collision();
	set<DynamicBall*> collect_neighbors_for_drone_pair(const array<DroneBall*, 2>& dronePair);

	void restore_velocity();
	void process_arrival();
	bool is_every_ball_arrive() const { return m_arrivalQ.empty() && m_restorationQ.empty(); }

	void update_contact_event_for_neighbors(const DroneBall* drone, const double& currTime);
	void update_restoration_event_for_neighbors(const DroneBall* drone, const double& currTime);

	bool adjust_contact_event_queue(const double& eventTime, VFaceCore* willBeConjunctiveFace);
	bool adjust_restoration_event_queue(const double& eventTime, DroneBall* willRestorateDrone);
	bool adjust_arrival_event_queue(const double& eventTime, DroneBall* willArriveDrone);

	pair<int, double> find_optimal_speed_config(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& isRoleChange = false) const;

	array<rg_Point3D, 2> find_basic_velocities(const array<DroneBall*, 2> dronesOnContact, const double& currTime) const;
	array<rg_Point3D, 2> find_velocities_for_role_change(const array<DroneBall*, 2> dronesOnContact, const double& currTime) const;
	
	double find_contact_escape_time(const DroneBall* drone, const rg_Point3D& velocity, const set<DynamicBall*>& neighbors, const double& currTime) const;

	bool is_avoidance_valid(const pair<int, double>& speedConfig, const double& eventTime) const;

	list<pair<int, double>> collect_valid_velocity_configs_with_contact_escape_time(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& bRoleChange = false) const;
	//for debug
	void collect_valid_velocity_configs_with_contact_escape_time_debug(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& bRoleChange = false) const;
	
	
	bool is_velocity_config_valid(const int& velocityConfig, const array<DroneBall*, 2> dronesOnContact) const;
	double calculate_nearest_collision_time_with_neighbors(const DroneBall* drone, const rg_Point3D& velocity, const set<DynamicBall*>& neighbors, const double& currTime) const;
	double calculate_collision_time_from_now(const rg_Point3D& relativePosition, const rg_Point3D& relativeVelocity) const;
	
	void change_speed_to_avoid_collision(DroneBall* drone, const int& velocityConfig, const double& conjunctionTime, const double& escapeTime);
	void change_role_to_avoid_collision(const array<DroneBall*, 2> dronesOnContact, const int& velocityConfig, const double& contactTime, const double& escapeTime);
	void update_way_points_for_role_change(DroneBall* drone, const double& currTime, const WayPoint& destinationWP, const double& speedConfig = 1.0);

	WayPoint make_way_point_at_curr_position(const DroneBall* drone, const double& time, const WAYPOINT_TYPE type, const Color3f& color);
	
	WayPoint make_arc_way_point(DroneBall& drone, const float& ratio, const double& z);
	void update_velocity_to_next_way_point(DroneBall* drone, const double& currTime);
	//void stop_drones_to_avoid_collision(const array<DroneBall*, 2> dronesOnContact, const DECIMAL& contactTime);

	void take_by_pass_to_avoid_collision(DroneBall* avoidingDrone, const double& contactTime, const rg_Point3D& avoidanceVelocity);
	double calculate_nearest_collision_time_for_candidate_avoidance(DroneBall* avoidingDrone, const double& contactTime, const rg_Point3D& candidateAvoidanceVelocity);

	inline void add_velocity_change_history(const DroneBall* drone, const double& time) { m_velocityChangeHistory.push_back({ drone->get_ID(), time, drone->get_velocity_vector() }); }
	void print_drone_info(const string& fileName) const;
};

