#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_SwarmController.h"
#include "SwarmSimulator.h"
#include "SwarmDisplayer.h"
#include <string>
#include <vector>
#include "PlanJsonWriter.h"

class SwarmController : public QMainWindow
{
	Q_OBJECT

private:
	SwarmSimulator m_simulator;
	QTimer* timer;
	int computationTime = 0;
	SwarmDisplayer* evasionView;
	int m_currSceneID = 0;
	map<int, int> m_currPositionMap;
	float m_currTime = 0;

	//informations for generate plan
	int m_numDrone = 0;
	float m_collisionRadius = 1.0f;
	float m_conjunctionRadius = 1.5f;
	float m_avoidanceAlt = 5.0f;
	float m_loiterTime = 0;
	float m_homeLat = 0.0f;
	float m_homeLon = 0.0f;
	float m_arenaFloorAlt = 0.0f;
	float m_screenTiltAngle = 0.0f;
	float m_arenaRotationAngle = 0.0f;
	int m_numPlots = 0;
	string m_planFileName;
	vector<Plot> m_plots;
	vector<vector<WayPoint>> m_wayPointsForDrones;
	vector<vector<PlanItem>> m_planItemsForDrones;

	int m_numOfTakeoffClusters;
	map<int, int> m_mapFromIDToTakeoffCluster;
	float m_timeIntervalBetweenTakeoffCluster;

	vector<rg_Point3D> m_takeoffPositions;
	vector<rg_Point3D> m_landPositions;

public:
	SwarmController(QWidget *parent = Q_NULLPTR);
	void prepare_path_planning();
	void load_plots();
	void add_intermissions();

	void make_destination_map_to_minimize_moving_distances(const int& currPlotID);
	void make_distance_array(const int& currPlotID, vector<vector<double>>& distanceArray) const;
	void make_manhatan_distance_array(const int& currPlotID, vector<vector<double>>& distanceArray) const;

	void generate_destination_assign_map(vector<int>& destinationAssignMap);
	void adjust_loiter_time_for_initial_plot();

	void load_takeoff_cluster_info_file(const string& fileName);

	void load_festa_input_format(const string& filePath);
	FESTA_COMMAND classify_festa_command(const string& command);
	void translate_plot_strings(const list<string>& plotStrings, Plot& plot);
	SWITCHING_PLAN_TYPE classify_switching_plan_type(const string& command);
	void translate_take_off_position_strings(const list<string>& plotStrings, vector<rg_Point3D>& takeoffPositions);

	void generate_flight_plans_for_drone_swarm();

	void write_JSON_plan(const string& planFileName);

private:
	Ui::SwarmControllerClass ui;

public slots:
	void load_festa_setting_file();
	void load_flight_setting_file();
	void play_simulation();
	void increase_simulation_time();
	void print_data();
	void show_option_changed();
	void priority_type_changed();

	void generate_festa_flight_plans();

	void save_way_points_for_plot_change();
	void sync_computation_coord_to_world_coord(const float& arenaFloorAlt, const float& screenTiltAngle);
	
	float find_min_Z();

	void convert_way_points(vector<vector<WayPoint>>& convertedWPsForDrones, const float& arenaFloorAlt, const float& screenTiltAngle);
	rg_Point3D convert_computation_coord_to_university_coord(const rg_Point3D& point, const float& minZ);
	rg_Point3D convert_computation_coord_to_world_coord(const rg_Point3D& point, const float& minZ, const float& arenaRotationAngle);

	void write_mission_file(const string& fileName);

	void write_converted_plot_files();
	void collect_plan_items_for_plots(vector<vector<PlanItem>>& planItemsForPlots);
	void write_converted_plot_file(const string& fileName, vector<PlanItem>& planItemsForPlot);


	array<float, 2> translate_coord_to_gps(const float& x, const float& y) const;
	array<float, 2> translate_gps_to_coord(const float& latitude, const float& longitude) const;

	//the formulation belows are based on WGS84 and refered from https://en.wikipedia.org/wiki/Geographic_coordinate_system
	inline double meter_per_latitude() const { return 111132.92 - 559.82*cos(2 * m_homeLat) + 1.175*cos(4 * m_homeLat) - 0.0023*cos(6 * m_homeLat); }
	inline double meter_per_longitude() const { return 111412.84*cos(m_homeLat) - 93.5*cos(3 * m_homeLat) + 0.118*cos(5 * m_homeLat); }

	void write_QGC_plan_files();

	void    start_QGC_Plan_file(ofstream& fout);
	void    start_mission(ofstream& fout);
	void    write_mission_items(ofstream& fout, const vector<PlanItem>& planItems);
	void    end_mission(ofstream& fout, const vector<PlanItem>& planItems);
	void    end_QGC_Plan_file(ofstream& fout);


	void    write_QGC_dummy_plan_files();
	void write_dummy_mission_items(ofstream& fout, const vector<PlanItem*>& planItems);
	void    end_dummy_mission(ofstream& fout, const vector<PlanItem*>& planItems);
};
