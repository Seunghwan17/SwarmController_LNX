#include "SwarmController.h"
#include <chrono>
#include <fstream>
#include "Hungarian.h"
#include <random>
#include <ctime>
#include "constForVDRCOpenGLWidget.h"

SwarmController::SwarmController(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(increase_simulation_time()));
	timer->stop();

	ui.swarmDisplayer->set_eye_distance(200);
	//ui.swarmDisplayer->m_bShowDVD = true;
	ui.swarmDisplayer->m_bShowConjunctionCircle = true;
	ui.swarmDisplayer->update();

	show_option_changed();

	showMaximized();
}



void SwarmController::prepare_path_planning()
{
	Clock::time_point begin = Clock::now();

	m_simulator.set_avoidance_altitude(m_avoidanceAlt);
	m_simulator.set_collision_radius(m_collisionRadius);
	m_simulator.set_conjunction_radius(m_conjunctionRadius);

	priority_type_changed();

	//load_plots();

	for (int i = 0; i < m_plots.size() - 1; i++)
	{
		Plot& currPlot = m_plots.at(i);
		Plot& nextPlot = m_plots.at(i+1);

		switch (nextPlot.switchingPlanType)
		{
		case APRIORI_DEFINED_FLIGHT_PLAN:
		{
			for (int j = 0; j < m_numDrone; j++)
				currPlot.destinationAssignMap.push_back(j);
			break;
		}
		case MIN_TOTAL_FLIGHT_DISTANCES:
			make_destination_map_to_minimize_moving_distances(i);
			break;
		case UNKNOWN_SWITCHING_PLAN:
		default:
			break;
		}
	}
	add_intermissions();


	m_simulator.make_drone_balls(m_plots.at(0));
	m_simulator.add_bounding_balls(500, m_simulator.get_collision_radius(), m_numDrone + 1);
	cout << "Initialization finish" << endl;

	m_simulator.set_target_plot(m_plots.at(1));
	vector<int> destinationAssignMap;
	generate_destination_assign_map(destinationAssignMap);

	m_simulator.set_basic_speed(m_plots.at(1).switchingVelocity);
	m_simulator.initialize_scene_change(destinationAssignMap);

	for (int i = 0; i < m_numDrone; i++)
	{
		m_currPositionMap[destinationAssignMap.at(i)] = i;
	}

	m_simulator.prepare_simulation();
	Clock::time_point end = Clock::now();
	computationTime += chrono::duration_cast<chrono::milliseconds>(end - begin).count();

	for (int i = 0; i < m_numDrone; i++)
	{
		m_wayPointsForDrones.push_back(vector<WayPoint>());
	}
}


void SwarmController::load_plots()
{
	for (auto& plot : m_plots)
	{
		ifstream fin;
		fin.open(plot.plotFileName.c_str());

		char* seps = " \t\n";
		char buffer[200];

		while (fin.getline(buffer, 200))
		{
			if (buffer[0] != '#')
			{
				break;
			}
		};

		int numPoints = atoi(strtok(buffer, seps));

		for (int i = 0; i < numPoints; i++)
		{
			fin.getline(buffer, 200);
			int   id = atoi(strtok(buffer, seps));
			float  x = atof(strtok(NULL, seps));
			float  y = atof(strtok(NULL, seps));
			float  z = atof(strtok(NULL, seps));
			float priority = atof(strtok(NULL, seps));
			float  r = atof(strtok(NULL, seps));
			float  g = atof(strtok(NULL, seps));
			float  b = atof(strtok(NULL, seps));

			PlotObject currObject;
			float tiltY = y * cos(plot.tiltAngle);
			float tiltZ = y * sin(plot.tiltAngle) + z;

			currObject.point = rg_Point3D(x, tiltY, tiltZ);
			currObject.priority = priority;
			currObject.color = Color3f(r, g, b);
			plot.plotObjects.push_back(currObject);
		}
		fin.close();
	}

	for (int i = 0; i < m_plots.size() - 1; i++)
	{
		Plot& currPlot = m_plots.at(i);
		if(currPlot.switchingPlanType)
			make_destination_map_to_minimize_moving_distances(i);
		else
			for(int j=0; j<m_numDrone; j++)
				currPlot.destinationAssignMap.push_back(j);
	}
}



void SwarmController::add_intermissions()
{
	default_random_engine engine;
	engine.seed(time(0));
	uniform_real_distribution<float> dist{ 0, 1};

	int numOriginalPlots = m_plots.size();
	int currPlotID = 0;
	for (int i = 0; i < numOriginalPlots - 1; i++)
	{
		Plot& currPlot = m_plots.at(currPlotID);
		Plot& nextPlot = m_plots.at(currPlotID + 1);

		Plot intermission;
		intermission.isReal = false;
		intermission.plotWaitingTime = 0;
		intermission.switchingVelocity = nextPlot.switchingVelocity;

		for (int j = 0; j < currPlot.plotObjects.size(); j++)
		{
			PlotObject intermissionObject;
			rg_Point3D currPlotPoint = currPlot.plotObjects.at(j).point;
			rg_Point3D nextPlotPoint = nextPlot.plotObjects.at(currPlot.destinationAssignMap.at(j)).point;
			intermissionObject.point = rg_Point3D(currPlotPoint.getX(), currPlotPoint.getY(), nextPlotPoint.getZ());
			intermissionObject.color = currPlot.plotObjects.at(j).color;
			intermission.plotObjects.push_back(intermissionObject);
			intermission.destinationAssignMap.push_back(currPlot.destinationAssignMap.at(j));
			currPlot.destinationAssignMap.at(j) = j;
		}

		m_plots.insert(m_plots.begin() + currPlotID+1, intermission);
		currPlotID += 2;
	}
}



void SwarmController::make_destination_map_to_minimize_moving_distances(const int& currPlotID)
{
	vector<vector<double>> distanceArray;
	//make_distance_array(currPlotID, distanceArray);
	make_manhatan_distance_array(currPlotID, distanceArray);

	HungarianAlgorithm hungarian;
	vector<int> assignment;
	double cost = hungarian.Solve(distanceArray, assignment);
	//cout << "Cost after: " << cost << endl;

	Plot& currPlot = m_plots.at(currPlotID);
	for (int i = 0; i < assignment.size(); i++)
	{
		currPlot.destinationAssignMap.push_back(assignment.at(i));
	}

	cout << "Assign Result: " << cost << endl;
	for (int i = 0; i < assignment.size(); i++)
	{
		cout << "i " << i << " -> j " << currPlot.destinationAssignMap.at(i) << endl;
	}
}



void SwarmController::make_distance_array(const int& currPlotID, vector<vector<double>>& distanceArray) const
{
	const Plot& currPlot = m_plots.at(currPlotID);
	const Plot& targetPlot = m_plots.at(currPlotID+1);
	distanceArray.resize(m_numDrone);
	for (int i = 0; i < m_numDrone; i++)
		distanceArray.at(i) = vector<double>(m_numDrone);

	int index_initial = 0;
	int index_target = 0;
	for (int i = 0; i < m_numDrone; i++)
	{
		for (int j = 0; j < m_numDrone; j++)
		{
			distanceArray.at(i).at(j) = currPlot.plotObjects.at(i).point.distance(targetPlot.plotObjects.at(j).point);
		}
	}
}



void SwarmController::make_manhatan_distance_array(const int& currPlotID, vector<vector<double>>& distanceArray) const
{
	const Plot& currPlot = m_plots.at(currPlotID);
	const Plot& targetPlot = m_plots.at(currPlotID + 1);
	distanceArray.resize(m_numDrone);
	for (int i = 0; i < m_numDrone; i++)
		distanceArray.at(i) = vector<double>(m_numDrone);

	int index_initial = 0;
	int index_target = 0;
	for (int i = 0; i < m_numDrone; i++)
	{
		for (int j = 0; j < m_numDrone; j++)
		{
			float zDiff = abs(currPlot.plotObjects.at(i).point.getZ() - targetPlot.plotObjects.at(j).point.getZ());
			float xyDist = sqrt(pow(targetPlot.plotObjects.at(j).point.getX() - currPlot.plotObjects.at(i).point.getX(), 2) + pow(targetPlot.plotObjects.at(j).point.getY() - currPlot.plotObjects.at(i).point.getY(), 2));

			distanceArray.at(i).at(j) = xyDist + zDiff;
		}
	}
}



void SwarmController::generate_destination_assign_map(vector<int>& destinationAssignMap)
{
	destinationAssignMap.resize(m_numDrone);
	vector<int>& originalDestinationMap = m_plots.at(m_currSceneID).destinationAssignMap;
	for (int i = 0; i < m_numDrone; i++)
	{
		destinationAssignMap.at(m_currPositionMap.at(i)) = originalDestinationMap.at(i);
	}
}



void SwarmController::adjust_loiter_time_for_initial_plot()
{
	map<int, float> mapFromIDToInitPlotArrivalTime;
	float maxArrivalTime = 0;
	for (auto& planItemForDrone : m_planItemsForDrones)
	{
		PlanItem& initialPlanItem = planItemForDrone.front();
		float alt = initialPlanItem.point.getZ();
		int takeoffCluster = m_mapFromIDToTakeoffCluster.at(initialPlanItem.droneID);
		float arrivalTime = (takeoffCluster - 1)*m_timeIntervalBetweenTakeoffCluster + alt / TAKEOFF_SPEED;
		mapFromIDToInitPlotArrivalTime[initialPlanItem.droneID] = arrivalTime;
		if (arrivalTime > maxArrivalTime)
			maxArrivalTime = arrivalTime;
	}

	for (auto& planItemForDrone : m_planItemsForDrones)
	{
		PlanItem& initialPlanItem = planItemForDrone.front();
		float loiterTime = maxArrivalTime - mapFromIDToInitPlotArrivalTime.at(initialPlanItem.droneID) + m_loiterTime;
		initialPlanItem.loiterTime = loiterTime;
	}
}



void SwarmController::load_takeoff_cluster_info_file(const string& fileName)
{
	ifstream TOCFile(fileName);
	string line;
	while (getline(TOCFile, line))
	{
		istringstream iss(line);
		string token;
		iss >> token;
		int ID = stoi(token);
		iss >> token;
		int takeoffCluster = stoi(token);
		m_mapFromIDToTakeoffCluster[ID] = takeoffCluster;
	}
	TOCFile.close();
}



void SwarmController::load_festa_input_format(const string& filePath)
{
	ifstream festaInput(filePath);
	festaInput.precision(10);

	list<string> takeoffPositionStrings;
	list<string> landPositionStrings;
	list<list<string>> plotStringSets;

	if (festaInput.is_open())
	{
		m_simulator.clear();
		string line;
		while (getline(festaInput, line))
		{
			istringstream iss(line);
			string token;
			iss >> token;

			FESTA_COMMAND command = classify_festa_command(token);
			switch (command)
			{
			case NUMBER_OF_DRONES:
				iss >> token;
				m_numDrone = stoi(token);
				cout << "Num drone : " << m_numDrone << endl;
				break;
			case CONJUNCT_RADIUS:
				iss >> token;
				m_conjunctionRadius = stod(token);
				cout << "Conjunction radius : " << m_conjunctionRadius << endl;
				break;
			case COLLISION_RADIUS:
				iss >> token;
				m_collisionRadius = stod(token);
				cout << "Collision radius : " << m_collisionRadius << endl;
				break;
			case COORDINATE_ORIGIN_LATITUDE:
				iss >> token;
				m_homeLat = stod(token);
				cout << "Home lat: " << m_homeLat << endl;
				break;
			case COORDINATE_ORIGIN_LONGITUDE:
				iss >> token;
				m_homeLon = stod(token);
				cout << "Home lon: " << m_homeLon << endl;
				break;
			case COORDINATE_ROTATION_ANGLE:
				iss >> token;
				m_arenaRotationAngle = stod(token);
				cout << "Home angle: " << m_arenaRotationAngle << endl;
				break;
			case NUMBER_OF_SCENES:
				iss >> token;
				m_numPlots = stoi(token);
				cout << "Num plots: " << m_numPlots << endl;
				break;
			case TAKEOFF_AND_PREPARE_TO_FLY:
			case SCENE:
			case PREPARE_TO_LAND:
			{
				plotStringSets.push_back(list<string>());
				list<string>& strings = plotStringSets.back();
				while (line.size() > 0)
				{
					strings.push_back(line);
					getline(festaInput, line);
				}
			}
			break;
			case HOME_POSITION:
			{
				while (line.size() > 0)
				{
					takeoffPositionStrings.push_back(line);
					getline(festaInput, line);
				}
			}
			break;
			case LAND:
			{
				while (line.size() > 0)
				{
					landPositionStrings.push_back(line);
					getline(festaInput, line);
				}
			}
			break;
			case COMMENT_SIGN:
				//Comments
				break;
			case UNKNOWN_COMMAND:
			default:
				cout << "Unknown commands: " << line << endl;
				break;
			}
		}
		cout << "Input read finish" << endl;
		festaInput.close();

		translate_take_off_position_strings(takeoffPositionStrings, m_takeoffPositions);

		for (auto& plotStrings : plotStringSets)
		{
			Plot currPlot;
			translate_plot_strings(plotStrings, currPlot);
			currPlot.tiltAngle = 0;
			m_plots.push_back(currPlot);
		}

		for (int i = 0; i < m_numDrone; i++)
		{
			m_currPositionMap[i] = i;
		}

		prepare_path_planning();

		ui.swarmDisplayer->pSimulator = &m_simulator;
	}
	else
		cout << "Fail to read port file" << endl;
}



FESTA_COMMAND SwarmController::classify_festa_command(const string& command)
{
	if (command.compare("%") == 0)
		return COMMENT_SIGN;
	if (command.compare("NUMBER_OF_DRONES") == 0)
		return NUMBER_OF_DRONES;
	if (command.compare("CONJUNCT_RADIUS") == 0)
		return CONJUNCT_RADIUS;
	if (command.compare("COLLISION_RADIUS") == 0)
		return COLLISION_RADIUS;

	if (command.compare("COORDINATE_ORIGIN_LATITUDE") == 0)
		return COORDINATE_ORIGIN_LATITUDE;
	if (command.compare("COORDINATE_ORIGIN_LONGITUDE") == 0)
		return COORDINATE_ORIGIN_LONGITUDE;
	if (command.compare("COORDINATE_ROTATION_ANGLE") == 0)
		return COORDINATE_ROTATION_ANGLE;

	if (command.compare("NUMBER_OF_SCENES") == 0)
		return NUMBER_OF_SCENES;

	if (command.compare("HOME_POSITION") == 0)
		return HOME_POSITION;
	if (command.compare("TAKEOFF_AND_PREPARE_TO_FLY") == 0)
		return TAKEOFF_AND_PREPARE_TO_FLY;
	if (command.compare("SCENE") == 0)
		return SCENE;
	if (command.compare("PREPARE_TO_LAND") == 0)
		return PREPARE_TO_LAND;
	if (command.compare("LAND") == 0)
		return LAND;

	return UNKNOWN_COMMAND;
}



void SwarmController::translate_plot_strings(const list<string>& plotStrings, Plot& plot)
{
	random_device rnd;
	uniform_real_distribution<float> perturbation(-1E-3, 1E-3);

	int lineIndex = 0;

	for (auto& str : plotStrings)
	{
		istringstream iss(str);
		string token;
		iss >> token;

		if (token.compare("%") == 0)
			continue;

		switch (lineIndex)
		{
		case 0:	//Header
			plot.plotFileName = str;	//temp
			break;
		case 1:	//Loiter time
			iss >> token;
			plot.plotWaitingTime = stoi(token);
			break;
		case 2: //Minimize switching distance
			iss >> token;
			plot.switchingPlanType = classify_switching_plan_type(token);
			break;
		case 3: //Switching velocity
			iss >> token;
			plot.switchingVelocity = stof(token);
			break;
		default: //Position
		{
			float x = stof(token) + perturbation(rnd);
			iss >> token;
			float y = stof(token) + perturbation(rnd);
			iss >> token;
			float z = stof(token) + perturbation(rnd);

			PlotObject object;
			object.point = rg_Point3D(x, y, z);
			object.priority = 1;
			object.color = Color3f(0, 0, 0);
			plot.plotObjects.push_back(object);
			break;
		}
		}		
		lineIndex++;
	}	
}



SWITCHING_PLAN_TYPE SwarmController::classify_switching_plan_type(const string& command)
{
	if (command.compare("APRIORI_DEFINED_FLIGHT_PLAN") == 0)
		return APRIORI_DEFINED_FLIGHT_PLAN;
	if (command.compare("MIN_TOTAL_FLIGHT_DISTANCES") == 0)
		return MIN_TOTAL_FLIGHT_DISTANCES;
	else
		return UNKNOWN_SWITCHING_PLAN;
}



void SwarmController::translate_take_off_position_strings(const list<string>& plotStrings, vector<rg_Point3D>& takeoffPositions)
{
	int lineIndex = 0;

	for (auto& str : plotStrings)
	{
		if (lineIndex >= 2)
		{
			cout << "Interpreted str: " << str <<endl;

			istringstream iss(str);
			string token;
			iss >> token;

			int ID = stoi(token);
			iss >> token;
			float x = stof(token);
			iss >> token;
			float y = stof(token);
			iss >> token;
			float z = stof(token);
			takeoffPositions.push_back(rg_Point3D(x, y, z));

			if (lineIndex == m_numDrone + 2)
				break;
		}

		lineIndex++;
	}
}



void SwarmController::generate_flight_plans_for_drone_swarm()
{
	Clock::time_point begin = Clock::now();

	bool doesAllPlanGenerated = false;
	while (doesAllPlanGenerated == false)
	{
		m_simulator.increase_time(INCREASE_TIME_STEP);
		m_currTime += INCREASE_TIME_STEP;

		if (m_simulator.is_every_ball_arrive())
		{
			m_currSceneID++;
			m_currTime += m_plots.at(m_currSceneID).plotWaitingTime;
			if (m_currSceneID < m_plots.size()-1)
			{
				Clock::time_point end = Clock::now();
				computationTime = chrono::duration_cast<chrono::milliseconds>(end - begin).count();

				cout << "Computation time: " << computationTime << endl;
				save_way_points_for_plot_change();
				m_simulator.clear();

				m_simulator.set_target_plot(m_plots.at(m_currSceneID + 1));
				vector<int> destinationAssignMap;
				generate_destination_assign_map(destinationAssignMap);

				m_simulator.set_basic_speed(m_plots.at(m_currSceneID+1).switchingVelocity);
				m_simulator.initialize_scene_change(destinationAssignMap);

				for (int i = 0; i < m_numDrone; i++)
				{
					m_currPositionMap[destinationAssignMap.at(i)] = i;
				}

				
				m_simulator.prepare_simulation();
			}
			else
			{
				doesAllPlanGenerated = true;
			}
		}
	}

	save_way_points_for_plot_change();
	cout << "Drone show finish - Computation time: " << computationTime << endl;

	cout << "Sync computation coord" << endl;
	sync_computation_coord_to_world_coord(0, 0);

	cout << "Write JSON file" << endl;
	write_JSON_plan(m_planFileName);
}



void SwarmController::write_JSON_plan(const string& planFileName)
{
	string directory = "JSONMissionFiles\\";

	for (auto& planItemsForDrone : m_planItemsForDrones)
	{
		int droneID = planItemsForDrone.front().droneID;
		string planFilePath = directory + planFileName + string("_") + to_string(droneID) + string(".plan");

		rg_Point3D takeOffPosition = m_takeoffPositions.at(droneID-1);

		PlanJsonWriter planWriter(m_homeLat, m_homeLon);
		planWriter.tranlate_plan_items_to_file(planFilePath, takeOffPosition, planItemsForDrone);
	}
}



void SwarmController::load_festa_setting_file()
{
	QString QfilePath = QFileDialog::getOpenFileName(this, tr("Open FESTA Setting File"), NULL, tr("Text file(*.txt)"));
	QFileInfo fileInfo(QfilePath);

	m_numDrone = 0;
	string filePath = translate_to_window_path(QfilePath);
	load_festa_input_format(filePath);

	ui.swarmDisplayer->pSimulator = &m_simulator;
}



void SwarmController::load_flight_setting_file()
{
	QString QfilePath = QFileDialog::getOpenFileName(this, tr("Open Flight Setting File"), NULL, tr("Text file(*.txt)"));
	QFileInfo fileInfo(QfilePath);

	m_numDrone = 0;
	string takeoffClusterInfoFileName;

	string filePath = translate_to_window_path(QfilePath);
	ifstream flightSettingFile(filePath.data());
	if (flightSettingFile.is_open()) {
		m_simulator.clear();
		string line;
		while (getline(flightSettingFile, line))
		{
			istringstream iss(line);
			string token;
			iss >> token;
			if(token.compare(string("#")) == 0)
				continue;

			if (m_numDrone == 0) //first line
			{
				m_numDrone = stoi(token);
				iss >> token;
				m_collisionRadius = stof(token);
				iss >> token;
				m_conjunctionRadius = stof(token);
				iss >> token;
				m_avoidanceAlt = stof(token);
				iss >> token;
				m_loiterTime = stof(token);
				iss >> token;
				m_homeLat = stof(token);
				iss >> token;
				m_homeLon = stof(token);
				iss >> token;
				m_arenaFloorAlt = stof(token);
				iss >> token;
				m_arenaRotationAngle = stof(token) / 180 * M_PI;
				iss >> token;
				m_screenTiltAngle = stof(token)/180*M_PI;
				iss >> token;
				m_planFileName = token;
				iss >> token;
				takeoffClusterInfoFileName = token;
				iss >> token;
				m_timeIntervalBetweenTakeoffCluster = stof(token);
				
			}
			else //plot files
			{
				m_plots.push_back(Plot());
				m_plots.back().plotFileName = token;
				m_plots.back().tiltAngle = m_screenTiltAngle;
				m_plots.back().plotWaitingTime = m_loiterTime;
				iss >> token;
				int useMinimalPath = stoi(token);
				if (useMinimalPath == 0)
					m_plots.back().switchingPlanType = APRIORI_DEFINED_FLIGHT_PLAN;
				else
					m_plots.back().switchingPlanType = MIN_TOTAL_FLIGHT_DISTANCES;
			}
		}
		cout << "Flight setting read finish" << endl;
		flightSettingFile.close();

		for (int i = 0; i < m_numDrone; i++)
		{
			m_currPositionMap[i] = i;
		}

		load_takeoff_cluster_info_file(takeoffClusterInfoFileName);
		prepare_path_planning();

		ui.swarmDisplayer->pSimulator = &m_simulator;
	}
	else
		cout << "Fail to read port file" << endl;
}



void SwarmController::play_simulation()
{
	if (timer->isActive())
		timer->stop();
	else
		timer->start(100);
}



void SwarmController::increase_simulation_time()
{
	Clock::time_point begin = Clock::now();

	/*m_simulator.increase_time_using_history(0.1);
	if(m_simulator.get_time_for_simulation() >= ARRIVAL_TIME)
		timer->stop();*/

	m_simulator.increase_time(0.1);
	m_currTime += 0.1;
	if (m_simulator.is_every_ball_arrive() )
	{
		m_currSceneID++;
		m_currTime += m_plots.at(m_currSceneID).plotWaitingTime;
		if (m_currSceneID < m_plots.size()-1)
		{
			cout << "Computation time: " << computationTime << endl;
			save_way_points_for_plot_change();
			m_simulator.clear();

			m_simulator.set_target_plot(m_plots.at(m_currSceneID+1));
			vector<int> destinationAssignMap;

			generate_destination_assign_map(destinationAssignMap);

			m_simulator.set_basic_speed(m_plots.at(m_currSceneID+1).switchingVelocity);
			m_simulator.initialize_scene_change(destinationAssignMap);

			for (int i = 0; i < m_numDrone; i++)
			{
				m_currPositionMap[destinationAssignMap.at(i)] = i;
			}

			m_simulator.prepare_simulation();
		}
		else
		{
			timer->stop();
			save_way_points_for_plot_change();
			cout << "Drone show finish - Computation time: " << computationTime << endl;

			cout << "Sync computation coord" << endl;
			sync_computation_coord_to_world_coord( m_arenaFloorAlt, m_screenTiltAngle);

			cout << "Write JSON file" << endl;
			write_JSON_plan(m_planFileName);

			/*adjust_loiter_time_for_initial_plot();
			cout << "Write mission file" << endl;
			write_mission_file(m_planFileName);
			cout << "Write plot files" << endl;
			write_converted_plot_files();
			cout << "Write QGC plan files" << endl;
			write_QGC_plan_files();
			cout << "Write Dummy QGC plan files" << endl;
			write_QGC_dummy_plan_files();*/

			const vector<DynamicEvent3D*>& DVDEvents = m_simulator.get_dynamic_VD().get_event_history();
			array<int, 3> eventCounter = { 0, 0, 0 };
			for (auto& event : DVDEvents)
			{
				switch (event->get_event_type())
				{
				case FACE_FLIP:
					eventCounter.at(0)++;
					break;
				case EDGE_FLIP:
					eventCounter.at(1)++;
					break;
				case VELOCITY_CHANGE:
					eventCounter.at(2)++;
					break;
				}
			}
			cout << "Events: " << eventCounter.at(0) << ", " << eventCounter.at(1) << ", " << eventCounter.at(2) << endl;
		}	
	}
		

	Clock::time_point end = Clock::now();
	computationTime += chrono::duration_cast<chrono::milliseconds>(end - begin).count();

	ui.label_curr_time->setText(QString::number(m_currTime));

	/*if (m_simulator.get_drones_on_evasion().empty() == false)
	{
		DroneBall* dronesOnEvasion = m_simulator.get_drones_on_evasion().front();
		rg_Point3D position = dronesOnEvasion->calculate_position_at_time(m_simulator.get_dynamic_VD().get_current_time());
		evasionView->set_local_origin(position.getX(), position.getY(), position.getZ());
		evasionView->update();
		//evasionView->show();
	}
	else
	{
		evasionView->hide();
	}*/

	ui.swarmDisplayer->update();
}



void SwarmController::print_data()
{
	int currTime = m_simulator.get_dynamic_VD().get_current_time();
	string fileName = string("data_") + to_string(currTime) + string(".dat");
	m_simulator.print_drone_info(fileName);
}



void SwarmController::show_option_changed()
{
	ui.swarmDisplayer->m_bShowTrajectory = ui.checkBox_show_trajectory->isChecked();
	ui.swarmDisplayer->m_bShowConjunctionCircle = ui.checkBox_show_conjunction_volume->isChecked();

	if (ui.radioButton_view_xy_plane->isChecked())
		ui.swarmDisplayer->set_eye_direction(rg_Point3D(0, 0, 1));

	if (ui.radioButton_view_xz_plane->isChecked())
		ui.swarmDisplayer->set_eye_direction(rg_Point3D(0, -1, 0));

	if (ui.radioButton_view_yz_plane->isChecked())
		ui.swarmDisplayer->set_eye_direction(rg_Point3D(1, 0, 0));

	ui.swarmDisplayer->update();
}



void SwarmController::priority_type_changed()
{
	if (ui.radioButton_priority_identical->isChecked())
		m_simulator.set_priority_type(PRIORITY_IDENTICAL);

	if (ui.radioButton_priority_given->isChecked())
		m_simulator.set_priority_type(PRIORITY_GIVEN);

	if (ui.radioButton_priority_distance->isChecked())
		m_simulator.set_priority_type(PRIORITY_DISTANCE);

	m_simulator.assign_priority();
}



void SwarmController::generate_festa_flight_plans()
{
	generate_flight_plans_for_drone_swarm();
}



void SwarmController::save_way_points_for_plot_change()
{
	float plotTransitionTime = 0;
	vector<DroneBall>& drones = m_simulator.get_drones();
	for (auto& drone : drones)
	{
		vector<WayPoint>& wayPoints = drone.get_way_points();

		if (plotTransitionTime < wayPoints.back().time)
			plotTransitionTime = wayPoints.back().time;

		vector<WayPoint>& wayPointsForDrone = m_wayPointsForDrones.at(drone.get_ID()-1);
		for (int i = 0; i < wayPoints.size(); i++)
		{
			wayPointsForDrone.push_back(wayPoints.at(i));
		}
	}

	m_plots.at(m_currSceneID).plotTransitionTime = plotTransitionTime;
}



void SwarmController::sync_computation_coord_to_world_coord(const float& arenaFloorAlt, const float& screenTiltAngle)
{
	vector<vector<WayPoint>> convertedWPsForDrones;
	convert_way_points(convertedWPsForDrones, arenaFloorAlt, screenTiltAngle);

	for (int i = 0; i<convertedWPsForDrones.size(); i++ )
	{
		vector<WayPoint>& convertedWPsForDrone = convertedWPsForDrones.at(i);
		m_planItemsForDrones.push_back(vector<PlanItem>());
		vector<PlanItem>& planItemsForDrone = m_planItemsForDrones.back();
		
		int sceneID = 0;

		PlanItem itemForInitLocation;
		WayPoint& WPForInitLocation = convertedWPsForDrone.front();
		itemForInitLocation.droneID = i + 1;
		itemForInitLocation.point = WPForInitLocation.point;
		itemForInitLocation.speed = m_plots.front().switchingVelocity; //TO BE UPDATED
		itemForInitLocation.type = INIT_LOC;
		itemForInitLocation.loiterTime = m_plots.front().plotWaitingTime;
		itemForInitLocation.color = WPForInitLocation.color;
		planItemsForDrone.push_back(itemForInitLocation);
		sceneID++;

		for (int j = 1; j < convertedWPsForDrone.size() - 1; j++)
		{
			PlanItem currItem;
			WayPoint& currWP = convertedWPsForDrone.at(j);
			currItem.droneID = i + 1;
			currItem.point = currWP.point;
			currItem.speed = m_plots.at(sceneID).switchingVelocity;	//TO BE UPDATED
			currItem.type = currWP.type;
			currItem.loiterTime = 0;
			if (currWP.type == PAUSE)
			{
				WayPoint& nextWP = convertedWPsForDrone.at(j+1);

				if (nextWP.type == AVOIDANCE_START)
				{
					currItem.loiterTime = nextWP.time - currWP.time;
					currItem.speed = AVOIDANCE_SPEED;
				}
			}
			if (currWP.type == AVOIDANCE_START)
			{
				currItem.speed = AVOIDANCE_SPEED;
			}
			if (currWP.type == DESTINATION)
			{
				float plotTransitionTime = m_plots.at(sceneID).plotTransitionTime;
				currItem.loiterTime = plotTransitionTime - currWP.time + m_plots.at(sceneID).plotWaitingTime;
				sceneID++;
			}
			currItem.color = currWP.color;
			planItemsForDrone.push_back(currItem);
		}

		PlanItem itemForFinalDestination;
		WayPoint& WPForFinalDestination = convertedWPsForDrone.back();
		itemForFinalDestination.droneID = i + 1;
		itemForFinalDestination.point = WPForFinalDestination.point;
		itemForFinalDestination.speed = 0;	//TO BE UPDATED
		itemForFinalDestination.type = DESTINATION;

		float plotTransitionTime = m_plots.at(sceneID).plotTransitionTime;
		itemForFinalDestination.loiterTime = plotTransitionTime - WPForFinalDestination.time + m_plots.back().plotWaitingTime;
		itemForFinalDestination.color = WPForFinalDestination.color;
		planItemsForDrone.push_back(itemForFinalDestination);
	}
}



float SwarmController::find_min_Z()
{
	float minZ = FLT_MAX;
	for (auto& wayPointsForDrone : m_wayPointsForDrones)
	{
		for (auto& wayPoint : wayPointsForDrone)
		{
			if (wayPoint.point.getZ() < minZ)
				minZ = wayPoint.point.getZ();
		}
	}
	return minZ;
}



void SwarmController::convert_way_points(vector<vector<WayPoint>>& convertedWPsForDrones, const float& arenaFloorAlt, const float& screenTiltAngle)
{
	float minZ = find_min_Z();
	for (auto& wayPointsForDrone : m_wayPointsForDrones)
	{
		convertedWPsForDrones.push_back(vector<WayPoint>());
		vector<WayPoint>& convertedWPsForDrone = convertedWPsForDrones.back();

		for (auto& wayPoint : wayPointsForDrone)
		{
			if(wayPoint.type == INIT_LOC && !convertedWPsForDrone.empty())
				continue;

			rg_Point3D pointAtUnivCoord = convert_computation_coord_to_world_coord(wayPoint.point, minZ, m_arenaRotationAngle);
			WayPoint convertedWP = wayPoint;
			convertedWP.point = pointAtUnivCoord;
			convertedWPsForDrone.push_back(convertedWP);
		}
	}
}



rg_Point3D SwarmController::convert_computation_coord_to_university_coord(const rg_Point3D& point, const float& minZ)
{
	float zAdd = 0;
	//float zAdd = m_arenaFloorAlt - minZ;

	float floppedX = -point.getX();
	float floppedY = -point.getY();

	float rotatedX = floppedX * cos(ROTATION_FOR_UNIVERSITY_STADIUM) + floppedY *sin(ROTATION_FOR_UNIVERSITY_STADIUM);
	float rotatedY = floppedX * -sin(ROTATION_FOR_UNIVERSITY_STADIUM) + floppedY * cos(ROTATION_FOR_UNIVERSITY_STADIUM);
	return rg_Point3D(rotatedX, rotatedY, point.getZ()+ zAdd);
}



rg_Point3D SwarmController::convert_computation_coord_to_world_coord(const rg_Point3D& point, const float& minZ, const float& arenaRotationAngle)
{
	float zAdd = 0;
	//float zAdd = m_arenaFloorAlt - minZ;

	float rotatedX = -point.getX() * cos(arenaRotationAngle) + point.getY() * sin(arenaRotationAngle);
	float rotatedY = -point.getX() * -sin(arenaRotationAngle) + point.getY() * cos(arenaRotationAngle);
	return rg_Point3D(rotatedX, rotatedY, point.getZ() + zAdd);
}



void SwarmController::write_mission_file(const string& fileName)
{
	ofstream missionFile(fileName);
	missionFile << m_planItemsForDrones.size() << " " << m_homeLat << " " << m_homeLon << "\n";

	for (auto& planItemsForDrone : m_planItemsForDrones)
	{
		int droneID = planItemsForDrone.front().droneID;
		int plotID = 0;
		for (auto& planItem : planItemsForDrone)
		{
			string tag;
			switch (planItem.type)
			{
			case INIT_LOC:
				tag = "Init";
				break;
			case PAUSE:
				tag = "Pause";
				break;
			case DESTINATION:
				tag = "Dest";
				plotID++;
				break;
			case AVOIDANCE_START:
				tag = "AS";
				break;
			case AVOIDANCE_END:
				tag = "AE";
				break;
			case ROLE_CHANGE:
			default:
				tag = "None";
				break;
			}

			missionFile << droneID << " " << planItem.point.getX() << " " << planItem.point.getY() << " " << planItem.point.getZ() << " " << planItem.speed << " " << planItem.loiterTime << " "<< planItem.color.getR() << " " << planItem.color.getG() << " " << planItem.color.getB() << " " <<tag<<" "<< plotID<<"\n";
		}
		droneID++;
	}
	missionFile.close();
}



void SwarmController::write_converted_plot_files()
{
	vector<vector<PlanItem>> planItemsForPlots;
	collect_plan_items_for_plots(planItemsForPlots);

	for (int i = 0; i < planItemsForPlots.size(); i++)
	{
		string fileName = string("convertedPlot_") + to_string(i + 1) + string(".txt");
		write_converted_plot_file(fileName, planItemsForPlots.at(i));
	}
}



void SwarmController::collect_plan_items_for_plots(vector<vector<PlanItem>>& planItemsForPlots)
{
	int droneID = 0;
	for (auto& planItemsForDrone : m_planItemsForDrones)
	{
		int sceneID = 0;
		for (auto& planItem : planItemsForDrone)
		{
			if (planItem.type == INIT_LOC || planItem.type == DESTINATION)
			{
				if (planItemsForPlots.size() < sceneID + 1)
				{
					planItemsForPlots.push_back(vector<PlanItem>());
					planItemsForPlots.back().resize(m_numDrone);
				}
					
				vector<PlanItem>& planItemForPlot = planItemsForPlots.at(sceneID);
				planItemForPlot.at(droneID) = planItem;
				sceneID++;
			}
		}
		droneID++;
	}

	cout << "Plan items for plots are collected: " << planItemsForPlots.size() << endl;
	for (int i = 0; i < planItemsForPlots.size(); i++)
	{
		cout << "Plot " << i << " collects " << planItemsForPlots.at(i).size() << endl;
	}
}



void SwarmController::write_converted_plot_file(const string& fileName, vector<PlanItem>& planItemsForPlot)
{
	ofstream planItemFile(fileName);
	planItemFile << "ID \t X \t Y \t Z \t Speed \t LoiterTime \n";
	int droneID = 0;
	for (auto& planItem : planItemsForPlot)
	{
		planItemFile << droneID << "\t" << planItem.point.getX() << "\t" << planItem.point.getY() << "\t" << planItem.point.getZ() << "\t" << planItem.speed << "\t" << planItem.loiterTime << "\n";
		droneID++;
	}
	planItemFile.close();
}



array<float, 2> SwarmController::translate_coord_to_gps(const float& x, const float& y) const
{
	float latitude = m_homeLat + (y / meter_per_latitude());
	float longitude = m_homeLon + (x / meter_per_longitude());
	return { latitude, longitude };
}

array<float, 2> SwarmController::translate_gps_to_coord(const float& latitude, const float& longitude) const
{
	float x = (longitude - m_homeLon)*meter_per_longitude();
	float y = (latitude - m_homeLat)*meter_per_latitude();
	return { x, y };
}



void SwarmController::write_QGC_plan_files()
{
	string directory = "QGCMissionFiles\\";

	for (auto& planItemsForDrone : m_planItemsForDrones)
	{
		int droneID = planItemsForDrone.front().droneID;

		ofstream planFile(directory+string("mission") + to_string(droneID) + string(".plan"));
		planFile << planItemsForDrone.size()*2+2 << "\n";
		planFile << "#seq \t frame \t command \t current \t auto \t p1 \t p2 \t p3 \t p4 \t x \t y \t z \t type\n";

		unsigned int itemID = 0;

		//////////////////////////////////////////////////////////////////////////
		// Phase 1: Move to show position
		//////////////////////////////////////////////////////////////////////////
		const PlanItem& planItemForInitPlot = planItemsForDrone.front();
		array<float, 2> GPSCoordForInitPoint = translate_coord_to_gps(planItemForInitPlot.point.getX(), planItemForInitPlot.point.getY());
		planFile.precision(10);
		/*planFile << itemID << "\t 3 \t 22 \t 1 \t 1 \t 0 \t 0 \t 0 \t nan \t" << GPSCoordForInitPoint.at(0) << "\t" << GPSCoordForInitPoint.at(1) << "\t 5 \t 0 \n";
		itemID++;
		planFile << itemID << "\t 3 \t 16 \t 0 \t 1 \t 0 \t 0 \t 0 \t nan \t" << GPSCoordForInitPoint.at(0) << "\t" << GPSCoordForInitPoint.at(1) << "\t 5 \t 0 \n";
		itemID++;
		planFile << itemID << "\t 3 \t 21 \t 0 \t 0 \t 0 \t 0 \t 0 \t nan \t" << GPSCoordForInitPoint.at(0) << "\t" << GPSCoordForInitPoint.at(1) << "\t 0 \t 0 \n";
		itemID++;*/
		
		//  Takeoff from ground
		planFile << itemID << "\t 3 \t 22 \t 1 \t 1 \t 0 \t 0 \t 0 \t nan \t" << GPSCoordForInitPoint.at(0) << "\t" << GPSCoordForInitPoint.at(1) << "\t 5 \t 0 \n";
		itemID++;

		//////////////////////////////////////////////////////////////////////////
		// Phase 2: Gooooooo
		//////////////////////////////////////////////////////////////////////////

		for (int i = 0; i < planItemsForDrone.size(); i++)
		{
			const PlanItem& item = planItemsForDrone.at(i);
			array<float, 2> GPSCoord = translate_coord_to_gps(item.point.getX(), item.point.getY());
			
			if (i==0)
				planFile << itemID << "\t 3 \t 16 \t 0 \t 0 \t" << item.loiterTime << "\t 0 \t 0 \t nan \t" << GPSCoord.at(0) << "\t" << GPSCoord.at(1) << "\t" << item.point.getZ() << "\t 0 \n";
			else
				planFile << itemID << "\t 3 \t 16 \t 0 \t 1 \t" << item.loiterTime << "\t 0 \t 0 \t nan \t" << GPSCoord.at(0) << "\t" << GPSCoord.at(1) << "\t" << item.point.getZ() << "\t 0 \n";
			
			itemID++;

			if(i< planItemsForDrone.size()-1)
				planFile << itemID << "\t 2 \t 178 \t 0 \t 1 \t 1 \t " << item.speed << "\t -1 \t 0 \t 0 \t 0 \t 0 \t 0\n";
			else
				planFile << itemID << "\t 2 \t 178 \t 0 \t 1 \t 1 \t 1.5 \t -1 \t 0 \t 0 \t 0 \t 0 \t 0\n";
			itemID++;
		}

		//  Land at location
		const PlanItem& itemForLastPlot = planItemsForDrone.back();
		array<float, 2> GPSCoordForLastPoint = translate_coord_to_gps(itemForLastPlot.point.getX(), itemForLastPlot.point.getY());
		planFile << itemID << "\t 3 \t 21 \t 0 \t 1 \t 0 \t 0 \t 0 \t nan \t" << GPSCoordForLastPoint.at(0) << "\t" << GPSCoordForLastPoint.at(1) << "\t 0 \t 0 \n";

		planFile.close();
	}
}

void SwarmController::start_QGC_Plan_file(ofstream& fout)
{
	fout << "{" << endl;
	fout << "    \"fileType\": \"Plan\"," << endl;
	fout << "    \"geoFence\": {" << endl;
	fout << "    }," << endl;
	fout << "    \"groundStation\" : \"QGroundControl\"," << endl;
}

void SwarmController::start_mission(ofstream& fout)
{
	fout << "    \"mission\": {" << endl;
	fout << "        \"cruiseSpeed\" : 15," << endl;
	fout << "        \"firmwareType\" : 12," << endl;
	fout << "        \"hoverSpeed\" : 2," << endl;
}



void SwarmController::write_mission_items(ofstream& fout, const vector<PlanItem>& planItems)
{
	unsigned int itemID = 0;

	//////////////////////////////////////////////
	//  open mission items
	fout << "        \"items\": [" << endl;


	//  Takeoff from ground / hand
	const PlanItem& planItemForInitPlot = planItems.front();

	array<float, 2> GPSCoordForInitPoint = translate_coord_to_gps(planItemForInitPlot.point.getX(), planItemForInitPlot.point.getY());
	fout << "           {" << endl;
	fout << "               \"autoContinue\": true," << endl;
	fout << "               \"command\" : 22," << endl;
	fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
	fout << "               \"frame\" : 3," << endl;
	fout << "               \"params\" : [" << endl;
	fout << "                   0," << endl;
	fout << "                   1," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout.precision(7);
	fout << "                   " << fixed << GPSCoordForInitPoint.at(0) << "," << endl;
	fout << "                   " << fixed << GPSCoordForInitPoint.at(1) << "," << endl;
	fout << "                   " << fixed << planItemForInitPlot.point.getZ() << endl;
	fout << "               ]," << endl;
	fout << "               \"type\": \"SimpleItem\"" << endl;
	fout << "           }," << endl;


	for (int i = 0; i < planItems.size(); i++) 
	{
		const PlanItem& item = planItems.at(i);
		array<float, 2> GPSCoord = translate_coord_to_gps(item.point.getX(), item.point.getY());

		//  Navigate to waypoint.
		fout << "           {" << endl;
		fout << "               \"autoContinue\": true," << endl;
		fout << "               \"command\" : 16," << endl;
		fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
		fout << "               \"frame\" : 3," << endl;
		fout << "               \"params\" : [" << endl;
		fout << "                   " << item.loiterTime <<"," << endl;
		fout << "                   1," << endl;
		fout << "                   0," << endl;
		fout << "                   0," << endl;
		fout.precision(7);
		fout << "                   " << fixed << GPSCoord.at(0) << "," << endl;
		fout << "                   " << fixed << GPSCoord.at(1) << "," << endl;
		fout << "                   " << fixed << item.point.getZ() << endl;
		fout << "               ]," << endl;
		fout << "               \"type\": \"SimpleItem\"" << endl;
		fout << "           }," << endl;


		//  Change speed and/or throttle set points.
		fout << "           {" << endl;
		fout << "               \"autoContinue\": true," << endl;
		fout << "               \"command\" : 178," << endl;
		fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
		fout << "               \"frame\" : 2," << endl;
		fout << "               \"params\" : [" << endl;
		fout << "                   1," << endl;
		fout << "                   " << item.speed << "," << endl;
		fout << "                   -1," << endl;
		fout << "                   0," << endl;
		fout << "                   0," << endl;
		fout << "                   0," << endl;
		fout << "                   null" << endl;
		fout << "               ]," << endl;
		fout << "               \"type\": \"SimpleItem\"" << endl;
		fout << "           }," << endl;
	}


	//  Land at location
	const PlanItem& itemForLastPlot = planItems.back();
	array<float, 2> GPSCoordForLastPoint = translate_coord_to_gps(itemForLastPlot.point.getX(), itemForLastPlot.point.getY());

	fout << "           {" << endl;
	fout << "               \"autoContinue\": true," << endl;
	fout << "               \"command\" : 21," << endl;
	fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
	fout << "               \"frame\" : 3," << endl;
	fout << "               \"params\" : [" << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout.precision(7);
	fout << "                   " << GPSCoordForLastPoint.at(0) << "," << endl;
	fout << "                   " << GPSCoordForLastPoint.at(1) << "," << endl;
	fout << "                   -5" << endl;
	fout << "               ]," << endl;
	fout << "               \"type\": \"SimpleItem\"" << endl;
	fout << "           }" << endl;


	//////////////////////////////////////////////
	//  close mission items
	fout << "        ]," << endl;
}

void SwarmController::end_mission(ofstream& fout, const vector<PlanItem>& planItems)
{
	const PlanItem& planItemForInitPlot = planItems.front();
	array<float, 2> GPSCoordForInitPoint = translate_coord_to_gps(planItemForInitPlot.point.getX(), planItemForInitPlot.point.getY());

	fout << "        \"plannedHomePosition\" : [" << endl;
	fout.precision(7);
	fout << "            " << fixed << GPSCoordForInitPoint.at(0) << ", " << endl;
	fout << "            " << fixed << GPSCoordForInitPoint.at(1) << ", " << endl;
	fout << "            0.0" << endl;
	fout << "        ]," << endl;
	fout << "        \"vehicleType\" : 2," << endl;
	fout << "        \"version\": 2" << endl;
	fout << "    }," << endl;
}

void SwarmController::end_QGC_Plan_file(ofstream& fout)
{
	fout << "    \"rallyPoints\": {" << endl;
	fout << "        \"points\": [" << endl;
	fout << "        ]," << endl;
	fout << "        \"version\" : 1" << endl;
	fout << "    }," << endl;
	fout << "    \"version\" : 1" << endl;
	fout << "}" << endl;
}



void SwarmController::write_QGC_dummy_plan_files()
{
	string directory = "QGCDummyMissionFiles\\";
	vector<vector<PlanItem*>> dummyPlanItems;
	
	for (auto& planItemsForDrone : m_planItemsForDrones)
	{
		int plotNum = 0;
		for (auto& planItem : planItemsForDrone)
		{
			if (planItem.type == INIT_LOC || planItem.type == DESTINATION)
			{
				if (plotNum % 2 == 0)
				{
					int plotID = plotNum / 2;
					if (dummyPlanItems.size() == plotID)
						dummyPlanItems.push_back(vector<PlanItem*>());

					vector<PlanItem*>& dummyPlanItemsForPlot = dummyPlanItems.at(plotID);
					dummyPlanItemsForPlot.push_back(&planItem);
				}

				plotNum++;
			}
		}

		int plotID = 1;
		for (auto& dummyPlanItemsForPlot : dummyPlanItems)
		{
			ofstream dummyPlanFile(directory + string("plan_") + to_string(plotID) + string(".plan"));
			start_QGC_Plan_file(dummyPlanFile);
			start_mission(dummyPlanFile);
			write_dummy_mission_items(dummyPlanFile, dummyPlanItemsForPlot);
			end_dummy_mission(dummyPlanFile, dummyPlanItemsForPlot);
			end_QGC_Plan_file(dummyPlanFile);
			dummyPlanFile.close();
			plotID++;
		}


		
	}
}



void SwarmController::write_dummy_mission_items(ofstream& fout, const vector<PlanItem*>& planItems)
{
	unsigned int itemID = 0;

	//////////////////////////////////////////////
	//  open mission items
	fout << "        \"items\": [" << endl;


	//  Takeoff from ground / hand
	const PlanItem* planItemForInitPlot = planItems.front();

	array<float, 2> GPSCoordForInitPoint = translate_coord_to_gps(planItemForInitPlot->point.getX(), planItemForInitPlot->point.getY());
	fout << "           {" << endl;
	fout << "               \"autoContinue\": true," << endl;
	fout << "               \"command\" : 22," << endl;
	fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
	fout << "               \"frame\" : 3," << endl;
	fout << "               \"params\" : [" << endl;
	fout << "                   0," << endl;
	fout << "                   1," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout.precision(7);
	fout << "                   " << fixed << GPSCoordForInitPoint.at(0) << "," << endl;
	fout << "                   " << fixed << GPSCoordForInitPoint.at(1) << "," << endl;
	fout << "                   " << fixed << planItemForInitPlot->point.getZ() << endl;
	fout << "               ]," << endl;
	fout << "               \"type\": \"SimpleItem\"" << endl;
	fout << "           }," << endl;


	for (int i = 1; i < planItems.size(); i++)
	{
		const PlanItem* item = planItems.at(i);
		array<float, 2> GPSCoord = translate_coord_to_gps(item->point.getX(), item->point.getY());

		//  Navigate to waypoint.
		fout << "           {" << endl;
		fout << "               \"autoContinue\": true," << endl;
		fout << "               \"command\" : 16," << endl;
		fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
		fout << "               \"frame\" : 3," << endl;
		fout << "               \"params\" : [" << endl;
		fout << "                   " << item->loiterTime << "," << endl;
		fout << "                   1," << endl;
		fout << "                   0," << endl;
		fout << "                   0," << endl;
		fout.precision(7);
		fout << "                   " << fixed << GPSCoord.at(0) << "," << endl;
		fout << "                   " << fixed << GPSCoord.at(1) << "," << endl;
		fout << "                   " << fixed << item->point.getZ() << endl;
		fout << "               ]," << endl;
		fout << "               \"type\": \"SimpleItem\"" << endl;
		fout << "           }," << endl;
	}


	//  Land at location
	const PlanItem* itemForLastPlot = planItems.back();
	array<float, 2> GPSCoordForLastPoint = translate_coord_to_gps(itemForLastPlot->point.getX(), itemForLastPlot->point.getY());

	fout << "           {" << endl;
	fout << "               \"autoContinue\": true," << endl;
	fout << "               \"command\" : 21," << endl;
	fout << "               \"doJumpId\" : " << ++itemID << "," << endl;
	fout << "               \"frame\" : 3," << endl;
	fout << "               \"params\" : [" << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout << "                   0," << endl;
	fout.precision(7);
	fout << "                   " << GPSCoordForLastPoint.at(0) << "," << endl;
	fout << "                   " << GPSCoordForLastPoint.at(1) << "," << endl;
	fout << "                   -5" << endl;
	fout << "               ]," << endl;
	fout << "               \"type\": \"SimpleItem\"" << endl;
	fout << "           }" << endl;


	//////////////////////////////////////////////
	//  close mission items
	fout << "        ]," << endl;
}

void SwarmController::end_dummy_mission(ofstream& fout, const vector<PlanItem*>& planItems)
{
	const PlanItem* planItemForInitPlot = planItems.front();
	array<float, 2> GPSCoordForInitPoint = translate_coord_to_gps(planItemForInitPlot->point.getX(), planItemForInitPlot->point.getY());

	fout << "        \"plannedHomePosition\" : [" << endl;
	fout.precision(7);
	fout << "            " << fixed << GPSCoordForInitPoint.at(0) << ", " << endl;
	fout << "            " << fixed << GPSCoordForInitPoint.at(1) << ", " << endl;
	fout << "            0.0" << endl;
	fout << "        ]," << endl;
	fout << "        \"vehicleType\" : 2," << endl;
	fout << "        \"version\": 2" << endl;
	fout << "    }," << endl;
}



