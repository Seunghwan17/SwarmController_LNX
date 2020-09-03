#include "SwarmSimulator.h"
#include <fstream>
#include "Color3f.h"
#include <random>
#include <ctime>
#include "BallGeneratorCore.h"
#include "VCellCore.h"

#define _USE_MATH_DEFINES
#include <math.h>

SwarmSimulator::SwarmSimulator()
	:DynamicSimulator3D()
{
	m_dynamicVD.set_process_collision_option(false);
	m_dynamicVD.set_update_VVtx_option(true);
}



SwarmSimulator::~SwarmSimulator()
{
}



void SwarmSimulator::clear()
{
	//DynamicSimulator3D::clear();
	m_dynamicVD.clear();
	m_mapForDestinationAssignment.clear();
	//m_contactQ.clear();
	//m_restorationQ.clear();
	//m_arrivalQ.clear();
	//m_snapshotCounter = 1;
	m_timeForSimulation = 0;
	m_indexOfLastVelocityChangeEvent = 0;
	m_velocityChangeHistory.clear();

	for (int i = 0; i < m_drones.size(); i++)
	{
		m_drones.at(i).set_time(0);
		m_drones.at(i).set_status(NORMAL);
		m_drones.at(i).set_priority(0);
		m_drones.at(i).clear_way_points();
	}
}



void SwarmSimulator::initialize_scene_change(const vector<int>& destinationAssignMap)
{
	m_mapForDestinationAssignment.clear();
	for (int i = 0; i < m_drones.size(); i++)
		m_mapForDestinationAssignment[i] = destinationAssignMap.at(i);

	initialize_drone_move();

	//for debug
	/*DroneBall& evadingDrone = m_drones.back();
	add_arc_path(evadingDrone, 5, 0.1);
	update_velocity_to_arrive_at_target_time(evadingDrone, 0);*/
}



void SwarmSimulator::add_arc_path(DroneBall& drone, int numArcPoints, float curvature)
{
	rg_Point3D startPoint = drone.get_way_points().front().point;
	rg_Point3D endPoint = drone.get_way_points().back().point;
	Color3f color = drone.get_way_points().front().color;
	float radius = (endPoint - startPoint).magnitude() / 2;

	for (int i = 1; i < numArcPoints; i++)
	{
		float ratio = (float)i / (float)numArcPoints;
		float angle = M_PI * ratio;
		float cosRatio = (cos(angle) + 1) / 2;	//0 to 1
		float z = sin(angle)*radius*curvature;
		
		WayPoint arcWayPoint;
		arcWayPoint.point = startPoint*cosRatio + endPoint* (1-cosRatio) + rg_Point3D(0, 0, z);
		arcWayPoint.type = DESTINATION;
		arcWayPoint.time = m_targetArrivalTime * ratio;
		arcWayPoint.color = color;

		drone.add_way_point(i, arcWayPoint);
	}
}



void SwarmSimulator::prepare_simulation()
{
	cout << "Prepare formation change to " <<m_targetPlot->plotFileName<< endl;

	m_dynamicVD.set_time_window(1000);
	m_dynamicVD.construct_initial_VD(m_dynamicBalls);
	m_dynamicVD.construct_initial_event_queue();

	int numInitialEvent = construct_initial_swarm_event_queue();
}



void SwarmSimulator::make_drone_balls(const Plot& initialPlot)
{
	m_drones.resize(initialPlot.plotObjects.size());
	for (int i=0; i<initialPlot.plotObjects.size(); i++)
	{
		m_drones.at(i) = (DroneBall(i+1, LINEAR_BALL, Sphere(initialPlot.plotObjects.at(i).point, m_collisionRadius), rg_Point3D(), initialPlot.plotObjects.at(i).color, NORMAL, 0.0, initialPlot.plotObjects.at(i).priority));
		m_dynamicBalls.push_back(&m_drones.at(i));
	}
}



void SwarmSimulator::assign_priority()
{
	for (auto& drone : m_drones)
	{
		switch (m_priorityType)
		{
		case PRIORITY_IDENTICAL:
			drone.set_priority(0.0);
			break;
		case PRIORITY_GIVEN:
			drone.set_priority(drone.get_given_priority());
			break;
		case PRIORITY_DISTANCE:
		{
			WayPoint& initLocationWP = drone.get_way_points().front();
			WayPoint& destinationWP = drone.get_way_points().back();
			drone.set_priority(destinationWP.point.distance(initLocationWP.point));
			break;
		}
		default:
			break;
		}
	}
}



void SwarmSimulator::initialize_drone_move()
{
	for (int i = 0; i < m_drones.size(); i++)
	{
		DroneBall& currDrone = m_drones.at(i);

		WayPoint initLocationWp;
		initLocationWp.point = currDrone.get_sphere_center();
		initLocationWp.time = 0;
		initLocationWp.color = currDrone.get_color();
		initLocationWp.type = INIT_LOC;

		const rg_Point3D& destination = m_targetPlot->plotObjects.at(m_mapForDestinationAssignment.at(i)).point;
		WayPoint destinationWp;
		destinationWp.point = destination;
		destinationWp.color = m_targetPlot->plotObjects.at(m_mapForDestinationAssignment.at(i)).color;
		destinationWp.type = DESTINATION;

		switch (m_pathGenerationMode)
		{
		case TIME_ON_TARGET:
			destinationWp.time = m_targetArrivalTime;
			currDrone.set_velocity_vector((destinationWp.point - initLocationWp.point) / m_targetArrivalTime);
			break;
		case CONSTANT_SPEED:
			destinationWp.time = 0; //will not be used
			currDrone.set_velocity_vector((destinationWp.point - initLocationWp.point).getUnitVector()*m_basicSpeed);
			break;
		default:
			break;
		}

		currDrone.set_time(initLocationWp.time);
		//currDrone.set_sphere_center(initLocationWp.point);
		currDrone.set_status(NORMAL);

		currDrone.get_way_points().clear();
		currDrone.get_way_points().push_back(initLocationWp);
		currDrone.get_way_points().push_back(destinationWp);

		//currDrone.set_color(initialScene.at(i).color);
		//currDrone.set_given_priority(initialScene.at(i).priority);
	}
	assign_priority();
}



void SwarmSimulator::update_velocity_to_arrive_at_target_time(DroneBall& drone, const double& currTime)
{
	WayPoint& destinationWP = drone.get_way_points().at(drone.get_WP_index()+1);

	rg_Point3D displacement = destinationWP.point - drone.calculate_position_at_time(currTime);
	double proceedingTime = destinationWP.time - currTime;
	float speed = displacement.magnitude() / proceedingTime;
	if (speed < 0 || speed > MAX_SPEED)
	{
		speed = MAX_SPEED;
		destinationWP.time = currTime + displacement.magnitude() / speed;
	}
		
	drone.set_velocity_vector(displacement.getUnitVector()*speed);
}



int SwarmSimulator::construct_initial_swarm_event_queue()
{
	add_initial_contact_event();
	add_initial_arrival_event();
	return (m_contactQ.size() + m_arrivalQ.size());
}



void SwarmSimulator::add_initial_contact_event()
{
	list<VFaceCore*> faces;
	m_dynamicVD.getAllVFaces(faces);
	for (auto& face : faces)
	{
		m_mapFromIDToLatestContactKey[face->getID()] = 0;

		double contactTime = calculate_contact_interval_for_face_from_now(face, 2 * m_conjunctionRadius, 0).at(0);
		if (contactTime > 0)
			adjust_contact_event_queue(contactTime, face);
	}
}




void SwarmSimulator::add_initial_arrival_event()
{
	for (vector<DroneBall>::iterator it = m_drones.begin(); it != m_drones.end(); it++)
	{
		DroneBall& currDrone = *it;
		m_mapFromIDToLatestRestorationKey[currDrone.get_ID()] = 0;
		m_mapFromIDToLatestArrivalKey[currDrone.get_ID()] = 0;

		double arrivalTime = currDrone.calculate_arrival_time_to_next_WP(0);
		adjust_arrival_event_queue(arrivalTime, &*it);
		//cout << "Arrival event: " << (*it).get_ID() << " - " << arrivalTime << endl;
	}
}



pair<SWARM_CONTROL_EVENT_TYPE, double> SwarmSimulator::find_next_swarm_control_event_info()
{
	array<double, 3> eventTimes = { DBL_MAX, DBL_MAX, DBL_MAX };

	/*const PQ_Node<VFaceCore*>* topContactEvent = m_contactQ.topNode();
	const PQ_Node<DroneBall*>* topRestorationEvent = m_restorationQ.topNode();
	const PQ_Node<DroneBall*>* topArrivalEvent = m_arrivalQ.topNode();*/

	const PredictedEvent* topContactEvent = find_imminent_contact_event();
	const PredictedEvent* topRestorationEvent = find_imminent_restoration_event();
	const PredictedEvent* topArrivalEvent = find_imminent_arrival_event();

	if (topContactEvent != NULL)
	{
		eventTimes.at(0) = topContactEvent->time;
	}

	if (topRestorationEvent != NULL)
	{
		eventTimes.at(1) = topRestorationEvent->time;
	}

	if (topArrivalEvent != NULL)
	{
		eventTimes.at(2) = topArrivalEvent->time;
	}

	int eventIndexInNearestFuture = 3;
	double eventTimeInNearestFuture = DBL_MAX;
	for (int i = 0; i < 3; i++)
	{
		if (eventTimes.at(i) < eventTimeInNearestFuture)
		{
			eventIndexInNearestFuture = i;
			eventTimeInNearestFuture = eventTimes.at(i);
		}
	}

	SWARM_CONTROL_EVENT_TYPE eventType = SC_NULL;
	switch (eventIndexInNearestFuture)
	{
	case 0:
		eventType = SC_CONJUNCTION;
		break;
	case 1:
		eventType = SC_RESTORATION;
		break;
	case 2:
		eventType = SC_ARRIVAL;
		break;
	case 3:	//No event
	default:
		eventTimeInNearestFuture = m_dynamicVD.get_time_window();
		break;
	}

	return make_pair(eventType, eventTimeInNearestFuture);
}



const PredictedEvent* SwarmSimulator::find_imminent_contact_event()
{
	if (m_contactQ.empty())
		return nullptr;

	while (!is_contact_event_valid(m_contactQ.top()))
	{
		m_contactQ.pop();
		if (m_contactQ.empty())
			return nullptr;
	}

	return &m_contactQ.top();
}



const PredictedEvent* SwarmSimulator::find_imminent_restoration_event()
{
	if (m_restorationQ.empty())
		return nullptr;

	while (!is_restoration_event_valid(m_restorationQ.top()))
	{
		m_restorationQ.pop();
		if (m_restorationQ.empty())
			return nullptr;
	}

	return &m_restorationQ.top();
}



const PredictedEvent* SwarmSimulator::find_imminent_arrival_event()
{
	if (m_arrivalQ.empty())
		return nullptr;

	while (!is_arrival_event_valid(m_arrivalQ.top()))
	{
		m_arrivalQ.pop();
		if (m_arrivalQ.empty())
			return nullptr;
	}

	return &m_arrivalQ.top();
}



bool SwarmSimulator::is_contact_event_valid(const PredictedEvent& event)
{
	auto it = m_mapFromIDToLatestContactKey.find(event.entityID);
	if (it != m_mapFromIDToLatestContactKey.end()
		&& event.key == (*it).second)
		return true;
	else
		return false;
}



bool SwarmSimulator::is_restoration_event_valid(const PredictedEvent& event)
{
	auto it = m_mapFromIDToLatestRestorationKey.find(event.entityID);
	if (it != m_mapFromIDToLatestRestorationKey.end()
		&& event.key == (*it).second)
		return true;
	else
		return false;
}



bool SwarmSimulator::is_arrival_event_valid(const PredictedEvent& event)
{
	auto it = m_mapFromIDToLatestArrivalKey.find(event.entityID);
	if (it != m_mapFromIDToLatestArrivalKey.end()
		&& event.key == (*it).second)
		return true;
	else
		return false;
}



void SwarmSimulator::adjust_contact_event_key_map_for_face(VFaceCore* face, const bool& isBornFace)
{
	if (isBornFace)
		m_mapFromIDToLatestContactKey[face->getID()] = 0;
	else
		m_mapFromIDToLatestContactKey.erase(face->getID());
}



void SwarmSimulator::proceed_to_time_for_history_generation(const double& time)
{
	pair<DVD_EVENT_TYPE, double> nextDVDEventInfo = m_dynamicVD.find_next_event_info();
	pair<SWARM_CONTROL_EVENT_TYPE, double> nextSwarmControlEventInfo = find_next_swarm_control_event_info();

	while (((nextDVDEventInfo.second < time) || (nextSwarmControlEventInfo.second < time))&&!is_every_ball_arrive())
	{
		if (nextSwarmControlEventInfo.second < nextDVDEventInfo.second)
		{
			process_swarm_control_event(nextSwarmControlEventInfo.first);
		}
		else
		{
			process_DVD_event();
		}
		nextSwarmControlEventInfo = find_next_swarm_control_event_info();
		nextDVDEventInfo = m_dynamicVD.find_next_event_info();
	}
	m_dynamicVD.proceed_to_future(time);

	/*list<DroneBall*> dronesToReturnColor;
	for (auto& colorChangedDronePair : mapForColorChangedTime)
	{
		if (colorChangedDronePair.second + 2 < time)
			dronesToReturnColor.push_back(colorChangedDronePair.first);
	}

	for (auto& drone : dronesToReturnColor)
	{
		mapForColorChangedTime.erase(drone);
		drone->set_color(BLACK);
	}*/
}



void SwarmSimulator::increase_time(const double& timeIncrement)
{
	double targetTime = m_dynamicVD.get_current_time() + timeIncrement;
	proceed_to_time(targetTime);
}



void SwarmSimulator::process_DVD_event()
{
	pair<DVD_EVENT_TYPE, void*> nextEventInfo = m_dynamicVD.find_next_event_information();
	if (nextEventInfo.first == FACE_FLIP)
		adjust_contact_event_key_map_for_face(static_cast<VFaceCore*>(nextEventInfo.second), false);

	m_dynamicVD.process_next_event();
	DynamicEvent3D* lastEvent = m_dynamicVD.get_event_history().back();
	switch (lastEvent->get_event_type())
	{
	case EDGE_FLIP:
	{
		FlipEvent3D* flipEvent = static_cast<FlipEvent3D*>(lastEvent);
		VFaceCore* newBornFace = m_dynamicVD.find_face_defined_by_quintuplet(flipEvent->get_IDs_of_quintuplet());
		adjust_contact_event_key_map_for_face(newBornFace, true);
		add_contact_event_for_new_face(static_cast<FlipEvent3D*>(lastEvent));
		break;
	}	
	case COLLISION:
		cout << "Collision occur: " <<static_cast<CollisionEvent3D*>(lastEvent)->get_left_ball_ID()<<", "<< static_cast<CollisionEvent3D*>(lastEvent)->get_right_ball_ID()<< endl;
		break;
	case VELOCITY_CHANGE:
	case FACE_FLIP:
	case BEGIN:
	case FINISH:
		break;
	}
}



void SwarmSimulator::process_swarm_control_event(const SWARM_CONTROL_EVENT_TYPE& eventType)
{
	switch (eventType)
	{
	case SC_CONJUNCTION:
		avoid_collision();
		break;
	case SC_RESTORATION:
		restore_velocity();
		break;
	case SC_ARRIVAL:
		process_arrival();
		break;
	case SC_NULL:
	default:
		break;
	}

#ifdef PRINT_SCENES
	//if (eventType == SC_CONJUNCTION || eventType == SC_RESTORATION)
		//print_scene(m_snapshotFileName);
#endif
}



void SwarmSimulator::go_to_time_using_history(const double& targetTime)
{
	if (targetTime > m_timeForSimulation)
	{
		while (m_indexOfLastVelocityChangeEvent + 1 < m_velocityChangeHistory.size()&&
			get<1>(m_velocityChangeHistory.at(m_indexOfLastVelocityChangeEvent + 1)) < targetTime)
		{
			tuple<int, double, rg_Point3D>& nextVelocityChangeEvent = m_velocityChangeHistory.at(m_indexOfLastVelocityChangeEvent + 1);
			DroneBall& drone = m_drones.at(get<0>(nextVelocityChangeEvent));
			drone.move_to_position_at_time(get<1>(nextVelocityChangeEvent));
			drone.set_velocity_vector(get<2>(nextVelocityChangeEvent));
			m_indexOfLastVelocityChangeEvent++;
		}

		m_timeForSimulation = targetTime;

		//move_drones_to_time(targetTime);
	}
	else
	{
		//Comming Soon....
	}
}



void SwarmSimulator::increase_time_using_history(const double& timeIncrement)
{
	double targetTime = m_timeForSimulation + timeIncrement;
	go_to_time_using_history(targetTime);
}



array<double, 2> SwarmSimulator::calculate_contact_interval_for_face_from_now(const VFaceCore* face, const float& contactDistance, const double& currTime) const
{
	if (face->isUnbounded() || face->getLeftVCell()->isUnbounded() || face->getRightVCell()->isUnbounded())
		return { DBL_MAX, DBL_MAX };

	int leftGeneratorID = static_cast<BallGeneratorCore*>(face->getLeftVCell()->getGenerator())->getInputID();
	int rightGeneratorID = static_cast<BallGeneratorCore*>(face->getRightVCell()->getGenerator())->getInputID();

	//for debug
	//if (leftGeneratorID == 4 && rightGeneratorID == 5 || leftGeneratorID == 5 && rightGeneratorID == 4 )
	//	cout << "Problem face" << endl;

	if (leftGeneratorID <= m_drones.size() && rightGeneratorID <= m_drones.size())
	{
		const DroneBall& leftDrone = m_drones.at(leftGeneratorID-1);
		const DroneBall& rightDrone = m_drones.at(rightGeneratorID-1);

		rg_Point3D relativePosition = leftDrone.calculate_position_at_time(currTime) - rightDrone.calculate_position_at_time(currTime);
		rg_Point3D relativeVelocity = leftDrone.get_velocity_vector() - rightDrone.get_velocity_vector();

		double possibleCollisionTimeFromNow = calculate_collision_time_from_now(relativePosition, relativeVelocity);
		if (0<possibleCollisionTimeFromNow && possibleCollisionTimeFromNow < m_dynamicVD.get_time_window())
		{
			array<double, 2> contactInterval = calculate_contact_interval_from_now(relativePosition, relativeVelocity, contactDistance);
			return { contactInterval.at(0), contactInterval.at(1) };
		}
		else
			return { DBL_MAX, DBL_MAX };
	}
	else
		return { DBL_MAX, DBL_MAX };
}



array<double, 2> SwarmSimulator::calculate_contact_interval_from_now(const rg_Point3D& relativePosition, const rg_Point3D& relativeVelocity, const float& contactDistance) const
{
	double a = pow(relativeVelocity.getX(), 2) + pow(relativeVelocity.getY(), 2) + pow(relativeVelocity.getZ(), 2);
	double b = relativeVelocity.getX()*relativePosition.getX() + relativeVelocity.getY()*relativePosition.getY() + relativeVelocity.getZ()*relativePosition.getZ();
	double c = pow(relativePosition.getX(), 2) + pow(relativePosition.getY(), 2) + pow(relativePosition.getZ(), 2) - pow(contactDistance, 2);

	if(a==0)
		return { DBL_MAX, DBL_MAX };

	//DECIMAL minDistance = c - b * b / a;

	double insideSqrt = pow(b, 2) - a * c;
	if (insideSqrt >= 0)
	{
		return { (-b - sqrt(insideSqrt)) / a, (-b + sqrt(insideSqrt)) / a };
	}
	else
	{
		return { DBL_MAX, DBL_MAX };
	}
}



bool SwarmSimulator::add_contact_event_for_new_face(const FlipEvent3D* edgeFlipEvent)
{
	double currTime = m_dynamicVD.get_current_time();
	VFaceCore* newFace = m_dynamicVD.find_face_defined_by_quintuplet(edgeFlipEvent->get_IDs_of_quintuplet());
	array<double, 2> contactInterval = calculate_contact_interval_for_face_from_now(newFace, 2 * m_conjunctionRadius, currTime);
	if (contactInterval.at(0) > 0)
	{
		//Will contact
		adjust_contact_event_queue(contactInterval.at(0)+currTime, newFace);
		return true;
	}
	else if (contactInterval.at(1) > 0)
	{
		//On contact
		adjust_contact_event_queue(currTime, newFace);
		return true;
	}
	else
	{
		//No contact
		return false;
	}
}



void SwarmSimulator::avoid_collision()
{
	/*const PQ_Node<VFaceCore*>* topContactEvent = m_contactQ.topNode();
	VFaceCore* faceOnContact = topContactEvent->getEntity();
	double eventTime = topContactEvent->getKey();*/

	const PredictedEvent& topContactEvent = m_contactQ.top();
	VFaceCore* faceOnContact = static_cast<VFaceCore*>(topContactEvent.entity);
	double eventTime = topContactEvent.time;

	m_contactQ.pop();

	BallGeneratorCore* leftGenerator = static_cast<BallGeneratorCore*>(faceOnContact->getLeftVCell()->getGenerator());
	BallGeneratorCore* rightGenerator = static_cast<BallGeneratorCore*>(faceOnContact->getRightVCell()->getGenerator());

	DroneBall* leftDrone = static_cast<DroneBall*>(leftGenerator->getUserData());
	DroneBall* rightDrone = static_cast<DroneBall*>(rightGenerator->getUserData());

	leftDrone->move_to_position_at_time(eventTime);
	rightDrone->move_to_position_at_time(eventTime);
	cout << "Contact: drone " << leftDrone->get_ID() << " & " << rightDrone->get_ID() << " at " << eventTime << endl;

	set<DynamicBall*> neighbors = collect_neighbors_for_drone_pair({ leftDrone, rightDrone });
	if (m_avoidanceType == ON_THE_PLANE)
	{
		//Step 1. try speed change
		pair<int, double> optimalSpeedConfig = find_optimal_speed_config({ leftDrone, rightDrone }, neighbors, eventTime);
		if (is_avoidance_valid(optimalSpeedConfig, eventTime))
		{
			if (optimalSpeedConfig.first > 0)
			{
				change_speed_to_avoid_collision(leftDrone, optimalSpeedConfig.first,
					eventTime, optimalSpeedConfig.second);
				//rightDrone->set_color(GREEN);
				//mapForColorChangedTime[rightDrone] = eventTime;
			}
			else
			{
				change_speed_to_avoid_collision(rightDrone, -optimalSpeedConfig.first,
					eventTime, optimalSpeedConfig.second);
				//leftDrone->set_color(GREEN);
				//mapForColorChangedTime[leftDrone] = eventTime;
			}
		}
		else
		{
			//Step 2. try role change
			optimalSpeedConfig = find_optimal_speed_config({ leftDrone, rightDrone }, neighbors, eventTime, true);
			change_role_to_avoid_collision({ leftDrone, rightDrone }, optimalSpeedConfig.first, eventTime, optimalSpeedConfig.second);
		}
	}
	else
	{
		array<double, 4> nearestCollisionTimesForConfigurations;
		nearestCollisionTimesForConfigurations.at(0) = calculate_nearest_collision_time_for_candidate_avoidance(leftDrone, eventTime, rg_Point3D(0, 0, AVOIDANCE_SPEED));
		nearestCollisionTimesForConfigurations.at(1) = calculate_nearest_collision_time_for_candidate_avoidance(leftDrone, eventTime, rg_Point3D(0, 0, -AVOIDANCE_SPEED));
		nearestCollisionTimesForConfigurations.at(2) = calculate_nearest_collision_time_for_candidate_avoidance(rightDrone, eventTime, rg_Point3D(0, 0, AVOIDANCE_SPEED));
		nearestCollisionTimesForConfigurations.at(3) = calculate_nearest_collision_time_for_candidate_avoidance(rightDrone, eventTime, rg_Point3D(0, 0, -AVOIDANCE_SPEED));

		if (leftDrone->get_status() == ARRIVAL)
		{
			//cout << "Left is arrived" << endl;
			nearestCollisionTimesForConfigurations.at(2) = 0;
			nearestCollisionTimesForConfigurations.at(3) = 0;
		}
		if (rightDrone->get_status() == ARRIVAL)
		{
			//cout << "Right is arrived" << endl;
			nearestCollisionTimesForConfigurations.at(0) = 0;
			nearestCollisionTimesForConfigurations.at(1) = 0;
		}


		int indexOfOptimalConfiguration = 0;
		double maxNearestCollisionTime = 0;
		for (int i = 0; i < 4; i++)
		{
			if (nearestCollisionTimesForConfigurations.at(i) > maxNearestCollisionTime)
			{
				maxNearestCollisionTime = nearestCollisionTimesForConfigurations.at(i);
				indexOfOptimalConfiguration = i;
			}
		}

		//cout << "Farthest collision time: " << maxNearestCollisionTime << " with config " << indexOfOptimalConfiguration << endl;

		switch (indexOfOptimalConfiguration)
		{
		case 0:
		default:
			take_by_pass_to_avoid_collision(leftDrone, eventTime, rg_Point3D(0, 0, AVOIDANCE_SPEED));
			break;
		case 1:
			take_by_pass_to_avoid_collision(leftDrone, eventTime, rg_Point3D(0, 0, -AVOIDANCE_SPEED));
			break;
		case 2:
			take_by_pass_to_avoid_collision(rightDrone, eventTime, rg_Point3D(0, 0, AVOIDANCE_SPEED));
			break;
		case 3:
			take_by_pass_to_avoid_collision(rightDrone, eventTime, rg_Point3D(0, 0, -AVOIDANCE_SPEED));
			break;
		}
	}	
}



set<DynamicBall*> SwarmSimulator::collect_neighbors_for_drone_pair(const array<DroneBall*, 2>& dronePair)
{
	set<DynamicBall*> commonNeighbors;
	m_dynamicVD.find_neighbor_balls(dronePair.at(0), commonNeighbors);
	m_dynamicVD.find_neighbor_balls(dronePair.at(1), commonNeighbors);

	return commonNeighbors;
}



void SwarmSimulator::restore_velocity()
{
	/*const PQ_Node<DroneBall*>* topRestorationEvent = m_restorationQ.topNode();
	DroneBall* drone = topRestorationEvent->getEntity();
	double eventTime = topRestorationEvent->getKey();*/

	const PredictedEvent& topRestorationEvent = m_restorationQ.top();
	DroneBall* drone = static_cast<DroneBall*>(topRestorationEvent.entity);
	double eventTime = topRestorationEvent.time;

	m_restorationQ.pop();

	drone->move_to_position_at_time(eventTime);
	rg_Point3D currCorrd = drone->calculate_position_at_time(eventTime);
	cout << "Restore velocity for drone " << drone->get_ID() << " at " << eventTime << " - [" << currCorrd.getX() << " " << currCorrd.getY() << " " << currCorrd.getZ() << "]" << endl;


	drone->set_status(NORMAL);
	const Color3f& droneColor = drone->find_current_WP().color;
	WayPoint WPForRT = make_way_point_at_curr_position(drone, eventTime, AVOIDANCE_END, droneColor);
	
	drone->increase_WP_index();
	drone->add_way_point_at_current(WPForRT);

	if (m_avoidanceType == OFF_THE_PLANE)
	{
		WayPoint WPForAS = drone->get_way_points().at(drone->get_WP_index()-1);
		WayPoint WPForDestination = drone->find_final_WP();
		float distance = WPForAS.point.distance(WPForDestination.point);
		if (distance > 0.1)
		{
			WayPoint WPForPausePoint;
			WPForPausePoint.point = WPForAS.point;
			WPForPausePoint.time = 0;
			WPForPausePoint.type = PAUSE;
			drone->add_way_point_at_next(WPForPausePoint);
		}
	}
	
	const WayPoint& nextWP = drone->find_next_WP();

	switch (m_pathGenerationMode)
	{
	case CONSTANT_SPEED:
	{
		rg_Point3D velocity = (nextWP.point - WPForRT.point).getUnitVector()*AVOIDANCE_SPEED;
		drone->set_velocity_vector(velocity);
		break;
	}
	case TIME_ON_TARGET:
	{
		update_velocity_to_arrive_at_target_time(*drone, eventTime);
		break;
	}
	default:
		break;
	}

	update_contact_event_for_neighbors(drone, eventTime);
	if(m_avoidanceType == ON_THE_PLANE)
		update_restoration_event_for_neighbors(drone, eventTime);

	double arrivalTime = drone->calculate_arrival_time_to_next_WP(eventTime);
	adjust_arrival_event_queue(arrivalTime, drone);

	m_dynamicVD.add_velocity_change_event(drone->get_ID(), eventTime);
	add_velocity_change_history(drone, eventTime);
}




void SwarmSimulator::process_arrival()
{
	/*const PQ_Node<DroneBall*>* topArrivalEvent = m_arrivalQ.topNode();
	DroneBall* drone = topArrivalEvent->getEntity();
	double eventTime = topArrivalEvent->getKey();*/

	const PredictedEvent& topArrivalEvent = m_arrivalQ.top();
	DroneBall* drone = static_cast<DroneBall*>(topArrivalEvent.entity);
	double eventTime = topArrivalEvent.time;

	m_arrivalQ.pop();

	drone->move_to_position_at_time(eventTime);

	drone->increase_WP_index();
	WayPoint& currWP = drone->get_way_points().at(drone->get_WP_index());
	drone->set_color(currWP.color);
	currWP.time = eventTime;

	//for debug
	/*if (drone->get_ID() == 30)
	{
		cout << "Total wayPoints for drone " << drone->get_ID() << ": " << drone->get_way_points().size() << ", curr index: "<<drone->get_WP_index()<<endl;
		int counter = 0;
		for (auto& WP : drone->get_way_points())
		{
			string tag;
			switch (WP.type)
			{
			case INIT_LOC:
				tag = "init";
				break;
			case AVOIDANCE_START:
				tag = "AS";
				break;
			case AVOIDANCE_END:
				tag = "AE";
				break;
			case PAUSE:
				tag = "Pause";
				break;
			case DESTINATION:
				tag = "Dest";
				break;
			}

			cout << "WP["<<counter<<"] - "<< tag << ": [" << WP.point.getX() << ", " << WP.point.getY() << ", " << WP.point.getZ() << "], " << WP.time << endl;
		}
	}*/

	if (drone->get_WP_index() == drone->get_way_points().size() - 1) //arrive final destination
	{
		drone->set_status(ARRIVAL);
		drone->set_velocity_vector(rg_Point3D(0, 0, 1.0e-10));
		drone->set_priority(0);
	}
	else
	{
		drone->set_status(NORMAL);
		
		switch (m_pathGenerationMode)
		{
		case CONSTANT_SPEED:
		{
			const WayPoint& nextWP = drone->find_next_WP();
			drone->set_velocity_vector((nextWP.point - drone->get_sphere_center()).getUnitVector()*m_basicSpeed);
			break;
		}
		case TIME_ON_TARGET:
			update_velocity_to_arrive_at_target_time(*drone, eventTime);
			break;
		default:
			break;
		}
		
		double arrivalTime = drone->calculate_arrival_time_to_next_WP(eventTime);
		adjust_arrival_event_queue(arrivalTime, drone);
	}

	adjust_restoration_event_queue(DBL_MAX, drone);
	update_contact_event_for_neighbors(drone, eventTime);
	//update_restoration_event_for_neighbors(drone, eventTime);

	m_dynamicVD.add_velocity_change_event(drone->get_ID(), eventTime);

	rg_Point3D coord = drone->get_sphere_center();
	cout << "Drone " << drone->get_ID() << " is arrive at " << eventTime <<" : ["<< coord.getX()<<", " << coord.getY() << ", " << coord.getZ() << "]"<< endl;
}



void SwarmSimulator::update_contact_event_for_neighbors(const DroneBall* drone, const double& currTime)
{
	const BallGeneratorCore* generator = m_dynamicVD.get_generator_from_ID(drone->get_ID());
	list<VFaceCore*> adjacentFaces;
	generator->getCellInVoronoiDiagram()->findBoundingVFaces(adjacentFaces);
	for (auto& face : adjacentFaces)
	{
		array<double, 2> contactInterval = calculate_contact_interval_for_face_from_now(face, 2 * m_conjunctionRadius, currTime);
		
		if (contactInterval.at(0) > m_dynamicVD.get_time_window())
		{
			adjust_contact_event_queue(DBL_MAX, face);
		}
		else if (contactInterval.at(0) > 0)
		{
			adjust_contact_event_queue(currTime+contactInterval.at(0), face);
		}
		else if (contactInterval.at(1) > 0)
		{
			adjust_contact_event_queue(currTime, face);
		}
	}
}



void SwarmSimulator::update_restoration_event_for_neighbors(const DroneBall* drone, const double& currTime)
{
	set<DynamicBall*> neighbors;
	m_dynamicVD.find_neighbor_balls(drone, neighbors);
	for (set<DynamicBall*>::const_iterator it = neighbors.cbegin(); it != neighbors.cend(); it++)
	{
		if ((*it)->get_type() == LINEAR_BALL)
		{
			DroneBall* neighbor = static_cast<DroneBall*>(*it);
			if (neighbor->get_status() == AVOIDING)
			{
				set<DynamicBall*> neighborsOfNeighbor;
				m_dynamicVD.find_neighbor_balls(neighbor, neighborsOfNeighbor);

				double contactEscapeTime = find_contact_escape_time(neighbor, neighbor->get_velocity_vector(), neighborsOfNeighbor, currTime);
				adjust_restoration_event_queue(contactEscapeTime, neighbor);
			}
		}
	}
}



bool SwarmSimulator::adjust_contact_event_queue(const double& eventTime, VFaceCore* willBeContactFace)
{
	if (eventTime < m_dynamicVD.get_time_window())
	{
		/*if (m_contactQ.doesHave(willBeContactFace) == true)
		{
			m_contactQ.replaceKey(willBeContactFace, eventTime);
		}
		else
		{
			m_contactQ.push(willBeContactFace, eventTime);
		}*/

		int key = ++m_mapFromIDToLatestContactKey.at(willBeContactFace->getID());
		m_contactQ.push({ willBeContactFace->getID(), willBeContactFace, eventTime, key });

		return true;
	}
	else
	{
		/*if (m_contactQ.doesHave(willBeContactFace) == true)
		{
			m_contactQ.replaceKey(willBeContactFace, -1);
			m_contactQ.pop();
		}*/
		++m_mapFromIDToLatestContactKey.at(willBeContactFace->getID());
		return false;
	}
}



bool SwarmSimulator::adjust_restoration_event_queue(const double& eventTime, DroneBall* willRestorateDrone)
{
	if (eventTime < m_dynamicVD.get_time_window())
	{
		/*if (m_restorationQ.doesHave(willRestorateDrone) == true)
		{
			m_restorationQ.replaceKey(willRestorateDrone, eventTime);
		}
		else
		{
			m_restorationQ.push(willRestorateDrone, eventTime);
		}*/
		int key = ++m_mapFromIDToLatestRestorationKey.at(willRestorateDrone->get_ID());
		m_restorationQ.push({ willRestorateDrone->get_ID(), willRestorateDrone, eventTime, key });

		return true;
	}
	else
	{
		/*if (m_restorationQ.doesHave(willRestorateDrone) == true)
		{
			m_restorationQ.replaceKey(willRestorateDrone, -1);
			m_restorationQ.pop();
		}*/
		++m_mapFromIDToLatestRestorationKey.at(willRestorateDrone->get_ID());

		return false;
	}
}




bool SwarmSimulator::adjust_arrival_event_queue(const double& eventTime, DroneBall* willArriveDrone)
{
	if (eventTime < m_dynamicVD.get_time_window())
	{
		/*if (m_arrivalQ.doesHave(willArriveDrone) == true)
		{
			m_arrivalQ.replaceKey(willArriveDrone, eventTime);
		}
		else
		{
			m_arrivalQ.push(willArriveDrone, eventTime);
		}*/
		int key = ++m_mapFromIDToLatestArrivalKey.at(willArriveDrone->get_ID());
		m_arrivalQ.push({ willArriveDrone->get_ID(), willArriveDrone, eventTime, key });

		return true;
	}
	else
	{
		/*if (m_arrivalQ.doesHave(willArriveDrone) == true)
		{
			m_arrivalQ.replaceKey(willArriveDrone, -1);
			m_arrivalQ.pop();
		}*/
		++m_mapFromIDToLatestArrivalKey.at(willArriveDrone->get_ID());

		return false;
	}
}



bool sort_in_increasing_order_by_contact_escape_time(pair<int, double>& lhs, pair<int, double>& rhs)
{
	return lhs.second < rhs.second;
}



pair<int, double> SwarmSimulator::find_optimal_speed_config(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& isRoleChange/*= false*/) const
{
	if ((dronesOnContact.at(0)->get_status() == ARRIVAL || dronesOnContact.at(1)->get_status() == ARRIVAL) && isRoleChange == false)
		return make_pair(0, DBL_MAX);

	list<pair<int, double>> collisionFreeVelocityConfigs = collect_valid_velocity_configs_with_contact_escape_time(dronesOnContact, neighbors, currTime, isRoleChange);
	if (collisionFreeVelocityConfigs.empty())
	{
		cout << "Empty velocity config" << endl;
		collect_valid_velocity_configs_with_contact_escape_time_debug(dronesOnContact, neighbors, currTime, isRoleChange);
		return make_pair(0, DBL_MAX);
	}
		

	collisionFreeVelocityConfigs.sort(sort_in_increasing_order_by_contact_escape_time);
	return collisionFreeVelocityConfigs.front();
}



array<rg_Point3D, 2> SwarmSimulator::find_basic_velocities(const array<DroneBall*, 2> dronesOnContact, const double& currTime) const
{
	array<rg_Point3D, 2> basicVelocities = { dronesOnContact.at(0)->get_velocity_vector(), dronesOnContact.at(1)->get_velocity_vector() };
	/*if (dronesOnContact.at(0)->get_status() == ON_AVOIDANCE)
		basicVelocities.at(0) = dronesOnContact.at(0)->calculate_velocity_to_time_on_target(currTime);

	if (dronesOnContact.at(1)->get_status() == ON_AVOIDANCE)
		basicVelocities.at(1) = dronesOnContact.at(1)->calculate_velocity_to_time_on_target(currTime);*/

	return basicVelocities;
}



array<rg_Point3D, 2> SwarmSimulator::find_velocities_for_role_change(const array<DroneBall*, 2> dronesOnContact, const double& currTime) const
{
	rg_Point3D firstDronePos = dronesOnContact.at(0)->calculate_position_at_time(currTime);
	rg_Point3D secondDronePos = dronesOnContact.at(1)->calculate_position_at_time(currTime);

	rg_Point3D firstDroneDestination = dronesOnContact.at(0)->find_destination();
	rg_Point3D secondDroneDestination = dronesOnContact.at(1)->find_destination();

	rg_Point3D velocityForFirstDrone, velocityForSecondDrone;
	switch (m_pathGenerationMode)
	{
	case CONSTANT_SPEED:
		velocityForFirstDrone = (secondDroneDestination - firstDronePos).getUnitVector()*m_basicSpeed;
		velocityForSecondDrone = (firstDroneDestination - secondDronePos).getUnitVector()*m_basicSpeed;
		break;
	case TIME_ON_TARGET:
		//Need implementation
		break;
	default:
		break;
	}

	return { velocityForFirstDrone, velocityForSecondDrone };
}



double SwarmSimulator::find_contact_escape_time(const DroneBall* drone, const rg_Point3D& velocity, const set<DynamicBall*>& neighbors, const double& currTime) const
{
	map<double, double> mapForContactIntervals;
	rg_Point3D position = drone->calculate_position_at_time(currTime);
	for (set<DynamicBall*>::const_iterator it = neighbors.begin(); it != neighbors.end(); it++)
	{
		const DroneBall* neighbor = static_cast<DroneBall*>(*it);
		if (neighbor != drone && neighbor->get_status() != ARRIVAL)
		{
			rg_Point3D relativePosition = neighbor->calculate_position_at_time(currTime) - position;
			rg_Point3D relativeVelocity = neighbor->get_velocity_vector() - velocity;
			
			array<double, 2> contactInterval = calculate_contact_interval_from_now(relativePosition, relativeVelocity, 2 * m_conjunctionRadius*1.1);
			//cout << "Drone " << drone->get_ID() << " and neighbor " << neighbor->get_ID() << " distance: " << relativePosition.magnitude() << ", CET: " << contactInterval.at(0) << endl;			
			
			if (contactInterval.at(0) != DBL_MAX)
				mapForContactIntervals[contactInterval.at(0)] = contactInterval.at(1);			
		}
	}

	double contactEscapeTime = 0;

	//map for contact intervals was sorted in increasing order
	for (map<double, double>::const_iterator it = mapForContactIntervals.cbegin(); it != mapForContactIntervals.cend(); it++)
		if ((*it).first < contactEscapeTime && (*it).second > contactEscapeTime)
			contactEscapeTime = (*it).second;

	return currTime+contactEscapeTime;
}



bool SwarmSimulator::is_avoidance_valid(const pair<int, double>& speedConfig, const double& eventTime) const
{
	bool bResult = true;
	switch (m_pathGenerationMode)
	{
	case TIME_ON_TARGET:
		bResult = speedConfig.second < m_targetArrivalTime;
		break;
	case CONSTANT_SPEED:
		bResult = speedConfig.second < eventTime + m_maxAvoidanceDuration;
		break;
	default:
		break;
	}

	return bResult;
}



list<pair<int, double>> SwarmSimulator::collect_valid_velocity_configs_with_contact_escape_time(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& bRoleChange /*= false*/) const
{
	array<rg_Point3D, 2> positions = { dronesOnContact.at(0)->calculate_position_at_time(currTime), dronesOnContact.at(1)->calculate_position_at_time(currTime) };
	array<rg_Point3D, 2> velocities;
	if (bRoleChange)
		velocities = find_velocities_for_role_change(dronesOnContact, currTime);
	else
		velocities = find_basic_velocities(dronesOnContact, currTime);

	list<int> velocityConfigs;
	if (dronesOnContact.at(0)->get_priority() >= dronesOnContact.at(1)->get_priority())
		for (int i = 1; i <= m_maxSpeedMultiplier * 10; i++)
			velocityConfigs.push_back(-i);

	if (dronesOnContact.at(0)->get_priority() <= dronesOnContact.at(1)->get_priority())
		for (int i = 1; i <= m_maxSpeedMultiplier * 10; i++)
			velocityConfigs.push_back(i);


	list<pair<int, double>> velocityConfigsWithContactEscapeTime;
	for (auto& velocityConfig : velocityConfigs)
	{
		DroneBall* drone = nullptr;
		rg_Point3D velocity;
		if (velocityConfig > 0)
		{
			drone = dronesOnContact.at(0);
			velocity = velocities.at(0)*velocityConfig*0.1;
		}
		else
		{
			drone = dronesOnContact.at(1);
			velocity = velocities.at(1)*-velocityConfig*0.1;
		}

		double nearestCollisionTime = calculate_nearest_collision_time_with_neighbors(drone, velocity, neighbors, currTime);
		double escapeTime = find_contact_escape_time(drone, velocity, neighbors, currTime);

		if (nearestCollisionTime > escapeTime)
		{
			velocityConfigsWithContactEscapeTime.push_back(make_pair(velocityConfig, escapeTime));
		}
	}
	return velocityConfigsWithContactEscapeTime;
}



void SwarmSimulator::collect_valid_velocity_configs_with_contact_escape_time_debug(const array<DroneBall*, 2> dronesOnContact, const set<DynamicBall*>& neighbors, const double& currTime, const bool& bRoleChange /*= false*/) const
{
	array<rg_Point3D, 2> positions = { dronesOnContact.at(0)->calculate_position_at_time(currTime), dronesOnContact.at(1)->calculate_position_at_time(currTime) };
	array<rg_Point3D, 2> velocities;
	if (bRoleChange)
		velocities = find_velocities_for_role_change(dronesOnContact, currTime);
	else
		velocities = find_basic_velocities(dronesOnContact, currTime);

	list<int> velocityConfigs;
	if (dronesOnContact.at(0)->get_priority() >= dronesOnContact.at(1)->get_priority())
		for (int i = 1; i <= m_maxSpeedMultiplier * 10; i++)
			velocityConfigs.push_back(-i);

	if (dronesOnContact.at(0)->get_priority() <= dronesOnContact.at(1)->get_priority())
		for (int i = 1; i <= m_maxSpeedMultiplier * 10; i++)
			velocityConfigs.push_back(i);


	list<pair<int, double>> velocityConfigsWithContactEscapeTime;
	for (auto& velocityConfig : velocityConfigs)
	{
		cout << "Curr velocity config: " << velocityConfig << "- ";

		DroneBall* drone = nullptr;
		rg_Point3D velocity;
		if (velocityConfig > 0)
		{
			drone = dronesOnContact.at(0);
			velocity = velocities.at(0)*velocityConfig*0.1;
		}
		else
		{
			drone = dronesOnContact.at(1);
			velocity = velocities.at(1)*-velocityConfig * 0.1;
		}

		double nearestCollisionTime = calculate_nearest_collision_time_with_neighbors(drone, velocity, neighbors, currTime);
		double escapeTime = find_contact_escape_time(drone, velocity, neighbors, currTime);

		cout << "NCT: " << nearestCollisionTime << ", ET: " << escapeTime << " -> ";

		if (nearestCollisionTime > escapeTime)
		{
			cout << "Valid" << endl;
			velocityConfigsWithContactEscapeTime.push_back(make_pair(velocityConfig, escapeTime));
		}
		else
		{
			cout << "Not valid" << endl;
		}
	}

	system("pause");
}



bool SwarmSimulator::is_velocity_config_valid(const int& velocityConfig, const array<DroneBall*, 2> dronesOnContact) const
{
	if (velocityConfig == 0)
		return false;

	if (velocityConfig > 0 && (dronesOnContact.at(0)->get_priority() > dronesOnContact.at(1)->get_priority()))
		return false;

	if (velocityConfig < 0 && (dronesOnContact.at(0)->get_priority() < dronesOnContact.at(1)->get_priority()))
		return false;

	return true;
}



double SwarmSimulator::calculate_nearest_collision_time_with_neighbors(const DroneBall* drone, const rg_Point3D& velocity, const set<DynamicBall*>& neighbors, const double& currTime) const
{
	rg_Point3D position = drone->calculate_position_at_time(currTime);
	double nearestCollisionTime = DBL_MAX;
	for (set<DynamicBall*>::const_iterator itForNeighbor = neighbors.cbegin(); itForNeighbor != neighbors.cend(); itForNeighbor++)
	{
		if ((*itForNeighbor) != drone)
		{
			rg_Point3D relativePosition = (*itForNeighbor)->calculate_position_at_time(currTime) - position;
			rg_Point3D relativeVelocity = (*itForNeighbor)->get_velocity_vector() - velocity;
			
			//if ((*itForNeighbor)->get_ID() == 30)
				//cout << "Will collide drones!" << endl;

			double collisionTime = calculate_collision_time_from_now(relativePosition, relativeVelocity);
			if (0 < collisionTime && collisionTime + currTime < nearestCollisionTime)
			{
				nearestCollisionTime = collisionTime+currTime;
			}
		}
	}
	return nearestCollisionTime;
}



double SwarmSimulator::calculate_collision_time_from_now(const rg_Point3D& relativePosition, const rg_Point3D& relativeVelocity) const
{
	//for debug
	rg_Point3D r = -relativePosition;
	rg_Point3D v = relativeVelocity;
	double minDistance = sqrt(pow(r.magnitude(), 2) - (r.innerProduct(v.getUnitVector())));

	array<double, 2> collisionInterval = calculate_contact_interval_from_now(relativePosition, relativeVelocity, 2 * m_collisionRadius);
	return collisionInterval.at(0);
}



void SwarmSimulator::change_speed_to_avoid_collision(DroneBall* drone, const int& velocityConfig, const double& contactTime, const double& escapeTime)
{
	drone->set_status(AVOIDING);

	const Color3f& droneColor = drone->find_current_WP().color;
	WayPoint WPForSC = make_way_point_at_curr_position(drone, contactTime, AVOIDANCE_START, droneColor);
	drone->add_way_point_at_next(WPForSC);
	drone->increase_WP_index();
	drone->set_velocity_vector(drone->get_velocity_vector()*0.1*velocityConfig);

	adjust_restoration_event_queue(escapeTime, drone);

	update_contact_event_for_neighbors(drone, contactTime);
	update_restoration_event_for_neighbors(drone, contactTime);

	double arrivalTime = drone->calculate_arrival_time_to_next_WP(contactTime);
	adjust_arrival_event_queue(arrivalTime, drone);
	cout << "Avoidance: make drone " << drone->get_ID() << " to move at x" << velocityConfig*0.1 << " speed until " << escapeTime << endl<<endl;

	m_dynamicVD.add_velocity_change_event(drone->get_ID(), contactTime);
	add_velocity_change_history(drone, contactTime);
}



void SwarmSimulator::change_role_to_avoid_collision(const array<DroneBall*, 2> dronesOnContact, const int& velocityConfig, const double& contactTime, const double& escapeTime)
{
	DroneBall* droneBothRCAndSC = nullptr; 
	DroneBall* droneOnlyRC = nullptr;

	int optimalVelocityConfig = velocityConfig * 0.1;
	if (optimalVelocityConfig > 0)
	{
		droneBothRCAndSC = dronesOnContact.at(0);
		droneOnlyRC = dronesOnContact.at(1);
	}
	else
	{
		droneBothRCAndSC = dronesOnContact.at(1);
		droneOnlyRC = dronesOnContact.at(0);
		optimalVelocityConfig = -optimalVelocityConfig;
	}

	WayPoint destinationForDroneBothRCAndSC = droneOnlyRC->get_way_points().back();
	WayPoint destinationForDroneOnlyRC = droneBothRCAndSC->get_way_points().back();

	update_way_points_for_role_change(droneBothRCAndSC, contactTime, destinationForDroneBothRCAndSC, optimalVelocityConfig);
	update_way_points_for_role_change(droneOnlyRC, contactTime, destinationForDroneOnlyRC);

	droneBothRCAndSC->set_status(AVOIDING);
	droneOnlyRC->set_status(NORMAL);

	update_velocity_to_next_way_point(droneBothRCAndSC, contactTime);
	update_velocity_to_next_way_point(droneOnlyRC, contactTime);
	
	adjust_restoration_event_queue(escapeTime, droneBothRCAndSC);
	cout << "Role change: make drone " << droneBothRCAndSC->get_ID() << " to move at x" << optimalVelocityConfig << " speed until " << escapeTime << endl << endl;

	for (int i = 0; i < 2; i++)
	{
		update_contact_event_for_neighbors(dronesOnContact.at(i), contactTime);
		update_restoration_event_for_neighbors(dronesOnContact.at(i), contactTime);
		double arrivalTime = dronesOnContact.at(i)->calculate_arrival_time_to_next_WP(contactTime);
		cout << "New Arrival time for " << dronesOnContact.at(i)->get_ID() << " : " << arrivalTime<<endl;
		adjust_arrival_event_queue(arrivalTime, dronesOnContact.at(i));
		m_dynamicVD.add_velocity_change_event(dronesOnContact.at(i)->get_ID(), contactTime);
	}
}



void SwarmSimulator::update_way_points_for_role_change(DroneBall* drone, const double& currTime, const WayPoint& destinationWP, const double& speedConfig /*= 1.0*/)
{
	rg_Point3D originalDestination = drone->find_destination();
	cout << "Add waypoint for drone" << drone->get_ID() << " : destination changed [" << originalDestination.getX() << ", " << originalDestination.getY() << ", " << originalDestination.getZ()<<"]" ;

	const Color3f& droneColor = drone->find_current_WP().color;
	WayPoint WPForRoleChange = make_way_point_at_curr_position(drone, currTime, ROLE_CHANGE, droneColor);

	drone->add_way_point(drone->get_WP_index()+1, WPForRoleChange);
	drone->increase_WP_index();

	if (drone->get_status() == ARRIVAL)
	{
		drone->add_way_point_at_next(destinationWP);
	}
	else
	{
		WayPoint& currDestinationWP = drone->get_way_points().back();
		currDestinationWP = destinationWP;
	}

	rg_Point3D newDestination = drone->find_destination();
	cout << " -> [" << newDestination.getX() << ", " << newDestination.getY() << ", " << newDestination.getZ() <<"]"<< endl;

	/*cout << "Waypoint list: ";
	for (auto& WP : drone->get_way_points())
	{
		rg_Point3D point = WP.point;
		cout << "[" << point.getX() << ", " << point.getY() << ", " << point.getZ() << "] ";
	}
	cout << endl;*/
}



WayPoint SwarmSimulator::make_way_point_at_curr_position(const DroneBall* drone, const double& time, const WAYPOINT_TYPE type, const Color3f& color)
{
	WayPoint wayPoint;
	wayPoint.type = type;
	wayPoint.point = drone->calculate_position_at_time(time);
	wayPoint.time = time;
	return wayPoint;
}



WayPoint SwarmSimulator::make_arc_way_point(DroneBall& drone, const float& ratio, const double& z)
{
	WayPoint& startWP = drone.get_way_points().front();
	WayPoint& endWP = drone.get_way_points().back();

	WayPoint arcWayPoint;
	arcWayPoint.point = startWP.point*(1-ratio) + endWP.point* ratio + rg_Point3D(0, 0, z);
	arcWayPoint.type = DESTINATION;
	arcWayPoint.time = m_targetArrivalTime * ratio;
	arcWayPoint.color = startWP.color;
	return arcWayPoint;
}



void SwarmSimulator::update_velocity_to_next_way_point(DroneBall* drone, const double& currTime)
{
	rg_Point3D currPosition = drone->calculate_position_at_time(currTime);
	const WayPoint& nextWP = drone->find_next_WP();

	switch (m_pathGenerationMode)
	{
	case CONSTANT_SPEED:
	{
		rg_Point3D velocity = (nextWP.point - currPosition).getUnitVector()*m_basicSpeed;
		drone->set_velocity_vector(velocity);
		break;
	}
	case TIME_ON_TARGET:
	{
		update_velocity_to_arrive_at_target_time(*drone, currTime);
		break;
	}
	default:
		break;
	}
}



void SwarmSimulator::take_by_pass_to_avoid_collision(DroneBall* avoidingDrone, const double& contactTime, const rg_Point3D& avoidanceVelocity)
{
	rg_Point3D positionOfAvoidingDrone = avoidingDrone->calculate_position_at_time(contactTime);
	rg_Point3D velocityOfAvoidingDrone = avoidingDrone->get_velocity_vector();

	/*if (avoidingDrone->get_status() == ARRIVAL)
	{
		WayPoint WPForPausePoint = avoidingDrone->find_final_WP();
		if (m_pathGenerationMode == TIME_ON_TARGET)
			WPForPausePoint.time = m_targetArrivalTime;
		WPForPausePoint.type = PAUSE;
		avoidingDrone->add_way_point_at_current_position(WPForPausePoint);
		avoidingDrone->increase_WP_index();
	}
	else
	{
		WayPoint WPForPausePoint;
		WPForPausePoint.point = positionOfAvoidingDrone;
		WPForPausePoint.time = 0;
		WPForPausePoint.type = PAUSE;
		avoidingDrone->add_way_point_at_next_position(WPForPausePoint);
		avoidingDrone->increase_WP_index();
	}*/

	const Color3f& droneColor = avoidingDrone->find_current_WP().color;
	WayPoint WPForBP = make_way_point_at_curr_position(avoidingDrone, contactTime, AVOIDANCE_START, droneColor);

	if (avoidingDrone->get_status() == ARRIVAL)
	{
		//avoidingDrone->decrease_WP_index();
		WayPoint WPForPausePoint = avoidingDrone->find_final_WP();
		/*if (m_pathGenerationMode == TIME_ON_TARGET)
			WPForPausePoint.time = m_targetArrivalTime;*/
		WPForPausePoint.type = PAUSE;
		avoidingDrone->add_way_point_at_current(WPForPausePoint);
	}
	avoidingDrone->increase_WP_index();
	avoidingDrone->add_way_point_at_current(WPForBP);
	avoidingDrone->set_velocity_vector(avoidanceVelocity);
	
	/*avoidingDrone->add_way_point_at_current(WPForBP);
	avoidingDrone->increase_WP_index();
	avoidingDrone->set_velocity_vector(avoidanceVelocity);*/
	//rg_Point3D targetPoint = avoidingDrone->calculate_position_at_time(minEscapeTime);

	avoidingDrone->set_status(AVOIDING);

	double restorationTime = abs(m_avoidanceAlt / avoidanceVelocity.getZ()) + contactTime;
	adjust_restoration_event_queue(restorationTime, avoidingDrone);

	update_contact_event_for_neighbors(avoidingDrone, contactTime);
	//update_restoration_event_for_neighbors(avoidingDrone, contactTime);

	//DECIMAL arrivalTime = avoidingDrone->calculate_arrival_time_to_next_WP(contactTime);
	//adjust_arrival_event_queue(arrivalTime, drone);
	adjust_arrival_event_queue(DBL_MAX, avoidingDrone);
	
	cout << "Velocity changed for "<<avoidingDrone->get_ID()<<" with vZ " << avoidanceVelocity.getZ() << " at [" << positionOfAvoidingDrone.getX() << " " << positionOfAvoidingDrone.getY() << " " << positionOfAvoidingDrone.getZ() << "] until " << restorationTime << endl;
	
	//cout << "Avoidance: make drone " << avoidingDrone->get_ID() << " to add altitude " << targetPoint.getZ() << " until " << minEscapeTime << endl << endl;

	m_dynamicVD.add_velocity_change_event(avoidingDrone->get_ID(), contactTime);
	add_velocity_change_history(avoidingDrone, contactTime);
}



double SwarmSimulator::calculate_nearest_collision_time_for_candidate_avoidance(DroneBall* avoidingDrone, const double& contactTime, const rg_Point3D& candidateAvoidanceVelocity)
{
	rg_Point3D positionOfAvoidingDrone = avoidingDrone->calculate_position_at_time(contactTime);
	rg_Point3D velocityOfAvoidingDrone = avoidingDrone->get_velocity_vector();

	set<DynamicBall*> neighbors = collect_neighbor_dynamic_balls(avoidingDrone);

	double nearestCollisionTime = calculate_nearest_collision_time_with_neighbors(avoidingDrone, candidateAvoidanceVelocity, neighbors, contactTime);
	return nearestCollisionTime;
}



void SwarmSimulator::print_drone_info(const string& fileName) const
{
	ofstream oFile(fileName);
	oFile << m_drones.size() << "\n";
	double currTime = m_dynamicVD.get_current_time();
	for (vector<DroneBall>::const_iterator it = m_drones.cbegin(); it != m_drones.cend(); it++)
	{
		rg_Point3D center = (*it).calculate_position_at_time(currTime);
		oFile << (*it).get_ID() << "\t" << center.getX() << "\t" << center.getY() << "\t" << center.getZ() << "\n";
	}
	oFile.close();
}
