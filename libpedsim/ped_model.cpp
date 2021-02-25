//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include "cuda_testkernel.h"
#include <omp.h>
#include <thread>
#include <stdlib.h>

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario, IMPLEMENTATION implementation)
{
	// Convenience test: does CUDA work on this machine?
	cuda_test();

	// Set 
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());

	// initilizes the region locks
	for(int i = 0; i < 4; i++){
		omp_init_lock(&regionLocks[i]);
	}

	// Set up destinations
	destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	// Sets the chosen implemenation. Standard in the given code is SEQ
	this->implementation = implementation;

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();
}

static int findRegion(int x, int y){
	if(x <= 400 && y <= 300 ){
		return 0;
	}
	else if(x > 400 && y <= 300)
	{
		return 1;
	}
	else if (x <= 400 && y > 300){
		return 2;
	}
	else if (x > 400 && y > 300){
		return 3;
	}
	else{
		std::cerr << "Error in findRegion." << std::endl;
		return -1;
	}
}


// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
bool Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::tuple<int, int, int> > prioritizedAlternatives;
	std::tuple<int, int, int> pDesired(agent->getDesiredX(), agent->getDesiredY(), findRegion(agent->getDesiredX(), agent->getDesiredY()));
	prioritizedAlternatives.push_back(pDesired);

	int diffX = std::get<0>(pDesired) - agent->getX();
	int diffY = std::get<1>(pDesired) - agent->getY();
	std::tuple<int, int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::tuple<int, int, int>(std::get<0>(pDesired) + diffY, std::get<1>(pDesired) + diffX, findRegion(std::get<0>(pDesired) + diffY, std::get<1>(pDesired) + diffX));
		p2 = std::tuple<int, int, int>(std::get<0>(pDesired) - diffY, std::get<1>(pDesired) - diffX, findRegion(std::get<0>(pDesired) - diffY, std::get<1>(pDesired) - diffX));
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::tuple<int, int, int>(std::get<0>(pDesired), agent->getY(), findRegion(std::get<0>(pDesired), agent->getY()));
		p2 = std::tuple<int, int, int>(agent->getX(), std::get<1>(pDesired), findRegion(agent->getX(), std::get<1>(pDesired)));
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	for(int i = 0; i < 3; i++){
		std::tuple<int, int, int> p = prioritizedAlternatives[i];
		// ta lås för area std::get<2>(p)
		omp_set_lock(&regionLocks[std::get<2>(p)]);
		bool neighbors = getNeighbors(std::get<0>(p), std::get<1>(p), 1);
		if(neighbors){
			agent->setX(std::get<0>(p));
			agent->setY(std::get<1>(p));
			omp_unset_lock(&regionLocks[std::get<2>(p)]);
			return true;
		}
		else{
			omp_unset_lock(&regionLocks[std::get<2>(p)]);
		}
	}
	return false;
}


void Ped::Model::tick_omp()
{
	int i;
	Ped::Tagent* tmp;
	agents = this->getAgents();
	int restProducts = agents.size() & 4;
#pragma omp parallel for private(i, tmp)
	for (i = 0; i < agents.size()-restProducts; i+=4) {
		this->vagents->destinationReached(i);
		this->vagents->getNextDestination(&agents, i);
		this->vagents->computeNextDesiredPosition(&agents, i);
		for(int k = i; k < i+4; k++){
			if (move(agents[k])) {
				this->vagents->x[k] = agents[k]->getX();
				this->vagents->y[k] = agents[k]->getY();
		    
			}
		}
	}
}

void Ped::Model::tick_serial()
{
	int i;
	Ped::Tagent* tmp;
	agents = this->getAgents();
	int restProducts = agents.size() % 4;
	for (i = 0; i < agents.size()-restProducts; i+=4) { //Högst temporärt, men nu kan serial iaf köra på graph och --timing-mode.. (graph kör på 458 agents)
		this->vagents->destinationReached(i);
		this->vagents->getNextDestination(&agents, i);
		this->vagents->computeNextDesiredPosition(&agents, i);

		for(int k = i; k < i+4; k++){
			if (move(agents[k])) {
				this->vagents->x[k] = agents[k]->getX();
				this->vagents->y[k] = agents[k]->getY();
		    
			}
		}
	}
}

static void tick_offset(int id, int step, std::vector<Ped::Tagent*> *agents, Ped::Vagent *vagents)
{
	Ped::Tagent* tmp;
	int offset = id * step;
	for (int i = offset; i < offset + step; i+=4) {
		vagents->destinationReached(i);
		vagents->getNextDestination(agents, i);
		vagents->computeNextDesiredPosition(agents, i);
		for(int k = i; k < i+4; k++){
			if (move((*agents)[k])) {
				vagents->x[k] = (*agents)[k]->getX();
				vagents->y[k] = (*agents)[k]->getY();
		    
			}
		}
	}
}

void Ped::Model::tick_threads(int cores)
{
	int i;
	agents = this->getAgents();
	int restProducts = agents.size() % (4 * cores);
	int step = (agents.size()-restProducts) / cores;
	std::thread* t = new::std::thread[cores];
	for (i = 0; i < cores; i++) {
		t[i] = std::thread(tick_offset, i, step, &agents, this->vagents);
	}
	for (int k = 0; k < cores; k++) {
		t[k].join();
	}
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \return  A bool corresponding to whether a spot is free or not.
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
bool Ped::Model::getNeighbors(int x, int y, int dist) const {
	std::vector<Tagent*> agents = this->getAgents();
	//Greedy, naive solution
	//We still iterate though the entire list of agents.
	//But now there is actually some collision checking..
	for (int i = 0; i < agents.size(); i++ ){
		int agentX = agents[i]->getX();
		int agentY = agents[i]->getY();
		if ((agentX - x) <= dist && (agentX - x) >= 0) {
			if ((agentY - y) < dist && (agentY - y) > 0){
 				return false;
 			}
		}
	}
	return true;
}
void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });
}
