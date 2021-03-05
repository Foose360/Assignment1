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
#include "cuda_runtime.h"

static int findRegion(int x, int y){
	if(x <= 80 && y <= 60 ){
		return 0;
	}
	else if(x > 80 && y <= 60)
	{
		return 1;
	}
	else if (x <= 80 && y > 60){
		return 2;
	}
	else if (x > 80 && y > 60){
		return 3;
	}
	else{
		std::cerr << "Error in findRegion." << std::endl;
		return -1;
	}
}


std::vector<std::vector<int>> Ped::Model::placeAgents() {
	int index;		
	std::vector<std::vector<int>> tmpvector;
	std::vector<int> a;
	std::vector<int> b;
	std::vector<int> c;
	std::vector<int> d;
	int restProducts = this->agents.size() % 4;
	//AGENTS = [R1, R2, R3, R4]
	for (int i = 0; i < agents.size()-restProducts; i++) {
		index = findRegion(agents[i]->getX(), agents[i]->getY());
		agents[i]->setRegion(index);
		agents[i]->setIndex(i);
		if (index == 0) {
			a.push_back(i);
		} else if (index == 1) {
			b.push_back(i);
		} else if (index == 2) {
			c.push_back(i);
		} else {
			d.push_back(i);
		}
		
	}
	tmpvector.push_back(a);
	tmpvector.push_back(b);
	tmpvector.push_back(c);
	tmpvector.push_back(d);
	return tmpvector;
}


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

	regionAgents = placeAgents();
	// Sets the chosen implemenation. Standard in the given code is SEQ
	this->implementation = implementation;

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();
}




// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
bool Ped::Model::move(Ped::Tagent *agent, bool addToagentQueue)
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
		bool neighbors = getNeighbors(std::get<0>(p), std::get<1>(p), 1, std::get<2>(p));
		if(neighbors){
			if (agent->getRegion() != std::get<2>(p)) {
				if (addToagentQueue) { //If it is being run in parallel, add to agentQueue, else update.
					agentQueue.push_back(agent->getIndex());
					return false;
				} else { //Updates agents in agentQueue
					agent->setX(std::get<0>(p));
					agent->setY(std::get<1>(p));
					std::vector<int>::iterator position = std::find(regionAgents[agent->getRegion()].begin(), regionAgents[agent->getRegion()].end(), agent->getIndex());
                                        if (position != regionAgents[agent->getRegion()].end()) {
                                                this->regionAgents[agent->getRegion()].erase(position);

                                        }
					agent->setRegion(std::get<2>(p));
					regionAgents[std::get<2>(p)].push_back(agent->getIndex());
					return true;
				}
				
			} else {
				agent->setX(std::get<0>(p));
				agent->setY(std::get<1>(p));
				return true;
			}						
		}
	}
	return false;
}


void Ped::Model::delegateTasks() {
#pragma omp parallel sections
{
#pragma omp section
{
        moveOmp(0);
}
#pragma omp section
{
        moveOmp(1);
}
#pragma omp section
{
        moveOmp(2);
}
#pragma omp section
{
        moveOmp(3);
}
}
}
void Ped::Model::moveOmp(int region) {
       for (int i = 0; i < this->regionAgents[region].size(); i++) {
                int index = this->regionAgents[region][i];
                if (move((this->agents)[index], true)) {
                        this->vagents->x[index] = this->agents[index]->getX();
                        this->vagents->y[index] = this->agents[index]->getY();
                }
        }
}

void Ped::Model::tick_omp()
{
	int i;
	Ped::Tagent* tmp;   
	int restProducts = agents.size() % 4;
#pragma omp parallel for private(i)
	for (i = 0; i < this->agents.size()-restProducts; i+=4) {
		this->vagents->destinationReached(i);
		this->vagents->getNextDestination(&agents, i);
		this->vagents->computeNextDesiredPosition(&agents, i);
	}
#pragma omp barrier   
	delegateTasks();
#pragma omp barrier
	#pragma omp critical
	for (int k = 0; k < agentQueue.size(); k++) {		
		int index = agentQueue[k];
		if (move((agents)[index], false)) { //Serialized				
			this->vagents->x[index] = this->agents[index]->getX();
			this->vagents->x[index] = this->agents[index]->getX();
		}
	}
	this->agentQueue.clear();
}



void Ped::Model::tick_serial()
{
	int i;
	Ped::Tagent* tmp;
	int restProducts = agents.size() % 4;
	for (i = 0; i < agents.size()-restProducts; i+=4) { 
		this->vagents->destinationReached(i);
		this->vagents->getNextDestination(&agents, i);
		this->vagents->computeNextDesiredPosition(&agents, i);
	}

	cuda_updateHeatmapSeq();

    for(int k = 0; k < agents.size()-restProducts; k++){
		if (move(agents[k], false)) {
			this->vagents->x[k] = agents[k]->getX();
			this->vagents->y[k] = agents[k]->getY();
			
		}
	}
	for(int i = 0; i < agentQueue.size(); i++) {
		int index = agentQueue[i];
		if (move(agents[index], false)) {
			this->vagents->x[index] = agents[index]->getX();
			this->vagents->y[index] = agents[index]->getY();
		}
		
	}

	// Uppsammling av CPU/GPU
	
	agentQueue.clear();
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
			if (move((*agents)[k]), false) {
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
bool Ped::Model::getNeighbors(int x, int y, int dist, int region) const {
       for (int i = 0; i < this->regionAgents[region].size(); i++ ){
                int index = this->regionAgents[region][i];
                int agentX = (agents)[index]->getX();
                int agentY = (agents)[index]->getY();
                if ((agentX - x) <= dist && (agentX - x) >= 0) {
                        if ((agentY - y) <= dist && (agentY - y) >= 0){
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
