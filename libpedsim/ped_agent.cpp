//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>

#include <stdlib.h>


Ped::Tagent::Tagent(int posX, int posY) {
	Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
	Ped::Tagent::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
	x = posX;
	y = posY;
	destination = NULL;
	lastDestination = NULL;
}

void Ped::Tagent::computeNextDesiredPosition() {
	destination = getNextDestination();
	if (destination == NULL) {
		// no destination, no need to
		// compute where to move to
		return;
	}

	double diffX = destination->getx() - x;
	double diffY = destination->gety() - y;
	double len = sqrt(diffX * diffX + diffY * diffY);
	desiredPositionX = (int)round(x + diffX / len);
	desiredPositionY = (int)round(y + diffY / len);
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
}
//getNextDestination with vectorization
//Potential flaws: Memory size might not align properly?
Ped::Twaypoint* Ped::Tagent::getNextDestinationVector(int TagentIndex) {
	__m128 xVector, yVector, destinationVector, destinationX, destinationY, diffX, diffY, length, destinationR, 
		nextDestination, agentReachedDestination, nullList, falseList;
	nullList = (float* _mm_malloc(4 * sizeof(float), 16); //Ett försök att aligna data för rätt storlekar
	falseList = (float* _mm_malloc(4 * sizeof(float), 16);
	length = (float* _mm_malloc(4 * sizeof(float), 16);
	diffX = (float* _mm_malloc(4 * sizeof(float), 16);
	diffY = (float* _mm_malloc(4 * sizeof(float), 16);
	xVector = _mm_load_ps(&TagentPointers->x[TagentIndex]);
	yVector = _mm_load_ps(&TagentPointers->y[TagentIndex]);
	destinationX = _mm_load_ps(&TagentPointers->destinationX[TagentIndex]);
	destinationY = _mm_load_ps(&TagentPointers->destinationY[TagentIndex]);
	destinationR = _mm_load_ps(&TagentPointers->lastDestinationR[TagentIndex]);
	nullList = [NULL, NULL, NULL, NULL]; //Fungerar detta?
	falseList = [false, false, false, false];

	nextDestination = _mm_load_ps(&nullList); 
	agentReachedDestination = _mm_load_ps(&falseList);

	if (_mm_cmpneq_ps(destinationX, nullList) { // (destination != NULL) Nu jämför vi enbart med X-koordinat
		diffX = _mm_sub_ps(destinationX, xVector);
	    diffY = mm_sub_ps(destinationY, yVector);
		length = _mm_sqrt_ps(_mm_add_ps((_mm_mul_ps(diffX, diffX), (_mm_mul_ps(diffY, diffY))))); //Fungerar detta?
		agentReachedDestination = _mm_cmplt_ps(length, destinationR); //Returns __m128 single-precision pointer
	}

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) { // waypointsize == 0
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
		    //ladda in waypoint-pekare | waypointPointer = _mm_load_ps(&Vagent->waypointPointer[TagentIndex]);
            //har vi nått slutet på listan? sätt waypoint-pekare till 0, annars + 1
		    //hämta nya waypoints med hjälp av pekaren
	
			waypoints.push_back(destination);
			nextDestination = waypoints.front();
			waypoints.pop_front();
		}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		_mm_store_ps(&) //hm. Hur vill vi spara nextDestination?
		//nextDestination = destination;
	}
}


Ped::Twaypoint* Ped::Tagent::getNextDestination() {
	Ped::Twaypoint* nextDestination = NULL;
	bool agentReachedDestination = false;

	if (destination != NULL) {
		// compute if agent reached its current destination
		double diffX = destination->getx() - x;
		double diffY = destination->gety() - y;
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destination->getr();
	}

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) { //waypointsize == 0
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
		//ladda pekare (destinationPointer)
		//om vi nått slutet på listan, sätt alla till 0 -> annars incrementa med 1
		//Ladda in rätta koordinater med hjälp av pekarna ifrån destination-listan (destinationListX / destinationListY / destinationListR)
		//Uppdatera destinationX och Y koordinater för Vagentstrukten
		waypoints.push_back(destination);
		nextDestination = waypoints.front();
		waypoints.pop_front();
	}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		nextDestination = destination;
	}

	return nextDestination;
}
