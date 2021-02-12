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
#include <emmintrin.h>

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

void Ped::Tagent::computeNextDesiredPosition(Ped::Vagent vagents, int i) {
	
	//will call on the function, but with the arrays
	destination = getNextDestination();
	
	__m128i _null, dest_x, dest_y, mask1, mask2;

	_null = _mm_set1_epi32(0);
	dest_x = _mm_load_si128(&vagents.destinationX[i]); //destination x
	dest_y = _mm_load_ps(&vagents.destinationY[i]); //destination y

	//mask1 if dest_x == null
	mask1 = _mm_cmpeq_epi32(dest_x, _null); // m1 = |11..11|00..00|11..11|00..00| if statement true or false for all 32 bits
	mask2 = _mm_cmpeq_epi32(dest_y, _null); // m1 = |00..00|00..00|11..11|00..00|

	//Want to logically OR them, because we want to know if the destination is null
	//Wherever the destination is null(or zero) the mask will be filled with 0s, so we will preform code below
	//and then logically and them, and so only the agents with destination will have a new destination
	mask1 = _mm_and_si128 (mask1, mask2); // m1 = |11.11|00..00|11..11|00..00|
	
	//int a = _mm_movemask_epi8 (mask1)
	
	if (_mm_movemask_epi8 (mask1) == 0) {
		// no destination, no need to
		// compute where to move to
		return;
	}
	

	__m128 t_x, t_y, dest_id, ldest_id; //ints
	__m128 d_x, d_y, d_r, t_a, t_b; //single floating point

	t_x = _mm_load_ps(&x[i]);   
	t_y = _mm_load_ps(&y[i]);

	//_mm_castpd_ps(t_x); //casts to double from int, only for compilor 
	//_mm_castps_pd(t_y); //

	//__m128d _mm_castps_pd (__m128 a)
	//Cast vector of type __m128 to type __m128d. This intrinsic is only used for compilation and does not generate any instructions, thus it has zero latency.

	d_x = _mm_load_ps(&destinationX[i]); //destination x
	d_y = _mm_load_ps(&destinationY[i]); //destination y

	d_x = _mm_sub_ps(d_x, t_x); //d_x is now diffX = destinationx - x
	d_y = _mm_sub_ps(d_x, t_y); //same

	t_a = _mm_mul_ps(d_x,d_x); //temporary a = diffX*diffX
	t_b = _mm_mul_ps(d_y,d_y); //temporary b diffY*diffY

	t_a = _mm_add_ps(t_a, t_b); // diffX*diffX - diffY*diffY

	t_a = _mm_sqrt_ps(t_a); // len = sqrt(diffX*diffX - diffY*diffY)

	d_x = _mm_div_ps(d_x, t_a); //second part of (int)round(x + diffX / len);
	d_y = _mm_div_ps(d_y, t_a);

	d_x = _mm_add_ps(d_x, t_x); //d_x is now the new x
	d_y = _mm_add_ps(d_y, t_y); //d_y is now the new y

	//_mm_castpd_ps(t_x); //casts to int from double 
	//_mm_castpd_ps(t_y);

	
	//|11.11|00..00|11..11|00..00|
	d_x = _mm_and_si128 (d_x, mask1);
	d_y = _mm_and_si128 (d_y, mask1);


	_mm_store_ps(&destinationX[i], d_x);
	_mm_store_ps(&destinationY[i], d_y);


	//double diffX = destination->getx() - x;
	//double diffY = destination->gety() - y;
	//double len = sqrt(diffX * diffX + diffY * diffY);
	//desiredPositionX = (int)round(x + diffX / len);
	//desiredPositionY = (int)round(y + diffY / len);
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
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

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) {
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
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
