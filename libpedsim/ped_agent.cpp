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

void Ped::Tagent::computeNextDesiredPosition(std::vector<Ped::Tagent*> agents, Ped::Vagent *vagents, int i) {
	__m128i _null, d_x, d_y, mask1, mask2;

	_null = _mm_set1_epi32(NULL); //Oklart om funkar
	_ones = _mm_set1_epi32(1); //TODO: 
	d_x = _mm_load_si128(&vagents->destinationX + i); //destination x
	d_y = _mm_load_si128(&vagents->destinationY + i); //destination y

	//mask1 if dest_x == null
	mask1 = _mm_cmpeq_epi32(d_x, _null); // m1 = |11..11|00..00|11..11|00..00| if statement true or false for all 32 bits
	mask2 = _mm_cmpeq_epi32(d_y, _null); // m1 = |00..00|00..00|11..11|00..00|

	mask1 = _mm_and_si128(mask1, mask2); // m1 = |11.11|00..00|11..11|00..00|

		//int a = _mm_movemask_epi8 (mask1)

	if (_mm_movemask_epi8(mask1) == _ones) {
		// no destination, no need to if destination == NULL { return}
		// compute where to move to
		return;
	}

	__m128i t_x, t_y, dest_id, ldest_id; //ints
	__m128 d_r, t_a, t_b, ps_x, ps_y; //single floating point

	t_x = _mm_load_si128(&vagents->x + i);
	t_y = _mm_load_si128(&vagents->y + i);
	d_x = _mm_sub_epi32(d_x, t_x); //d_x is now diffX = destinationx - x
	d_y = _mm_sub_epi32(d_x, t_y); //same

	ps_x = _mm_castsi128_ps(d_x);
	ps_y = _mm_castsi128_ps(d_y);

	t_a = _mm_mul_ps(ps_x, ps_x); //temporary a = diffX*diffX
	t_b = _mm_mul_ps(ps_y, ps_y); //temporary b diffY*diffY

	t_a = _mm_add_ps(t_a, t_b); // diffX*diffX - diffY*diffY

	t_a = _mm_sqrt_ps(t_a); // len = sqrt(diffX*diffX - diffY*diffY)

	ps_x = _mm_div_ps(ps_x, t_a); //second part of (int)round(x + diffX / len);
	ps_y = _mm_div_ps(ps_y, t_a);

	_mm_castps_si128(ps_x); //casts to int from float 
	_mm_castps_si128(ps_y);

	t_x = _mm_add_epi32(d_x, t_x); //t_x is now the new x
	t_y = _mm_add_epi32(d_y, t_y); //t_y is now the new y


	//|11.11|00..00|11..11|00..00|
	t_x = _mm_and_si128(t_x, mask1);
	t_y = _mm_and_si128(t_y, mask1);


	_mm_store_si128(&vagents->destinationX + i, t_x);
	_mm_store_si128(&vagents->destinationY + i, t_y);

	for (k = 0; k < 4; k++) {
		agents[i]->setX(&vagents->destinationX + i + k);
		agents[i]->setY(&vagents->destinationY + i + k);
	}
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
}

void Ped::Tagent::destinationReached(Ped::Vagent* agents, int i) {

	__m128i _null, d_x, d_y, d_r, _reached, _x, _y, _len;
	__m128 ps_x, ps_y, t_a, t_b;
	// compute if agent reached its current destination

	d_x = _mm_load_si128(&vagents->destinationX + i); //destination x
	d_y = _mm_load_si128(&vagents->destinationY + i); //destination y
	d_r = _mm_load_si128(&vagents->destinationR + i); //destination r
	_x = _mm_load_si128(&vagents->x + i);
	_y = _mm_load_si128(&vagents->y + i);

	d_x = _mm_sub_epi32(d_x, _x); //d_x is now diffX = destinationx - x
	d_y = _mm_sub_epi32(d_x, _y); //same

	ps_x = _mm_castsi128_ps(d_x); //float version of diffx and diffy
	ps_y = _mm_castsi128_ps(d_y);

	t_a = _mm_mul_ps(ps_x, ps_x); //temporary a = diffX * diffX
	t_b = _mm_mul_ps(ps_y, ps_y); //temporary b = diffY * diffY

	t_a = _mm_add_ps(t_a, t_b); // diffX*diffX + diffY*diffY

	t_a = _mm_sqrt_ps(t_a); // len = sqrt(diffX*diffX - diffY*diffY)

	_len = _mm_castps_si128(t_a); //cast to int

	_reached = _mm_cmpgt_epi32(d_r, _len); //mask telling if agent
	_mm_store_si128(&vagents->reachedDestination + i, _reached);
}


void Ped::Tagent::getNextDestination(Ped::Vagent* agents, int i) {
	Ped::Twaypoint* nextDestination = NULL;
	int k = i + 4;
	int newi = i;
	for (newi; newi < k; newi++) {
		int agentReachedDestination = agents->destinationReached + newi;
		bool check;
		if (agentReachedDestination > 0) {
		check = true;
	    }
		else {
			check = false;
		}
		if ((check || (agents->destinationX + newi) == NULL) && !waypoints.empty()) { //waypointsize == 0
			// Case 1: agent has reached destination (or has no current destination);
			// get next destination if available
			waypoints.push_back(destination);
			nextDestination = waypoints.front();
			waypoints.pop_front();
			agents->destinationX + newi = nextDestination->getx();
			agents->destinationY + newi = nextDestination->gety();
			agents->destinationR + newi = nextDestination->getr();
			agents->destinationID + newi = nextDestination->getid();
			//uppdatera pekarvärde!
		}
		else {
			// Case 2: agent has not yet reached destination, continue to move towards
			// current destination
			agents->destinationX + newi = nextDestination->getx();
			agents->destinationY + newi = nextDestination->gety();
			agents->destinationR + newi = nextDestination->getr();
			agents->destinationID + newi = nextDestination->getid();
			//nextDestination = destination;
		}
	}
}
