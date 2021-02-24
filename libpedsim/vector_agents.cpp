#include "vector_agents.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>
#include <stdlib.h>
#include <emmintrin.h>

#include <iostream>

void Ped::Vagent::init(std::vector<Ped::Tagent*> tagents) {

    std:size_t s = tagents.size();

	x = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till int:s på rad.
	y = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till int:s på rad.

    reachedDestination = (float *)_mm_malloc(s * sizeof(float), 16);
	destinationX = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	destinationY = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	destinationR = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:S på rad.

	LastdestinationX = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	LastdestinationY = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	LastdestinationR = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:S på rad.

    Ped::Tagent* tmp;

    float *c1 = x;
    float *c2 = y;
    float *b1 = reachedDestination;
    float *d2 = destinationX;
    float *d3 = destinationY;
    float *d4 = destinationR;

    float *d6 = LastdestinationX;
    float *d7 = LastdestinationY;
    float *d8 = LastdestinationR;

    for (int i = 0; i < s; i++) {
		tmp = tagents[i];

        // set all the values
        b1[i] = 1;
        c1[i] = (float)tmp->getX();
        c2[i] = (float)tmp->getY();
        d2[i] = 0;
        d3[i] = 0;
        d4[i] = 0;
        d6[i] = 0;
        d7[i] = 0;
        d8[i] = 0;

    }
}

Ped::Vagent::Vagent(std::vector<Ped::Tagent*> tagents) {
	Ped::Vagent::init(tagents);
}

void Ped::Vagent::destinationReached(int i) {
	__m128 dest_x, dest_y, dest_r, ps_x, ps_y, tmp_a, tmp_b, _reached, _x, _y, d_x, d_y;
	// compute if agent reached its current destination

	dest_x = _mm_load_ps(this->destinationX + i); //destination x
	dest_y = _mm_load_ps(this->destinationY + i); //destination y
	dest_r = _mm_load_ps(this->destinationR + i); //destination r

	_x = _mm_load_ps(this->x + i);
    _y = _mm_load_ps(this->y + i);

	d_x = _mm_sub_ps(dest_x, _x); //dest_x is now diffX = destinationx - x
	d_y = _mm_sub_ps(dest_y, _y); //same

	tmp_a = _mm_mul_ps(d_x, d_x); //temporary a = diffX * diffX
	tmp_b = _mm_mul_ps(d_y, d_y); //temporary b = diffY * diffY

	tmp_a = _mm_add_ps(tmp_a, tmp_b); // diffX*diffX + diffY*diffY

	tmp_a = _mm_sqrt_ps(tmp_a); // len = sqrt(diffX*diffX - diffY*diffY)

	_reached = _mm_cmpgt_ps(dest_r, tmp_a); //mask telling if agent
	_mm_store_ps(this->reachedDestination + i, _reached);
}

void Ped::Vagent::computeNextDesiredPosition(std::vector<Ped::Tagent*> *tagents, int i) {
    __m128 dest_x, dest_y, mask1, mask2, _null;
    _null = _mm_set1_ps(0);
    dest_x = _mm_load_ps(this->destinationX + i); //destination x
    dest_y = _mm_load_ps(this->destinationY + i); //destination y



    //mask1 if dest_x == null
    mask1 = _mm_cmpneq_ps(dest_x, _null); // m1 = |11..11|00..00|11..11|00..00| if statement true or false for all 32 bits
    mask2 = _mm_cmpneq_ps(dest_y, _null); // m1 = |00..00|00..00|11..11|00..00|

    mask1 = _mm_and_ps(mask1, mask2); // m1 = |11.11|00..00|11..11|00..00|

            //int a = _mm_movemask_epi8 (mask1)

    if (_mm_movemask_ps(mask1) == 0) {
        // no destination, no need to if destination == NULL { return}
        // compute where to move to
        return;
    }

    __m128i dest_id, ldest_id, int_dest_x, int_dest_y; //ints
    __m128 dest_r, tmp_a, tmp_b, ps_x, ps_y, _x, _y, d_x, d_y; //single floating point

    _x = _mm_load_ps(this->x + i);
    _y = _mm_load_ps(this->y + i);

    d_x = _mm_sub_ps(dest_x, _x); //dest_x is now diffX = destinationx - x
    d_y = _mm_sub_ps(dest_y, _y);

    tmp_a = _mm_mul_ps(d_x, d_x); //temporary a = diffX*diffX
    tmp_b = _mm_mul_ps(d_y, d_y); //temporary b diffY*diffY

    tmp_a = _mm_add_ps(tmp_a, tmp_b); // diffX*diffX - diffY*diffY

    tmp_a = _mm_sqrt_ps(tmp_a); // len = sqrt(diffX*diffX - diffY*diffY)

    ps_x = _mm_div_ps(d_x, tmp_a); //second part of (int)round(x + diffX / len);
    ps_y = _mm_div_ps(d_y, tmp_a);

    _x = _mm_add_ps(ps_x, _x); //t_x is now the new x
    _y = _mm_add_ps(ps_y, _y); //t_y is now the new y

    _mm_store_ps(this->x + i, _x);
    _mm_store_ps(this->y + i, _y);
    //|11.11|00..00|11..11|00..00|
    //_x = _mm_and_ps(_x, mask1);
    //_y = _mm_and_ps(_y, mask1);


    //_mm_store_ps(this->destinationX + i, _x);
    //_mm_store_ps(this->destinationY + i, _y); Varför ville vi stora x och y->destination??????


    /// Update the agent to keep it synced. Could be refactored away.
    for (int k = i; k < i+4; k++) {
      int tmpDesiredX = (int)(this->x[k]);
      int tmpDesiredY = (int)(this->y[k]);
        (*tagents)[k]->setDesiredX(tmpDesiredX);
        (*tagents)[k]->setDesiredY(tmpDesiredY);
    }
}


void Ped::Vagent::getNextDestination(std::vector<Ped::Tagent*> *tagents, int i) {
    Ped::Twaypoint* nextDestination = NULL;
	int k = i + 4;
	for (int a = i; a < k; a++) {
        Ped::Tagent* agent = (*tagents)[a];
        // get the waypoints deque of the agent.
        deque<Twaypoint*> *waypoints = agent->getWaypoints();
		int agentReachedDestination = *(this->reachedDestination + a);
		bool check;
		if (agentReachedDestination != 0) {
		    check = true;
	    }
		else {
			check = false;
		}
        if ((check || *(this->destinationX + a) == 0) && !waypoints->empty()) {
            // Case 1: agent has reached destination (or has no current destination);
            // get next destination if available
            waypoints->push_back(agent->getDest()); //Stämmer verkligen detta?
            nextDestination = waypoints->front();
            waypoints->pop_front();
            //agent->setDestination(nextDestination); //// Unessesary?
            //uppdatera pekarvärde!
            if (nextDestination != NULL) {
	      agent->setDestination(nextDestination);
              *(this->destinationX + a) = (float)nextDestination->getx();
              *(this->destinationY + a) = (float)nextDestination->gety();
              *(this->destinationR + a) = (float)nextDestination->getr();
            }
        }
		//else {
			// Case 2: agent has not yet reached destination, continue to move towards
			// current destination
        //    Twaypoint* dest = agent->getDest();
        //    *(this->destinationX + i) = (float)dest->getx();
        //    *(this->destinationY + i) = (float)dest->gety();
        //    *(this->destinationR + i) = (float)dest->getr();
		//}
	}
}
