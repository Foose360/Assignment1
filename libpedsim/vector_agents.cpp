#include "vector_agents.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <math.h>
#include <stdlib.h>
#include <emmintrin.h>

Ped::Vagent::Vagent(Ped::Model mod) {
	Ped::Vagent::init(mod);
}

void Ped::Vagent::init(Ped::Model mod) {

    std::vector<Tagent*> agents = mod.getAgents();
    std:size_t s = agents.size();

	int *x = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	int *y = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.

    int *reachedDestination = (bool *)_mm_malloc(s * sizeof(bool), 16);
	int *destinationId = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	float *destinationX = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	float *destinationY = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	float *destinationR = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:S på rad.

	int *LastdestinationId = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	float *LastdestinationX = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	float *LastdestinationY = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:s på rad.
	float *LastdestinationR = (float *)_mm_malloc(s * sizeof(float), 16); // pekare till double:S på rad.

    Ped::Tagent* tmp;

    int *c1 = x;
    int *c2 = y;
    int *b1 = reachedDestination;
    int *d1 = destinationId;
    float *d2 = destinationX;
    float *d3 = destinationY;
    float *d4 = destinationR;

    int *d5 = LastdestinationId;
    float *d6 = LastdestinationX;
    float *d7 = LastdestinationY;
    float *d8 = LastdestinationR;

    for (int i = 0; i < s; i++) {
		tmp = agents[i];

        // iterate all the values
        b1 = b1 + 1;
        c1 = c1 + 1;
        c2 = c2 + 1;
        d1 = d1 + 1;
        d2 = d2 + 1;
        d3 = d3 + 1;
        d4 = d4 + 1;
        d5 = d5 + 1;
        d6 = d6 + 1;
        d7 = d7 + 1;
        d8 = d8 + 1;

        // set all the values
        *b1 = 0;
        *c1 = tmp->getX();
        *c2 = tmp->getY();
        *d1 = tmp->getDest()->getid();
        *d2 = tmp->getDest()->getx();
        *d3 = tmp->getDest()->gety();
        *d4 = tmp->getDest()->getr();
        *d5 = NULL;
        *d6 = NULL;
        *d7 = NULL;
        *d8 = NULL;
    }
}

void Ped::Vagent::destinationReached(int i) {

	__m128i _null, d_x, d_y, d_r, _reached, _x, _y, _len;
	__m128 ps_x, ps_y, t_a, t_b;
	// compute if agent reached its current destination

	d_x = _mm_load_si128(this->destinationX + i); //destination x
	d_y = _mm_load_si128(this->destinationY + i); //destination y
	d_r = _mm_load_si128(this->destinationR + i); //destination r
	_x = _mm_load_si128(this->x + i);
	_y = _mm_load_si128(this->y + i);

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
	_mm_store_si128(this->reachedDestination + i, _reached);
}

void Ped::Vagent::computeNextDesiredPosition(std::vector<Ped::Tagent*> tagents, int i) {
	__m128i _null, d_x, d_y, mask1, mask2;

	_null = _mm_set1_epi32(NULL); //Oklart om funkar
	_ones = _mm_set1_epi32(1); //TODO: 
	d_x = _mm_load_si128(this->destinationX + i); //destination x
	d_y = _mm_load_si128(this->destinationY + i); //destination y

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

	t_x = _mm_load_si128(this->x + i);
	t_y = _mm_load_si128(this->y + i);
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


	_mm_store_si128(this->destinationX + i, t_x);
	_mm_store_si128(this->destinationY + i, t_y);

	for (int k = 0; k < 4; k++) {
        float *tmpDestX = this->destinationX + (i + k);
        float *tmpDestY = this->destinationY + (i + k);
		tagents[i]->setX(*tmpDestX);
		tagents[i]->setY(*tmpDestY);
	}
}

void Ped::Vagent::getNextDestination(std::vector<Ped::Tagent*> tagents, int i) {
	Ped::Twaypoint* nextDestination = NULL;
	int k = i + 4;
	for (i; i < k; i++) {
        Ped::Tagent* agent = tagents[i];
        // get the waypoints deque of the agent.
        deque<Twaypoint*> waypoints = agent->getWaypoints();
		int agentReachedDestination = *this->reachedDestination + i;   /// TODO: is this correct?
		bool check;
		if (agentReachedDestination > 0) {
		    check = true;
	    }
		else {
			check = false;
		}
		if ((check || (this->destinationX + i) == NULL) && !waypoints.empty()) { //waypointsize == 0
			// Case 1: agent has reached destination (or has no current destination);
			// get next destination if available
			waypoints.push_back(agent->getDest());
			nextDestination = waypoints.front();
			waypoints.pop_front();

			float *tmpDestX = this->destinationX + i;
            float *tmpDestY = this->destinationY + i;
            float *tmpDestR = this->destinationR + i;
            int *tmpDestId = this->destinationId + i;
            *tmpDestX = (float)nextDestination->getx();
            *tmpDestY = (float)nextDestination->gety();
            *tmpDestR = (float)nextDestination->getr();
            *tmpDestId = (float)nextDestination->getid();
			//uppdatera pekarvärde!
		}
		else {
			// Case 2: agent has not yet reached destination, continue to move towards
			// current destination
			float *tmpDestX = this->destinationX + i;
            float *tmpDestY = this->destinationY + i;
            float *tmpDestR = this->destinationR + i;
            int *tmpDestId = this->destinationId + i;
            *tmpDestX = (float)nextDestination->getx();
            *tmpDestY = (float)nextDestination->gety();
            *tmpDestR = (float)nextDestination->getr();
            *tmpDestId = (float)nextDestination->getid();

			//nextDestination = destination; ////////// TODO: Är denna viktig?
		}
	}
}