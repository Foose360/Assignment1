#include "vector_agents.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>
#include <stdlib.h>
#include <emmintrin.h>

void Ped::Vagent::init(std::vector<Ped::Tagent*> agents) {

    std:size_t s = agents.size();

	int *x = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	int *y = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.

    int *reachedDestination = (int *)_mm_malloc(s * sizeof(int), 16);
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

        // set all the values
        *b1 = 0;
        *c1 = tmp->getX();
        *c2 = tmp->getY();
        *d1 = 0;
        *d2 = 0;
        *d3 = 0;
        *d4 = 0;
        *d5 = 0;
        *d6 = 0;
        *d7 = 0;
        *d8 = 0;

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

    }
}

Ped::Vagent::Vagent(std::vector<Ped::Tagent*> agents) {
	Ped::Vagent::init(agents);
}

void Ped::Vagent::destinationReached(int i) {
	__m128i _x, _y, _reached_int;
	__m128 dest_x, dest_y, dest_r, ps_x, ps_y, tmp_a, tmp_b, _reached;
	// compute if agent reached its current destination

	dest_x = _mm_load_ps(this->destinationX + i); //destination x
	dest_y = _mm_load_ps(this->destinationY + i); //destination y
	dest_r = _mm_load_ps(this->destinationR + i); //destination r

    int *xp = this->x + i;
	_x = _mm_load_si128((__m128i*)xp);
    int *yp = this->y + i;
	_y = _mm_load_si128((__m128i*)yp);

	//ps_x = _mm_castsi128_ps(_x); //float version of x and y
	//ps_y = _mm_castsi128_ps(_y);

	dest_x = _mm_sub_ps(dest_x, ps_x); //dest_x is now diffX = destinationx - x
	dest_y = _mm_sub_ps(dest_x, ps_y); //same

	tmp_a = _mm_mul_ps(dest_x, dest_x); //temporary a = diffX * diffX
	tmp_b = _mm_mul_ps(dest_y, dest_y); //temporary b = diffY * diffY

	tmp_a = _mm_add_ps(tmp_a, tmp_b); // diffX*diffX + diffY*diffY

	tmp_a = _mm_sqrt_ps(tmp_a); // len = sqrt(diffX*diffX - diffY*diffY)

	_reached = _mm_cmpgt_ps(dest_r, tmp_a); //mask telling if agent
	_reached_int = _mm_castps_si128(_reached);
    int *rdp = this->reachedDestination + i;
	_mm_store_si128((__m128i*)rdp, _reached_int);
}

void Ped::Vagent::computeNextDesiredPosition(std::vector<Ped::Tagent*> tagents, int i) {
    __m128 dest_x, dest_y, mask1, mask2, _null;
    _null = _mm_set1_ps(0); //Oklart om funkar
    dest_x = _mm_load_ps(this->destinationX + i); //destination x
    dest_y = _mm_load_ps(this->destinationY + i); //destination y

    //mask1 if dest_x == null
    mask1 = _mm_cmpeq_ps(dest_x, _null); // m1 = |11..11|00..00|11..11|00..00| if statement true or false for all 32 bits
    mask2 = _mm_cmpeq_ps(dest_y, _null); // m1 = |00..00|00..00|11..11|00..00|

    mask1 = _mm_and_ps(mask1, mask2); // m1 = |11.11|00..00|11..11|00..00|

            //int a = _mm_movemask_epi8 (mask1)

    if (_mm_movemask_ps(mask1) == 0) {
        // no destination, no need to if destination == NULL { return}
        // compute where to move to
        return;
    }

    __m128i _x, _y, dest_id, ldest_id, int_dest_x, int_dest_y; //ints
    __m128 dest_r, tmp_a, tmp_b, ps_x, ps_y, float_x, float_y; //single floating point

    int *xp = this->x + i;
    _x = _mm_load_si128((__m128i*)xp);
    int *yp = this->x + i;
    _y = _mm_load_si128((__m128i*)yp);

    ps_x = _mm_castsi128_ps(_x);
    ps_y = _mm_castsi128_ps(_y);

    dest_x = _mm_sub_ps(dest_x, ps_x); //dest_x is now diffX = destinationx - x
    dest_y = _mm_sub_ps(dest_y, ps_y);

    tmp_a = _mm_mul_ps(dest_x, dest_x); //temporary a = diffX*diffX
    tmp_b = _mm_mul_ps(dest_y, dest_y); //temporary b diffY*diffY

    tmp_a = _mm_add_ps(tmp_a, tmp_b); // diffX*diffX - diffY*diffY

    tmp_a = _mm_sqrt_ps(tmp_a); // len = sqrt(diffX*diffX - diffY*diffY)

    ps_x = _mm_div_ps(dest_x, tmp_a); //second part of (int)round(x + diffX / len);
    ps_y = _mm_div_ps(dest_y, tmp_a);

    float_x = _mm_castsi128_ps(_x);
    float_y = _mm_castsi128_ps(_y);

    float_x = _mm_add_ps(ps_x, float_x); //t_x is now the new x
    float_y = _mm_add_ps(ps_y, float_y); //t_y is now the new y

    //|11.11|00..00|11..11|00..00|
    float_x = _mm_and_ps(float_x, mask1);
    float_y = _mm_and_ps(float_y, mask1);


    _mm_store_ps(this->destinationX + i, float_x);
    _mm_store_ps(this->destinationY + i, float_y);

    for (int k = 0; k < 4; k++) {
        float* tmpDestX = this->destinationX + (i + k);
        float* tmpDestY = this->destinationY + (i + k);
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