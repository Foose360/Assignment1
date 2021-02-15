#include "vector_agents.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <math.h>
#include <stdlib.h>
<<<<<<< HEAD
#include <emmintrin.h>
=======

>>>>>>> 2056aee134817ff0841816f893d4350766b1a4f1



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