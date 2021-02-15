#include "vector_agents.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <math.h>
#include <stdlib.h>




Ped::Vagent::Vagent(Ped::Model mod) {
	Ped::Vagent::init(mod);
}

void Ped::Vagent::init(Ped::Model mod) {

    std::vector<Tagent*> agents = mod.getAgents();
    std:size_t s = agents.size();
    std::vector<Twaypoint*> waypoints = mod.getDestinations();
    destinationSize = waypoints.size();

    double* destinationListX = (double*)_mm_malloc(destinationSize * sizeof(double), 16);
    double* destinationListY = (double*)_mm_malloc(destinationSize * sizeof(double), 16);
    double* destinationListR = (double*)_mm_malloc(destinationSize * sizeof(double), 16);

	int *x = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	int *y = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.

    int *waypointPointer = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till waypointindex

	int *destinationId = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	double *destinationX = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:s på rad.
	double *destinationY = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:s på rad.
	double *destinationR = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:S på rad.

	int *LastdestinationId = (int *)_mm_malloc(s * sizeof(int), 16); // pekare till int:s på rad.
	double *LastdestinationX = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:s på rad.
	double *LastdestinationY = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:s på rad.
	double *LastdestinationR = (double *)_mm_malloc(s * sizeof(double), 16); // pekare till double:S på rad.

    Ped::Tagent* tmp;

    int *c1 = x;
    int *c2 = y;

    int* c3 = waypointPointer;

    int *d1 = destinationId;
    double *d2 = destinationX;
    double *d3 = destinationY;
    double *d4 = destinationR;

    int *d5 = LastdestinationId;
    double *d6 = LastdestinationX;
    double *d7 = LastdestinationY;
    double *d8 = LastdestinationR;

    for (int i = 0; i < destinationSize; i++) {
        destinationListX[i] = waypoints[i]->getx();
        destinationListY[i] = waypoints[i]->gety();
        destinationListR[i] = waypoints[i]->getr();
    }

    for (int i = 0; i < s; i++) {
		tmp = agents[i];

        // iterate all the values
        c1 = c1 + 1;
        c2 = c2 + 1;
        c3 = c3 + 1;
        d1 = d1 + 1;
        d2 = d2 + 1;
        d3 = d3 + 1;
        d4 = d4 + 1;
        d5 = d5 + 1;
        d6 = d6 + 1;
        d7 = d7 + 1;
        d8 = d8 + 1;

        // set all the values
        *c1 = tmp->getX();
        *c2 = tmp->getY();
        *c3 = 0;
        *d1 = NULL;
        *d2 = NULL;
        *d3 = NULL;
        *d4 = NULL;
        *d5 = NULL;
        *d6 = NULL;
        *d7 = NULL;
        *d8 = NULL;

    }
}