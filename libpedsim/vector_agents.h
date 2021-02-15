#ifndef _vector_agents_h_
#define _vector_agents_h_ 1

#include <vector>
#include <deque>

using namespace std;

namespace Ped {
	class Vagent {
	public:
		Vagent(Ped::Model mod);

        int *x; // pekare till int:s på rad.
        int *y; // pekare till int:s på rad.

        int *destinationId; // pekare till int:s på rad.
        double *destinationX; // pekare till double:s på rad.
        double *destinationY; // pekare till double:s på rad.
        double *destinationR; // pekare till double:S på rad.

        int *LastdestinationId; // pekare till int:s på rad.
        double *LastdestinationX; // pekare till double:s på rad.
        double *LastdestinationY; // pekare till double:s på rad.
        double *LastdestinationR; // pekare till double:S på rad.

        deque<Twaypoint*>* *wp; // pekare till waypoints.

	private:

        void init(Ped::Model mod);
	};
}

#endif