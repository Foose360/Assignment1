#ifndef _vector_agents_h_
#define _vector_agents_h_ 1

#include <vector>
#include <deque>
#include <emmintrin.h>
using namespace std;

namespace Ped {
	class Vagent {
	public:
		Vagent(Ped::Model mod);

        int *x; // pekare till int:s på rad.
        int *y; // pekare till int:s på rad.

        int *reachedDestination; 
        int *destinationId; // pekare till int:s på rad
        float *destinationX; // pekare till double:s på rad.
        float *destinationY; // pekare till double:s på rad.
        float *destinationR; // pekare till double:S på rad.

        int *LastdestinationId; // pekare till int:s på rad.
        float *LastdestinationX; // pekare till double:s på rad.
        float *LastdestinationY; // pekare till double:s på rad.
        float *LastdestinationR; // pekare till double:S på rad.


	private:

        void init(Ped::Model mod);
	};
}

#endif