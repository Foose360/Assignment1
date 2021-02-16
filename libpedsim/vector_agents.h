#ifndef _vector_agents_h_
#define _vector_agents_h_ 1

#include <vector>
#include <deque>
#include <emmintrin.h>

#include "ped_model.h"

using namespace std;

namespace Ped {
        class Model;
        class Tagent;
	class Vagent 
        {
	public:
	Vagent(Ped::Model mod);

        ///////////////////////////////////////////////////////////
        /// Attributes
        ///////////////////////////////////////////////////////////

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

        ///////////////////////////////////////////////////////////
        /// Methods
        ///////////////////////////////////////////////////////////

        // Update the position according to get closer
	// to the current destination
	void computeNextDesiredPosition(std::vector<Ped::Tagent*> tagents, int i);

	void destinationReached(int i);

        void getNextDestination(std::vector<Ped::Tagent*> tagents, int i);

	private:

        void init(Ped::Model mod);

	};
}

#endif