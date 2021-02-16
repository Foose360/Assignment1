#ifndef _vector_agents_h_
#define _vector_agents_h_ 1

#include <vector>
#include <deque>
#include <emmintrin.h>

#include "ped_agent.h"

using namespace std;

namespace Ped {
        class Tagent;
	class Vagent 
        {
	public:
	Vagent(std::vector<Ped::Tagent*> agents);

        ///////////////////////////////////////////////////////////
        /// Attributes
        ///////////////////////////////////////////////////////////

        float *x; // pekare till int:s på rad.
        float *y; // pekare till int:s på rad.
        float *reachedDestination; 
        float *destinationX; // pekare till double:s på rad.
        float *destinationY; // pekare till double:s på rad.
        float *destinationR; // pekare till double:S på rad.

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

        void init(std::vector<Ped::Tagent*> agents);

	};
}

#endif