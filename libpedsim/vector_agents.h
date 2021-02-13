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
        double* destinationListX; //Alla destinationers x-koordinater.
        double* destinationListY; //Alla destinationers y-koordinater.
        double* destinationListR; //Alla destinationers radier.
        size_t destinationSize; // storlek på destinationslistan


        int* destinationPointer; // Pekare till agentens nuvarande position i destinationslistan
        int *destinationId; // pekare till agentens lokala destinationid.
        double *destinationX; // pekare till agentens lokala x-koordinat.
        double *destinationY; // pekare till agentens lokala y-koordinat.
        double *destinationR; // pekare till agentens lokala radie.

        int *LastdestinationId; // pekare till int:s på rad.
        double *LastdestinationX; // pekare till double:s på rad.
        double *LastdestinationY; // pekare till double:s på rad.
        double *LastdestinationR; // pekare till double:S på rad.

	private:

        void init(Ped::Model mod);
        std::vector<Twaypoint*> destinations;

	};
}

#endif