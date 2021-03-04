//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// Model coordinates a time step in a scenario: for each
// time step all agents need to be moved by one position if
// possible.
//
#ifndef _ped_model_h_
#define _ped_model_h_

#include <vector>
#include <map>
#include <set>
#include <omp.h>

#include "ped_agent.h"
#include "vector_agents.h"

namespace Ped{
	class Tagent;
	class Vagent;

	// The implementation modes for Assignment 1 + 2:
	// chooses which implementation to use for tick()
	enum IMPLEMENTATION { CUDA, VECTOR, OMP, PTHREAD, SEQ };

	class Model
	{
	public:

		// Sets everything up
		void setup(std::vector<Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario,IMPLEMENTATION implementation);
		
		// Coordinates a time step in the scenario: move all agents by one step (if applicable).
		void tick_serial();
		void tick_threads(int cores);
		void tick_omp();

		// Returns the agents of this scenario
		const std::vector<Tagent*> getAgents() const { return agents; };

		const std::vector<Twaypoint*> getDestinations() const { return destinations; };
		// Adds an agent to the tree structure
		void placeAgent(const Ped::Tagent *a);
	 	       
	        std::vector<std::vector<int>> placeAgents();
		// Cleans up the tree and restructures it. Worth calling every now and then.
		void cleanup();
		~Model();

		// Returns the heatmap visualizing the density of agents
		int const * const * getHeatmap() const { return blurred_heatmap; };
		int getHeatmapSize() const;

		Ped::Vagent *vagents;
         	void delegateTasks();

	        void moveOmp(int region);
		// Adds a Vagent to the model. // STUDENT MADE //
		void addVagent(Ped::Vagent *v) { vagents = v; }

	private:

		// Denotes which implementation (sequential, parallel implementations..)
		// should be used for calculating the desired positions of
		// agents (Assignment 1)
		IMPLEMENTATION implementation;

		// The agents in this scenario
		std::vector<Tagent*> agents;

	        //The agents in this scenario sorted by their current regions
	        std::vector<std::vector<int>> regionAgents;
		// The waypoints in this scenario
		std::vector<Twaypoint*> destinations;

         	std::vector<int> agentQueue;

		// Moves an agent towards its next position
	  bool move(Ped::Tagent *agent, bool addToagentQueue);

		// Locks for the regions
		omp_lock_t regionLocks[4];

		////////////
		/// Everything below here won't be relevant until Assignment 3
		///////////////////////////////////////////////

		// Returns the set of neighboring agents for the specified position
	  bool getNeighbors(int x, int y, int dist, int region) const;

		////////////
		/// Everything below here won't be relevant until Assignment 4
		///////////////////////////////////////////////

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

		// The heatmap representing the density of agents
		int ** heatmap;

		// The scaled heatmap that fits to the view
		int ** scaled_heatmap;

		// The final heatmap: blurred and scaled to fit the view
		int ** blurred_heatmap;

		void setupHeatmapSeq();
		void updateHeatmapSeq();

		void apply_decay(int *d_desX, int *d_desY);
		void apply_heat(int *d_desX, int *d_desY);
		void set_heat(int *d_desX, int *d_desY);
		void scale_heatmap(int *d_desX, int *d_desY);
		void apply_gaussian(int *d_desX, int *d_desY);
	};
}
#endif
