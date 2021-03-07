///////////////////////////////////////////////////
// Low Level Parallel Programming 2016.
//
//     ==== There is no need to change this file ====
// 

#include "PedSimulation.h"
#include <iostream>
#include <QApplication>

#include <stdlib.h>

using namespace std;

PedSimulation::PedSimulation(Ped::Model &model_, MainWindow &window_) : model(model_), window(window_), maxSimulationSteps(-1)
{
	tickCounter = 0;
}

int PedSimulation::getTickCount() const
{
	return tickCounter;
}
void PedSimulation::simulateOneStep()
{
	//Temporärt ändrad, så nu stödjer vårt system inte flera cores/enbart serial.
    //Måste undersöka hur SLOT fungerar - i nuläget strular den med våra parametrar.	
	int tick_mode = 1;
	int cores = 4;
	if (tick_mode == 0)
	{
		tickCounter++;
		model.tick_serial();
		window.paint();
		if (maxSimulationSteps-- == 0)
		{
			QApplication::quit();
		}
	}
	else if(tick_mode == 1)
	{
		tickCounter++;
		model.tick_omp();
		window.paint();
		if (maxSimulationSteps-- == 0)
		{
			QApplication::quit();
		}
	}
	else if(tick_mode == 2)
	{
		tickCounter++;
		model.tick_threads(cores);
		window.paint();
		if (maxSimulationSteps-- == 0)
		{
			QApplication::quit();
		}
	}
	else
	{
		cout << "invalid tick_mode value detected." << endl;
	}
}

void PedSimulation::runSimulationWithQt(int maxNumberOfStepsToSimulate, int tick_mode, int cores)
{
	maxSimulationSteps = maxNumberOfStepsToSimulate;

	//movetimer.setInterval(50); // Limits the simulation to 20 FPS (if one so whiches).
	QObject::connect(&movetimer, SIGNAL(timeout()), this, SLOT(simulateOneStep()));
	movetimer.start();
}

void PedSimulation::runSimulationWithoutQt(int maxNumberOfStepsToSimulate, int tick_mode, int cores)
{
	maxSimulationSteps = maxNumberOfStepsToSimulate;

	if (tick_mode == 0)
	{
		for (int i = 0; i < maxSimulationSteps; i++)
		{
			tickCounter++;
			model.tick_serial();
		}
	}
	else if (tick_mode == 1)
	{
		for (int i = 0; i < maxSimulationSteps; i++)
		{
			tickCounter++;
			model.tick_omp();
		}
	}
	else if (tick_mode == 2)
	{
		for (int i = 0; i < maxSimulationSteps; i++)
		{
			tickCounter++;
			model.tick_threads(cores);
		}	
	}
	else
	{
		cout << "invalid tick_mode value detected." << endl;
	}
}

#include "PedSimulation.moc"
