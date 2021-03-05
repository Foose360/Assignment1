#include "ped_model.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

// Updates the heatmap according to the agent positions
__global__ void cuda_update(int *d_desX, int *d_desY, int **d_heatmap, int **d_scaled_heatmap, int **d_blurred_heatmap, size_t agentSize)
{
    const id = blockIdx.x * blockDim.x + threadIdx.x;

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			// heat fades
			d_heatmap[y][x] = (int)round(d_heatmap[y][x] * 0.80);
		}
	}

	// Count how many agents want to go to each location
	for (int i = 0; i < agentSize; i++)
	{
		int x = d_desX[i];
		int y = d_desY[i];

		if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
		{
			continue;
		}

		// intensify heat for better color results
		d_heatmap[y][x] += 40;

	}

	for (int x = 0; x < SIZE; x++)
	{
		for (int y = 0; y < SIZE; y++)
		{
			d_heatmap[y][x] = d_heatmap[y][x] < 255 ? d_heatmap[y][x] : 255;
		}
	}

	// Scale the data for visual representation
	for (int y = 0; y < SIZE; y++)
	{
		for (int x = 0; x < SIZE; x++)
		{
			int value = d_heatmap[y][x];
			for (int cellY = 0; cellY < CELLSIZE; cellY++)
			{
				for (int cellX = 0; cellX < CELLSIZE; cellX++)
				{
					d_scaled_heatmap[y * CELLSIZE + cellY][x * CELLSIZE + cellX] = value;
				}
			}
		}
	}

	// Weights for blur filter
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};

#define WEIGHTSUM 273
	// Apply gaussian blurfilter		       
	for (int i = 2; i < SCALED_SIZE - 2; i++)
	{
		for (int j = 2; j < SCALED_SIZE - 2; j++)
		{
			int sum = 0;
			for (int k = -2; k < 3; k++)
			{
				for (int l = -2; l < 3; l++)
				{
					sum += w[2 + k][2 + l] * d_scaled_heatmap[i + k][j + l];
				}
			}
			int value = sum / WEIGHTSUM;
			d_blurred_heatmap[i][j] = 0x00FF0000 | value << 24;
		}
	}
}

void Ped::Model::cuda_updateHeatmapSeq(){
    ///// SKAPA DATA ATT LADDA IN /////
	int h_desX[agents.size()];
	int h_desY[agents.size()];
	int *d_desX;
	int *d_desY;

    int h_heatmap[SIZE][SIZE];
    int h_scaled_heatmap[SCALED_SIZE][SCALED_SIZE];
    int h_blurred_heatmap[SCALED_SIZE][SCALED_SIZE];
    int **d_heatmap;
    int **d_scaled_heatmap;
    int **d_blurred_heatmap;

    ///////////////////////////////////////

    ///// INIT DATA SOM SKA LADDAS IN /////
	for (int i = 0; i < agents.size(); i++){
		h_desX[i] = agents[i]->getDesiredX();
		h_desY[i] = agents[i]->getDesiredY();
	}

    for(int i = 0; i < SIZE; i++){
        for(int k = 0; k < SIZE; k++){

            h_heatmap[i][k] = heatmap[i][k];

        }
    }

    for(int i = 0; i < SCALED_SIZE; i++){       ///Detta borde inte vara nödvändigt, men safe:ar.
        for(int k = 0; k < SCALED_SIZE; k++){
    
            h_scaled_heatmap[i][k] = 0;
            h_blurred_heatmap[i][k] = 0;
        }
    }


    ///////////////////////////////////////

    ///// INLADDNING AV DATA /////

    // storlekarna att allokera
    size_t AgentBytes =  agents.size() * sizeof(int);
    size_t HeatmapBytes = SIZE * SIZE * sizeof(int);
    size_t ScaledHeatmapBytes = SCALED_SIZE * SCALED_SIZE * sizeof(int);

    // allocering av minne i device variabler
    cudaMallocHost((void **)&d_desX, AgentBytes);
    cudaMallocHost((void **) &d_desY, AgentBytes);

    cudaMallocHost((void **)&d_heatmap, HeatmapBytes);

    cudaMallocHost((void **)&d_scaled_heatmap, ScaledHeatmapBytes);
    cudaMallocHost((void **)&d_blurred_heatmap, ScaledHeatmapBytes);


    // koppiering av värden från Host till Device
	cudaMemcpy((void *)d_desX, (void *)h_desX, AgentBytes, cudaMemcpyHostToDevice);
	cudaMemcpy((void *)d_desY, (void *)h_desY, AgentBytes, cudaMemcpyHostToDevice);

    cudaMemcpy((void *)d_heatmap, (void *)h_heatmap, HeatmapBytes, cudaMemcpyHostToDevice);

    cudaMemcpy((void *)d_scaled_heatmap, (void *)h_scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);
    cudaMemcpy((void *)d_blurred_heatmap, (void *)h_blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);

    ///////////////////////////////////////

    ///// KALL AV KERNEL /////
	cuda_update<<<1, 1024>>>(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agents.size());

    // koppiering av värden från Device till Host.  

    // OBS!!!!!! NOTERA ATT DETTA KOPIERAS DIREKT IN I VÅRA FAKTISKA HEATMAPS I MODEL!!!
    cudaMemcpy((void *)heatmap, (void *)d_heatmap, HeatmapBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy((void *)scaled_heatmap, (void *)d_scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy((void *)blurred_heatmap, (void *)d_blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);

}