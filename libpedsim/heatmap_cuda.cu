#include "ped_model.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

// Updates the heatmap according to the agent positions
__global__ void cuda_update(int *d_desX, int *d_desY, int **d_heatmap, int **d_scaled_heatmap, int **d_blurred_heatmap, size_t agentSize)
{
    int id = threadIdx.x;

	for (int x = 0; x < SIZE; x++)
	{
		d_heatmap[id][x] = (int)round(d_heatmap[id][x] * 0.80);
	}

    __syncthreads();

	// Count how many agents want to go to each location
    if(id <= agentSize){
        int x = d_desX[id];
		int y = d_desY[id];

		if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
		{

		}
        else {
		    // intensify heat for better color results TODO: AtomicAdd
		    d_heatmap[y][x] += 40;
        }
    }

    __syncthreads();

	for (int x = 0; x < SIZE; x++)
	{
		d_heatmap[id][x] = d_heatmap[id][x] < 255 ? d_heatmap[id][x] : 255;
	}

    __syncthreads();

	// Scale the data for visual representation
		for (int x = 0; x < SIZE; x++){

		int value = d_heatmap[id][x];

		for (int cellY = 0; cellY < CELLSIZE; cellY++){
			for (int cellX = 0; cellX < CELLSIZE; cellX++){
				d_scaled_heatmap[id * CELLSIZE + cellY][x * CELLSIZE + cellX] = value;
			}
		}
	}

    __syncthreads();

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
	for (int i = 2; i < CELLSIZE - 2; i++)
	{
		for (int j = 2; j < SCALED_SIZE - 2; j++)
		{
			int sum = 0;
			for (int k = -2; k < 3; k++)
			{
				for (int l = -2; l < 3; l++)
				{
					sum += w[2 + k][2 + l] * d_scaled_heatmap[i + id + k][j + l];
				}
			}
			int value = sum / WEIGHTSUM;
			d_blurred_heatmap[i][j] = 0x00FF0000 | value << 24;
		}
	}

    __syncthreads(); // Notera denna. Kanske inte behövlig.

}

void Ped::Model::cuda_updateHeatmapSeq(){
    ///// SKAPA DATA ATT LADDA IN /////
    size_t agentSize = agents.size();
	int h_desX[agentSize];
	int h_desY[agentSize];
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
	for (int i = 0; i < agentSize; i++){
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
    size_t AgentBytes =  sizeof(int) * agentSize;
    size_t HeatmapBytes = SIZE * SIZE * sizeof(int);
    size_t ScaledHeatmapBytes = SCALED_SIZE * SCALED_SIZE * sizeof(int);

    // allocering av minne i device variabler
    cudaMallocHost((void **)&d_desX, AgentBytes);
    cudaMallocHost((void **)&d_desY, AgentBytes);

    cudaError_t errd = cudaGetLastError();
    if ( errd != cudaSuccess )
    {
       printf("CUDA Error in 'DEST!': %s\n", cudaGetErrorString(errd));       
    }

    cudaMallocHost((void **)&d_heatmap, HeatmapBytes);

    cudaMallocHost((void **)&d_scaled_heatmap, ScaledHeatmapBytes);
    cudaMallocHost((void **)&d_blurred_heatmap, ScaledHeatmapBytes);

    cudaError_t err1 = cudaGetLastError();
    if ( err1 != cudaSuccess )
    {
       printf("CUDA Error in 'allocering av minne i device variabler': %s\n", cudaGetErrorString(err1));       
    }


    // koppiering av värden från Host till Device
	cudaMemcpy((void *)d_desX, (void *)h_desX, AgentBytes, cudaMemcpyHostToDevice);
	cudaMemcpy((void *)d_desY, (void *)h_desY, AgentBytes, cudaMemcpyHostToDevice);

    cudaMemcpy((void *)d_heatmap, (void *)h_heatmap, HeatmapBytes, cudaMemcpyHostToDevice);

    cudaMemcpy((void *)d_scaled_heatmap, (void *)h_scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);
    cudaMemcpy((void *)d_blurred_heatmap, (void *)h_blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);

    cudaError_t err2 = cudaGetLastError();
    if ( err2 != cudaSuccess )
    {
       printf("CUDA Error in 'koppiering av värden från Host till Device': %s\n", cudaGetErrorString(err2));       
    }

    ///////////////////////////////////////

    ///// KALL AV KERNEL /////
	cuda_update<<<1, 1024>>>(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agentSize);

    cudaError_t err3 = cudaGetLastError();
    if ( err3 != cudaSuccess )
    {
       printf("CUDA Error in 'KALL AV KERNEL': %s\n", cudaGetErrorString(err3));       
    }

    // koppiering av värden från Device till Host.  
    cudaMemcpy((void *)h_heatmap, (void *)d_heatmap, HeatmapBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy((void *)h_scaled_heatmap, (void *)d_scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);
    cudaMemcpy((void *)h_blurred_heatmap, (void *)d_blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);

    cudaError_t err4 = cudaGetLastError();
    if ( err4 != cudaSuccess )
    {
       printf("CUDA Error in 'koppiering av värden från Device till Host': %s\n", cudaGetErrorString(err4));       
    }

    ///////////////////////////////////////

    ///// UPPDATERING AV VÄRDEN I MODEL /////
    
    /// OBS! detta är sjukt dummt och innefektivt.
    /// koppieringen bör gå dirrekt till model och detta ska inte behövas.
    /// Temp lösning för att kolla så allt funkar.
    for(int i = 0; i < SIZE; i++){
        for(int k = 0; k < SIZE; k++){
            heatmap[i][k] = h_heatmap[i][k];
        }
    }

    for(int i = 0; i < SCALED_SIZE; i++){
        for(int k = 0; k < SCALED_SIZE; k++){
    
            scaled_heatmap[i][k] = h_scaled_heatmap[i][k];
            blurred_heatmap[i][k] = h_blurred_heatmap[i][k];
        }
    }
}