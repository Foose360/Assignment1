#include "ped_model.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cuda.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

void Ped::Model::setupHeatmapSeq()
{
 
        size_t agentSize = agents.size();
        size_t AgentBytes =  sizeof(int) * agentSize;
        size_t HeatmapBytes = SIZE * SIZE * sizeof(int);
        size_t ScaledHeatmapBytes = SCALED_SIZE * SCALED_SIZE * sizeof(int);
	
	int *hm = (int*)calloc(SIZE*SIZE, sizeof(int));
	int *shm = (int*)malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));
	int *bhm = (int*)malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));
	
	//heatmap = (int**)malloc(SIZE*sizeof(int*));
	//scaled_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));
	//blurred_heatmap = (int**)malloc(SCALED_SIZE*sizeof(int*));

	cudaMallocHost((void **)&heatmap, SIZE*sizeof(int*));
	cudaMallocHost((void **)&scaled_heatmap, SCALED_SIZE*sizeof(int*));
	cudaMallocHost((void **)&blurred_heatmap, SCALED_SIZE*sizeof(int*));
	    // allocering av minne i device variabler
	cudaMallocHost((void **)&h_desX, AgentBytes);
	cudaMallocHost((void **)&h_desY, AgentBytes);
	//cudaMallocHost((void **)&h_heatmap, HeatmapBytes);
	//cudaMallocHost((void **)&h_scaled_heatmap, ScaledHeatmapBytes);
	//cudaMallocHost((void **)&h_blurred_heatmap, ScaledHeatmapBytes);

	cudaMalloc((void **)&d_desX, AgentBytes);
	cudaMalloc((void **)&d_desY, AgentBytes);
	cudaMalloc((void **)&d_heatmap, HeatmapBytes);
	cudaMalloc((void **)&d_scaled_heatmap, ScaledHeatmapBytes);
	cudaMalloc((void **)&d_blurred_heatmap, ScaledHeatmapBytes);
	cudaMemcpyAsync((void *)d_heatmap, (void *)*heatmap, HeatmapBytes, cudaMemcpyHostToDevice);

	
	for (int i = 0; i < SIZE; i++)
	{
		heatmap[i] = hm + SIZE*i;
	}
	for (int i = 0; i < SCALED_SIZE; i++)
	{
		scaled_heatmap[i] = shm + SCALED_SIZE*i;
		blurred_heatmap[i] = bhm + SCALED_SIZE*i;
	}
}

// Updates the heatmap according to the agent positions
__device__ void cuda_update(int *d_desX, int *d_desY, int *d_heatmap, int *d_scaled_heatmap, int *d_blurred_heatmap, size_t agentSize)
{
    int id = threadIdx.x;

	for (int x = 0; x < SIZE; x++)
	{
		d_heatmap[id*SIZE + x] = (int)round(d_heatmap[id*SIZE + x] * 0.80);
	}

    __syncthreads();

	// Count how many agents want to go to each location
    for (int i = id; i < agentSize; i = i + 1024) {
        int x = d_desX[i];
		int y = d_desY[i];

	if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
	  {

	  }
	else {
	  // intensify heat for better color results TODO: AtomicAdd
	  atomicAdd(&d_heatmap[y*SIZE + x],40);
	}
      }
    
    __syncthreads();
}

__device__ void scale_map(int *d_heatmap, int *d_scaled_heatmap) {
    // Scale the data for visual representation

  int row = blockIdx.y * blockDim.y + threadIdx.y; //   32x32
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  
        d_heatmap[row*SIZE + col] = d_heatmap[row*SIZE + col] < 255 ? d_heatmap[row*SIZE + col] : 255;
	      int value = d_heatmap[row*SIZE + col];
	
	for (int cellY = 0; cellY < CELLSIZE; cellY++)
	  {
	    for (int cellX = 0; cellX < CELLSIZE; cellX++)
	      {
		d_scaled_heatmap[SCALED_SIZE*(row * CELLSIZE + cellY) + col * CELLSIZE + cellX] = value;
	      }
	  }	        
    __syncthreads();
}
__device__ void apply_gaussian(int *d_scaled_heatmap, int *d_blurred_heatmap)
{
  __shared__ int s_scaled_heatmap[32 * 32];
	// Weights for blur filter
  int row = blockIdx.y * blockDim.y + threadIdx.y; //(blockID * 32 + threadID) => [0,1,2,3,4,5,6,7..., 31]... [128, 129,...,n]
  int col = blockIdx.x * blockDim.x + threadIdx.x; //
	const int w[5][5] = {
		{ 1, 4, 7, 4, 1 },
		{ 4, 16, 26, 16, 4 },
		{ 7, 26, 41, 26, 7 },
		{ 4, 16, 26, 16, 4 },
		{ 1, 4, 7, 4, 1 }
	};
	////////////////////////////////////
	//INIT SHARED HEATMAP /////////////
	///////////////////////////////////
	s_scaled_heatmap[threadIdx.y * 32 + threadIdx.x] = d_scaled_heatmap[row * SCALED_SIZE + col];
	__syncthreads();
#define WEIGHTSUM 273
	// Apply gaussian blurfilter
	if (row >= 2 && row + 5 < SCALED_SIZE -2 && col >= 2 && col + 5 < SCALED_SIZE -2) {
	  if((threadIdx.y >= 2 && threadIdx.y + 5 < blockDim.y - 2 && threadIdx.x >= 2 && threadIdx.x + 5 < blockDim.x - 2)) { //Se till att vi inte hamnar utanför index	    
	    int sum = 0;
	    for (int k = -2; k < 3; k++)
	      {
		for (int l = -2; l < 3; l++)
		  {
		    sum +=  w[2 + k][2 + l] * s_scaled_heatmap[blockDim.x * (threadIdx.y + k) + (threadIdx.x + l)];
		  }
	      }
	    int value = sum / WEIGHTSUM;
	    d_blurred_heatmap[row * SCALED_SIZE + col] = 0x00FF0000 | value << 24;
	    
	  }
	  else
	    { //Fetch from d_scaled_heatmap - this means we hit outside designated tile
	    int sum = 0;
	    for (int k = -2; k < 3; k++)
	      {
		for (int l = -2; l < 3; l++)
		  {
		    sum +=  w[2 + k][2 + l] * d_scaled_heatmap[SCALED_SIZE * (row + k) + (col + l)];
		  }
	      }
	    int value = sum / WEIGHTSUM;
	    d_blurred_heatmap[row * SCALED_SIZE + col] = 0x00FF0000 | value << 24;
	  }
	  
	}
}


__global__ void kernelA(int *d_desX, int *d_desY, int *d_heatmap, int *d_scaled_heatmap, int *d_blurred_heatmap, size_t agentSize) {
  cuda_update(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agentSize);
}

__global__ void kernelB(int *d_heatmap, int *d_scaled_heatmap) {
  scale_map(d_heatmap, d_scaled_heatmap);
}

__global__ void kernelC(int *d_scaled_heatmap, int *d_blurred_heatmap) {
  apply_gaussian(d_scaled_heatmap, d_blurred_heatmap);
}


void Ped::Model::cuda_updateHeatmapSeq(){
    ///// SKAPA DATA ATT LADDA IN /////
    size_t agentSize = agents.size();	
    ///////////////////////////////////////
    ///////////////////////////////////////
    ///// INLADDNING AV DATA /////
    // storlekarna att allokera
    size_t AgentBytes =  sizeof(int) * agentSize;  
    size_t ScaledHeatmapBytes = SCALED_SIZE * SCALED_SIZE * sizeof(int);
  
    ///// INIT DATA SOM SKA LADDAS IN /////

    for (int i = 0; i < agentSize; i++){
      h_desX[i] = agents[i]->getDesiredX();
      h_desY[i] = agents[i]->getDesiredY();
    }
    // koppiering av värden från Host till Device
    cudaMemcpyAsync((void *)d_desX, (void *)h_desX, AgentBytes, cudaMemcpyHostToDevice);
    cudaMemcpyAsync((void *)d_desY, (void *)h_desY, AgentBytes, cudaMemcpyHostToDevice);
    
    //    cudaMemcpyAsync((void *)d_scaled_heatmap, (void *)*scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);
    //cudaMemcpyAsync((void *)d_blurred_heatmap, (void *)*blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyHostToDevice);
    ///////////////////////////////////////

    ///// KALL AV KERNEL /////
    //id = 0, 5, 10, 15, 20
    dim3 dimBlock(32, 32); //32*32 threads per block = 1024
    dim3 dimGrid(SCALED_SIZE/dimBlock.y, SCALED_SIZE/dimBlock.x); //5120/128 = 40*40 = 1600 thread blocks
    dim3 dimGridB(SIZE/dimBlock.y, SIZE/dimBlock.x);
    kernelA<<<1, 1024>>>(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agentSize);

    kernelB<<<dimGridB, dimBlock>>>(d_heatmap, d_scaled_heatmap);
    
    kernelC<<<dimGrid, dimBlock>>>(d_scaled_heatmap, d_blurred_heatmap);

    // koppiering av värden från Device till Host.  
    //    cudaMemcpyAsync((void *)*heatmap, (void *)d_heatmap, HeatmapBytes, cudaMemcpyDeviceToHost);
    //cudaMemcpyAsync((void *)*scaled_heatmap, (void *)d_scaled_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);
    cudaMemcpyAsync((void *)*blurred_heatmap, (void *)d_blurred_heatmap, ScaledHeatmapBytes, cudaMemcpyDeviceToHost);
 
}
