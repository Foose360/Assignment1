#include "ped_model.h"

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cuda.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"


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
    if(id <= agentSize){
        int x = d_desX[id];
	int y = d_desY[id];

		if (x < 0 || x >= SIZE || y < 0 || y >= SIZE)
		{

		}
		else {
		    // intensify heat for better color results TODO: AtomicAdd
		  atomicAdd(&d_heatmap[y*SIZE + x],40);
		}
    }

    __syncthreads();

    // Scale the data for visual representation
    for (int x = 0; x < SIZE; x++)
      {
        d_heatmap[id*SIZE + x] = d_heatmap[id*SIZE + x] < 255 ? d_heatmap[id*SIZE + x] : 255;
	      int value = d_heatmap[id*SIZE + x];
	
	for (int cellY = 0; cellY < CELLSIZE; cellY++)
	  {
	    for (int cellX = 0; cellX < CELLSIZE; cellX++)
	      {
		d_scaled_heatmap[SCALED_SIZE*(id * CELLSIZE + cellY) + x * CELLSIZE + cellX] = value;
	      }
	  }
      }
	        
    __syncthreads();
}
__device__ void apply_gaussian(int *d_scaled_heatmap, int *d_blurred_heatmap)
{
  __shared__ int s_scaled_heatmap[32 * 32];
  //__shared__ int s_blurred_heatmap[32 * 32];
	// Weights for blur filter
  int row = blockIdx.y * blockDim.y + threadIdx.y; //(blockID * 32 + threadID) => [0,1,2,3,4,5,6,7..., 127] [128, 129,...,n]
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
	__syncthreads(); // Notera denna. Kanske inte behövlig.
}


__global__ void kernelA(int *d_desX, int *d_desY, int *d_heatmap, int *d_scaled_heatmap, int *d_blurred_heatmap, size_t agentSize) {
  cuda_update(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agentSize);
}

__global__ void kernelB(int *d_scaled_heatmap, int *d_blurred_heatmap) {
  apply_gaussian(d_scaled_heatmap, d_blurred_heatmap);
}


void Ped::Model::cuda_updateHeatmapSeq(){
    ///// SKAPA DATA ATT LADDA IN /////
    size_t agentSize = agents.size();
	int *h_desX;
	int *h_desY;
	int *d_desX;
	int *d_desY;

    int *h_heatmap;
    int *h_scaled_heatmap;
    int *h_blurred_heatmap;
    int *d_heatmap;
    int *d_scaled_heatmap;
    int *d_blurred_heatmap;
    ///////////////////////////////////////


    ///////////////////////////////////////

    ///// INLADDNING AV DATA /////

    // storlekarna att allokera
    size_t AgentBytes =  sizeof(int) * agentSize;
    size_t HeatmapBytes = SIZE * SIZE * sizeof(int);
    size_t ScaledHeatmapBytes = SCALED_SIZE * SCALED_SIZE * sizeof(int);

    // allocering av minne i device variabler
    cudaMallocHost((void **)&h_heatmap, HeatmapBytes);
    cudaMallocHost((void **)&h_scaled_heatmap, ScaledHeatmapBytes);
    cudaMallocHost((void **)&h_blurred_heatmap, ScaledHeatmapBytes);
    cudaError_t errd = cudaGetLastError();
    if ( errd != cudaSuccess )
    {
       printf("CUDA Error in 'DEST!': %s\n", cudaGetErrorString(errd));       
    }
    cudaMallocHost((void **)&h_desX, AgentBytes);
    cudaMallocHost((void **)&h_desY, AgentBytes);

  
    ///// INIT DATA SOM SKA LADDAS IN /////
    for (int i = 0; i < agentSize; i++){
      h_desX[i] = agents[i]->getDesiredX();
      h_desY[i] = agents[i]->getDesiredY();
    }
    for(int i = 0; i < SIZE; i++){
      for(int k = 0; k < SIZE; k++){
	
	h_heatmap[i*SIZE + k] = heatmap[i][k];

      }
    }


    cudaError_t err1 = cudaGetLastError();
    if ( err1 != cudaSuccess )
    {
       printf("CUDA Error in 'allocering av minne i device variabler': %s\n", cudaGetErrorString(err1));       
    }
    cudaMalloc((void **)&d_desX, AgentBytes);
    cudaMalloc((void **)&d_desY, AgentBytes);
    cudaMalloc((void **)&d_heatmap, HeatmapBytes);
    cudaMalloc((void **)&d_scaled_heatmap, ScaledHeatmapBytes);
    cudaMalloc((void **)&d_blurred_heatmap, ScaledHeatmapBytes);

    // Record event för tidsgranskning
    float gpu_time;

    cudaEvent_t start;
    cudaEvent_t stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

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
    //id = 0, 5, 10, 15, 20
    dim3 dimBlock(32, 32); //32*32 threads per block = 1024
    dim3 dimGrid(SCALED_SIZE/dimBlock.y, SCALED_SIZE/dimBlock.x); //5120/128 = 40*40 = 1600 thread blocks  
    kernelA<<<1, 1024>>>(d_desX, d_desY, d_heatmap, d_scaled_heatmap, d_blurred_heatmap, agentSize);
    
    kernelB<<<dimGrid, dimBlock>>>(d_scaled_heatmap, d_blurred_heatmap);
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
            heatmap[i][k] = h_heatmap[i*SIZE + k];
        }
    }

    for(int i = 0; i < SCALED_SIZE; i++){
        for(int k = 0; k < SCALED_SIZE; k++){    
            scaled_heatmap[i][k] = h_scaled_heatmap[i*SCALED_SIZE + k];
            blurred_heatmap[i][k] = h_blurred_heatmap[i*SCALED_SIZE + k];	    
	}
    }
    cudaFree(d_desX);
    cudaFree(d_desY);
    cudaFree(d_heatmap);
    cudaFree(d_scaled_heatmap);
    cudaFree(d_blurred_heatmap);
    cudaFreeHost(h_desX);
    cudaFreeHost(h_desY);
    cudaFreeHost(heatmap);
    cudaFreeHost(scaled_heatmap);
    cudaFreeHost(blurred_heatmap);

    // PRINTA TID
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&gpu_time, start, stop);

    std::cout << "\n gpu_time:  " << gpu_time;
    cudaEventDestroy(start);
    cudaEventDestroy(stop);






    /*
    delete [] h_desX;
    delete [] h_desY;
    delete [] h_heatmap;
    delete [] h_scaled_heatmap;
    delete [] h_blurred_heatmap;
    */
    
}