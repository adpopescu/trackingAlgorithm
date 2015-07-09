#include <stdio.h>

#include "global.h"

cudaArray* d_pGaussWeights = NULL;
texture<float, 2, cudaReadModeElementType> t_gaussian_weights;
// float4 -x UL - y UR - z BL - w BR -- one lookup table for each cell in the block
cudaArray* d_pBilinearWeights = NULL;
texture<float4, 2, cudaReadModeElementType> t_bilinear_weights;


__host__ int prepareGaussWeights()
{
	const float cX = HOG_BLOCK_WIDTH / 2 - 0.5f;
	const float cY = HOG_BLOCK_HEIGHT / 2 - 0.5f;
	float h_pGauss[HOG_BLOCK_WIDTH][HOG_BLOCK_HEIGHT];

	for(int y=0; y < HOG_BLOCK_HEIGHT; y++) {
		for(int x=0; x < HOG_BLOCK_WIDTH; x++) {
			h_pGauss[x][y] = 1.f /(2.f * (float)M_PI * SIGMA) * exp(- 0.5f * ( (x-cX)*(x-cX)/(SIGMA*SIGMA) + (y-cY)*(y-cY)/(SIGMA*SIGMA) ) );
		}
	}
	// normalize to 1
	float sum = 0;
	for(int x=0; x < HOG_BLOCK_WIDTH; x++) {
		for(int y=0; y < HOG_BLOCK_HEIGHT; y++) {
			sum += h_pGauss[x][y];
		}
	}
	for(int x=0; x < HOG_BLOCK_WIDTH; x++) {
		for(int y=0; y < HOG_BLOCK_HEIGHT; y++) {
			h_pGauss[x][y] /= sum;
		}
	}

#ifdef DEBUG_GAUSS_WEIGHTS
	// output gauss weights to file for debugging
	FILE* fp = fopen("gauss_weights.txt", "w");
	for(int x=0; x < 16; x++) {
		for(int y=0; y < 16; y++)
			fprintf(fp, "%.8f ", h_pGauss[x][y] );
		fprintf(fp, "\n");
	}
	fclose(fp);
#endif

	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float>();

	cudaMallocArray( &d_pGaussWeights, &channelDesc, HOG_BLOCK_WIDTH, HOG_BLOCK_HEIGHT);
		ONFAIL("malloc array\n");

	cudaMemcpyToArray(d_pGaussWeights, 0, 0, h_pGauss,
						HOG_BLOCK_WIDTH * HOG_BLOCK_HEIGHT * sizeof(float),
						cudaMemcpyHostToDevice);
		ONFAIL("memcpy to array\n");

	cudaBindTextureToArray( t_gaussian_weights, d_pGaussWeights, channelDesc);
		ONFAIL("bind tex to array\n");

	return 0;
}

#ifdef DEBUG_GAUSS_WEIGHTS
__global__ void testGaussWeights(float* d_pOutput)
{
	if(threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0 && blockIdx.y == 0 ) {
		for(int blockX=0; blockX < HOG_BLOCK_WIDTH; blockX++) {
			for(int blockY=0; blockY < 16; blockY+=8) {
				for(int i=0; i < HOG_CELL_SIZE; i++) {
					float g = tex2D( t_gaussian_weights, blockX, blockY+i);
					d_pOutput[HOG_BLOCK_WIDTH*(blockY+i)+blockX] = g;
				}
			}
		}
	}

}
#endif

__host__ int prepareBilinearWeights()
{
	float* h_pWeights = (float*)malloc(sizeof(float) * 4 * 2 * HOG_CELL_SIZE * 2 * HOG_CELL_SIZE);
	if(!h_pWeights) {
		printf("prepareBilinearWeights: malloc failed!\n");
		return -1;
	}

	float h_weights_L[HOG_CELL_SIZE*2];	// left cells
	float h_weights_R[HOG_CELL_SIZE*2];	// right cells (left mirrored)
	float h_weights_T[HOG_CELL_SIZE*2];	// upper cells
	float h_weights_B[HOG_CELL_SIZE*2];	// bottom cells

	memset(h_weights_L, 0, sizeof(float) * HOG_CELL_SIZE * 2);
	memset(h_weights_R, 0, sizeof(float) * HOG_CELL_SIZE * 2);
	memset(h_weights_T, 0, sizeof(float) * HOG_CELL_SIZE * 2);
	memset(h_weights_B, 0, sizeof(float) * HOG_CELL_SIZE * 2);

	int d = 9;
	int g = 0;
	for(int x=0; x < 4; x++, g+=2) {
			h_weights_L[x] = (d+g) / 16.f;
			h_weights_T[x] = (d+g) / 16.f;
	}
	d = 15;
	g = 0;
	for(int x=4; x < 12; x++, g+=2) {
		h_weights_L[x] = (d-g) / 16.f;
		h_weights_T[x] = (d-g) / 16.f;
	}

	for(int x=0; x < HOG_CELL_SIZE*2; x++) {
		h_weights_R[x] = h_weights_L[HOG_CELL_SIZE*2-1-x];
		h_weights_B[x] = h_weights_T[HOG_CELL_SIZE*2-1-x];
	}

	// prepare a complete lookup table for each pixel in a _block_ !
	for(int x=0; x < 2*HOG_CELL_SIZE; x++) {
		for(int y=0; y < 2*HOG_CELL_SIZE; y++) {
			const int idx = 4 * (y * 2*HOG_CELL_SIZE + x);
			// float4 -- x UL - y UR - z BL - w BR
			h_pWeights[idx+0] = h_weights_L[x] * h_weights_T[y];
			h_pWeights[idx+1] = h_weights_R[x] * h_weights_T[y];
			h_pWeights[idx+2] = h_weights_L[x] * h_weights_B[y];
			h_pWeights[idx+3] = h_weights_R[x] * h_weights_B[y];
		}
	}

#ifdef DEBUG_BILINEAR_WEIGHTS
	// output table to file for debugging
	int j =0;
	FILE* fp = fopen("bilweights.txt", "w");
	for(int i=0; i < 4; i++) {
		j=0;
		for(int x=0; x < 4*16*16; x+=4) {
			fprintf(fp, "%.3f ", h_pWeights[x+i] );
			j++;
			if(j == 16 ) { j=0; fprintf(fp, "\n"); }
		}
		fprintf(fp, "\n\n");
	}
	fclose(fp);
#endif

	// export the lookup table as a texture
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float4>();

	cudaMallocArray( &d_pBilinearWeights, &channelDesc, 2*HOG_CELL_SIZE, 2*HOG_CELL_SIZE);
		ONFAIL("malloc weights array\n");

	cudaMemcpyToArray(d_pBilinearWeights, 0, 0, h_pWeights,
						sizeof(float) * 4 * 2*HOG_CELL_SIZE * 2*HOG_CELL_SIZE,
						cudaMemcpyHostToDevice);
		ONFAIL("memcpy to weights array\n");

	cudaBindTextureToArray( t_bilinear_weights, d_pBilinearWeights, channelDesc);
		ONFAIL("bind tex to weights array\n");

	free(h_pWeights);
	return 0;
}


#ifdef DEBUG_BILINEAR_WEIGHTS
__global__ void testBilinearWeights(float* d_pOutput)
{
	if(threadIdx.x == 0 && threadIdx.y == 0 && blockIdx.x == 0 && blockIdx.y == 0 ) {
		for(int i=0; i < HOG_BLOCK_HEIGHT; i++) {
			for(int j=0; j < HOG_BLOCK_WIDTH; j++) {
				float4 weights = tex2D(t_bilinear_weights, j, i);
				d_pOutput[HOG_BLOCK_WIDTH*i+j] = weights.w;
			}
		}
	}
}
#endif


__host__ int blocks_finalize()
{
	cudaUnbindTexture(t_gaussian_weights);
		ONFAIL("t_gaussian_weights");
	cudaFreeArray(d_pGaussWeights);
		ONFAIL("cudaFree: d_pGaussianWeights");

	cudaUnbindTexture(t_bilinear_weights);
		ONFAIL("t_bilinear_weights");
	cudaFreeArray(d_pBilinearWeights);
		ONFAIL("cudaFree: d_pBilinearWeights");

	return 0;
}


// one thread for each cell column -- 8 * 4 threads -- 32 threads
// 1. load gradients & magnitudes in shared mem (2 floats * 576 pixels)
// 2. compute cell histograms
// 3. combine cell histograms and normalize
__global__ void d_compute_blocks(int width, int height, float2* d_pGradMag, float* d_blocks)
{
	volatile __shared__ float	s_block[32][2*2][NBINS];		// 32 threads, 4 cells, 9 bins
	volatile __shared__ float	s_squares[4];

// copy relevant gradient data for the block
	const int cellIdx = threadIdx.x;	// 0-3	cells in block
	const int columnIdx = threadIdx.y;	// 0-7	columns
	const int sIdx = threadIdx.y * blockDim.x + threadIdx.x;	// which of the 32 threads are we?

	// position of the upper-most pixel in the column for this thread
	const int blockX = (cellIdx % 2)*HOG_CELL_SIZE + columnIdx;
	const int blockY = cellIdx < 2 ? 0 : HOG_CELL_SIZE;

	const int pixelX = blockIdx.x * (HOG_BLOCK_WIDTH/2) + blockX;		// we assume 50% overlap
	const int pixelY = blockIdx.y * (HOG_BLOCK_HEIGHT/2) + blockY;

	// initialize all bins for this thread
	for(int i=0; i < NBINS; i++) {
		for(int cell =0; cell < HOG_BLOCK_CELLS_X*HOG_BLOCK_CELLS_Y; cell++)
			s_block[sIdx][cell][i] = 0.f;
	}

	__syncthreads();
// ----------------------------------------------------------------------------


// for each pixel in the column of this thread
	if(pixelX < width && pixelY < height) {
	for(int i=0; i < HOG_CELL_SIZE; i++)
	{
// decide which cells to contribute to
// compute the contribution by weights
// magnitude * gaussian_weight * trilinear_interpolation
		const int pixelIdx = (pixelY + i) * width + pixelX;

		float magnitude = d_pGradMag[pixelIdx].y;
		float contribution = magnitude * tex2D(t_gaussian_weights, blockY+i, blockX);

	// calculate contribution to the two bins
		float binSize = 180.f / NBINS;

		float orientation = d_pGradMag[pixelIdx].x - binSize/2.f;
		if(orientation < 0) orientation += 180.f;
		float delta = (orientation * NBINS) / 180.f;

		int leftBin = (int)floorf( delta );
		delta -= leftBin;
		int rightBin = leftBin >= NBINS-1 ? 0 : leftBin+1;
		if( leftBin < 0 ) leftBin = NBINS -1;

		float rightContribution = contribution * (delta);
		float leftContribution = contribution * (1-delta);

	// add contributions to cells (with appropriate bilinear weights)
		float4 weights = tex2D(t_bilinear_weights, blockX, blockY+i);
		s_block[sIdx][0][leftBin] += leftContribution * weights.x;
		s_block[sIdx][0][rightBin]+= rightContribution * weights.x;

		s_block[sIdx][1][leftBin] += leftContribution * weights.y;
		s_block[sIdx][1][rightBin]+= rightContribution * weights.y;

		s_block[sIdx][2][leftBin] += leftContribution * weights.z;
		s_block[sIdx][2][rightBin]+= rightContribution * weights.z;

		s_block[sIdx][3][leftBin] += leftContribution * weights.w;
		s_block[sIdx][3][rightBin]+= rightContribution * weights.w;
	}
	}
	__syncthreads();
// ----------------------------------------------------------------------------
// reduce histograms in shared mem to one histogram
	if(threadIdx.y == 0)
	{
		// first reduce all the column results into one column
		for(int i=1; i < 32; i++) {
			for(int bin=0; bin < NBINS; bin++) {
				s_block[0][threadIdx.x][bin] += s_block[i][threadIdx.x][bin];
			}
		}
	}
	__syncthreads();
// ----------------------------------------------------------------------------
// normalize the block histogram - L2+Hys normalization

	const float epsilon = 0.036f * 0.036f;	// magic numbers
	const float eHys	= 0.1f * 0.1f;
	const float clipThreshold = 0.2f;

	if(threadIdx.y == 0 ) {
		float ls = 0.f;
		for(int j=0; j < NBINS; j++) {
			ls += s_block[0][threadIdx.x][j] * s_block[0][threadIdx.x][j];
		}
		s_squares[threadIdx.x] = ls;
	}
	__syncthreads();
	if(threadIdx.y == 0 && threadIdx.x == 0 ) {
		s_squares[0] += s_squares[1] + s_squares[2] + s_squares[3];
	}
	__syncthreads();
	// we use rsqrtf (reciprocal sqrtf) because of CUDA pecularities
	float normalization = rsqrtf(s_squares[0]+epsilon);
	// normalize and clip
	if(threadIdx.y == 0 ) {
		for(int j=0; j < NBINS; j++) {
			s_block[0][threadIdx.x][j] *= normalization;
			s_block[0][threadIdx.x][j] = s_block[0][threadIdx.x][j] > clipThreshold ? clipThreshold : s_block[0][threadIdx.x][j];
		}
	}

	// renormalize
	if(threadIdx.y == 0 ) {
		float ls = 0.f;
		for(int j=0; j < NBINS; j++) {
			ls += s_block[0][threadIdx.x][j] * s_block[0][threadIdx.x][j];
		}
		s_squares[threadIdx.x] = ls;
	}
	__syncthreads();
	if(threadIdx.y == 0 && threadIdx.x == 0 ) {
		s_squares[0] += s_squares[1] + s_squares[2] + s_squares[3];
	}

	normalization = rsqrtf(s_squares[0]+eHys);
	if(threadIdx.y == 0 ) {
		for(int j=0; j < NBINS; j++) {
			s_block[0][threadIdx.x][j] *= normalization;
		}
	}
/*	 // L1
		const float epsilon = 0.001f;
		float sum = 0.f;
		for(int i=0; i < 4; i++) {
			for(int j=0; j < NBINS; j++) {
				sum += s_block[0][i][j];
			}
		}
		float normalization = 1.f / (sum + epsilon);
		for(int i=0; i < 4; i++) {
			for(int j=0; j < NBINS; j++) {
				s_block[0][i][j] *= normalization;
			}
		}
	__syncthreads();
*/

// ----------------------------------------------------------------------------
// copy the block histogram to device mem
	if(threadIdx.y == 0 ) {
		const int writeIdx = NBINS*4 * (blockIdx.y * gridDim.x + blockIdx.x);
		for(int bin=0; bin < NBINS; bin++) {
			d_blocks[writeIdx + threadIdx.x*NBINS + bin] = s_block[0][threadIdx.x][bin];
		}
	}
}

__host__ int compute_blocks(dim3 grid, int width, int height, float2* d_pGradMag, float* d_pBlocks)
{
#ifdef DEBUG_DUMP_BLOCKS
	const int nBlocks = grid.x * grid.y;
	const int blocksMemorySize = nBlocks * HOG_BLOCK_CELLS_X * HOG_BLOCK_CELLS_Y * NBINS * sizeof(float);
	cudaMemset(d_pBlocks, 0, blocksMemorySize);
#endif

	// call the cuda kernel to do the computation
	dim3 threads;
	threads.x = 4; threads.y = 8;

	d_compute_blocks<<< grid , threads >>>(width, height, d_pGradMag, d_pBlocks);
		ONFAIL("compute_blocks kernel failed");

#ifdef DEBUG_DUMP_BLOCKS
	// debug: read the result back to host memory
	float* h_pBlocks = (float*)malloc(blocksMemorySize);
	cudaMemcpy(h_pBlocks, d_pBlocks, blocksMemorySize, cudaMemcpyDeviceToHost);
		ONFAIL("cudaMemcpy for block histograms failed"); //dirty, if we leave here..

	FILE* fp = fopen("blocks.txt", "w");
	int j=0, k=4, block_counter=0;
	fprintf(fp, "blocks: %d x %d\n", grid.x, grid.y);
	for(int x=0; x < NBINS * 4 * grid.x * grid.y; x++) {
		if(k==4) { fprintf(fp, "\n%d\n", block_counter); k=0; block_counter++; }
		fprintf(fp, "%.16f\t", h_pBlocks[x]);
		j++; if(j==NBINS){ fprintf(fp, "\n"); j=0; k++; }
	}
	fclose(fp);
	free(h_pBlocks);
#endif

	return 0;
}
