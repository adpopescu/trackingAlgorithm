#include <stdio.h>

#include "global.h"
#include "parameters.h"

namespace cudaHOG {

// create block descriptor with x,y being the upper-left block
int compute_descriptor(float* d_pDescriptor, float* d_pBlocks, int x, int y, int nBlocksPerRow, ModelParameters& params)
{
	const int nPerBlock = HOG_BLOCK_CELLS_X * HOG_BLOCK_CELLS_Y * NBINS;
	const int nBytesPerBlock = sizeof(float) * nPerBlock;

	for(int i=0; i < params.HOG_DESCRIPTOR_HEIGHT; i++) {
			cudaMemcpy(d_pDescriptor + nPerBlock * (i * params.HOG_DESCRIPTOR_WIDTH),
						d_pBlocks + nPerBlock * ((i+y) * nBlocksPerRow + x),
						nBytesPerBlock * params.HOG_DESCRIPTOR_WIDTH,
						cudaMemcpyDeviceToDevice);
	}
	return 0;
}

#ifdef DEBUG_DUMP_DESCRIPTOR
int test_descriptor(float* d_pDescriptor, ModelParameters& params)
{
	const int nPerBlock = HOG_BLOCK_CELLS_X * HOG_BLOCK_CELLS_Y * NBINS;
	const int nBytesPerBlock = sizeof(float) * nPerBlock;
	const int nDescriptorSize = nBytesPerBlock * params.HOG_DESCRIPTOR_HEIGHT * params.HOG_DESCRIPTOR_WIDTH;

	float* h_pDescriptor = (float*)malloc(params.HOG_DESCRIPTOR_HEIGHT * params.HOG_DESCRIPTOR_WIDTH * nBytesPerBlock);

	cudaMemcpy(h_pDescriptor, d_pDescriptor, nDescriptorSize, cudaMemcpyDeviceToHost);

	FILE* fp = fopen("descriptor.txt", "w");

	for(int i=0; i < params.HOG_DESCRIPTOR_HEIGHT; i++) {
		for( int j=0; j < params.HOG_DESCRIPTOR_WIDTH; j++) {
			float* currBlock = h_pDescriptor + nPerBlock * (i * params.HOG_DESCRIPTOR_WIDTH + j);

			for(int i = 0; i < 4*9; i++) {
				fprintf(fp, "%.5f ", currBlock[i] );
			}
			fprintf(fp, "\n");
		}
	}
	fclose(fp);

	free(h_pDescriptor);

	return 0;
}
#endif

}	// end of namespace cudaHOG
