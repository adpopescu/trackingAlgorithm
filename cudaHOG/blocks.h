#ifndef __blocks_h__
#define __blocks_h__

extern "C++" __host__ int prepareGaussWeights();
extern "C++" __global__ void testGaussWeights(float* d_pOutput);

extern "C++" __host__ int prepareBilinearWeights();
extern "C++" __global__ void testBilinearWeights(float* d_pOutput);

extern "C++" __host__ int blocks_finalize();

extern "C++" __host__ int compute_blocks(dim3 grid,
						int width, int height, float2* d_pGradMag,
						float* d_pBlocks);
#endif
