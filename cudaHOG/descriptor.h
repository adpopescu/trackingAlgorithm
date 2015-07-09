#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

#include "parameters.h"

namespace cudaHOG {

int compute_descriptor(float* d_pDescriptor, float* d_pBlocks, int x, int y, int nBlocksPerRow, ModelParameters& params);

#ifdef DEBUG_DUMP_DESCRIPTOR
int test_descriptor(float* d_pDescriptor, ModelParameters& params);
#endif

}	// end of namespace cudaHOG
#endif
