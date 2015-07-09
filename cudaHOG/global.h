#ifndef __global_h__
#define __global_h__

// all the mallocs are done according to this maximum requirement
const int MAX_IMAGE_DIMENSION = 3000;

const int HOG_SVM_MAX_MODELS = 10;

// NMS mean-shift parameters
const int 	SIGMA_FACTOR 	= 4;
const float	SIGMA_SCALE		= 1.6f;

#define KERNEL_RADIUS	10
#define HOG_BLOCK_CELLS_X	2
#define HOG_BLOCK_CELLS_Y	2
#define HOG_BLOCK_WIDTH 	(8*HOG_BLOCK_CELLS_X)
#define HOG_BLOCK_HEIGHT	(8*HOG_BLOCK_CELLS_Y)
#define HOG_CELL_SIZE		8		// we assume 8x8 cells!

#define HOG_PADDING_X		16		// padding in pixels to add to each side
#define HOG_PADDING_Y		16		// padding in pixels to add to top&bottom

#define NEGATIVE_SAMPLES_PER_IMAGE	10

// 0 -- use weighted sum score for mode
// 1 -- use maximum score for mode
#define NMS_MAXIMUM_SCORE	1
//#define SKIP_NON_MAXIMUM_SUPPRESSION // disable NMS step

extern float	HOG_START_SCALE;
extern float 	HOG_SCALE_STEP;

const float SIGMA	=		(0.5 * HOG_BLOCK_WIDTH);		// gaussian window size for block histograms

const int NBINS = 9;
// the original procedure uses 1.2 steps
const float HOG_TRAINING_SCALE_STEP = 1.05f;//1.2f

const int MAXIMUM_HARD_EXAMPLES = 50000;

// FLAGS
#define ENABLE_GAMMA_COMPRESSION	// enable sqrt gamma compression

#define PRINT_PROFILING_TIMINGS		0 // show profiling timings for each frame
#define PRINT_VERBOSE_INFO		0 // show detection information
#define PRINT_DEBUG_INFO		0 // show verbose debug information at each scale level

// DEBUG FLAGS
//#define DEBUG_GAUSS_WEIGHTS		// dump gauss weights
//#define DEBUG_BILINEAR_WEIGHTS	// dump bilinear weights
//#define DEBUG_DUMP_GRADIENTS		// dump gradient&magnitude to file
//#define DEBUG_DUMP_BLOCKS
//#define DEBUG_DUMP_DESCRIPTOR
//#define DEBUG_DUMP_SVM_RESULTS
//#define DEBUG_DUMP_SVM_MODEL_DATA
//#define DEBUG_DUMP_POSITIVE_DETECTIONS
//#define DEBUG_PRINT_POSITIVE_DETECTIONS
//#define DEBUG_NMS_TEST
//#define DEBUG_ROI_TO_FILE
//#define DEBUG_ROI_TO_SCREEN
#define DEBUG_PRINT_PROGRESS					0
#define DEBUG_PRINT_SCALE_CONSTRAINTS 			0


// HACKS - workarounds for camera file formats
#define ENABLE_HACK_LEUVEN	0
#define ENABLE_HACK_BELLEVUE	0

// -----------------------------------------------
#define VERBOSE_CUDA_FAILS

#ifdef VERBOSE_CUDA_FAILS
#define ONFAIL(S) { cudaError_t e = cudaGetLastError(); \
					if(e) { printf("%s:%s:"S":%s\n", __FILE__, __FUNCTION__ , cudaGetErrorString(e));\
					return -2; } }
#else
#define ONFAIL(S)
#endif

#endif
