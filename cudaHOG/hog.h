#include "detections.h"
#include "roi.h"
#include "cudaHOG.h"

namespace cudaHOG {

extern int hog_initialize();
extern int hog_finalize();

extern int hog_transfer_image(const unsigned char* h_pImg, int width, int height);
extern int hog_release_image();

extern int hog_process_image(int width, int height, float scale,
							int padX, int padY, ROI* roi, int* cntBlocks, int* cntSVM, MultiDetectionList& detections);
extern int hog_process_image_multiscale(int width, int height, std::vector<ROI>& roi, int* cntBlocks, int* cntSVM,
							double* timings, MultiDetectionList& detections);

extern int hog_get_descriptor(int width, int height, int bPad,
						int featureX, int featureY, float scale,
						ModelParameters& params,
						float* h_pDescriptor);

}	// end of namespace cudaHOG
