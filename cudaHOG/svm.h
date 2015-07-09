#ifndef __SVM_H__
#define __SVM_H__

#include "cudaHOG.h"
#include "detections.h"

namespace cudaHOG {

int svm_initialize();
int svm_finalize();

int svm_add_model(const char* fnModel);

int svm_evaluate(float* d_pBlocks, int nBlockX, int nBlockY,
				int padX, int padY, int minX, int minY, float scale,
				int* cnt,
				MultiDetectionList& detections);

}
#endif
