#ifndef __NMS_H__
#define __NMS_H__

#include "cudaHOG.h"
#include "detections.h"

namespace cudaHOG {

void nms_process_detections(DetectionList& detections, DetectionList& detections_nms,
							ModelParameters& params);

void nms_test(const char* fnDetections);

}
#endif
