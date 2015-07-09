#ifndef __DETECTIONS_H__
#define __DETECTIONS_H__

#include <vector>
#include <string>

#include "cudaHOG.h"

namespace cudaHOG {

void dump_DetectionList(DetectionList& det, const std::string& fn);
void dump_MultiDetectionList(MultiDetectionList& det, const std::string& fn);

}	// end of namespace cudaHOG
#endif
