#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "detections.h"
#include "cudaHOG.h"
#include "parameters.h"

namespace cudaHOG {

void dump_DetectionList(DetectionList& det, const std::string& fn)
{
	FILE* fp = fopen(fn.c_str(), "w");
	for(size_t i=0; i < det.size(); i++) {
		Detection ptr = det[i];
		fprintf(fp,"%d\t%d\t%f\t%f\n", ptr.x, ptr.y, ptr.scale, ptr.score);
	}
	fclose(fp);
}

void dump_MultiDetectionList(MultiDetectionList& det, const std::string& fn)
{
	assert( g_params.models.size() > 0 );

	FILE* fp = fopen(fn.c_str(), "w");

	for(size_t j=0; j < det.size(); j++) {
		fprintf(fp, "\n#--------------------------------\n");
		fprintf(fp, "#  model: %d\n#\n", j+1);

		const int detection_width = g_params.models[j].HOG_WINDOW_WIDTH;
		const int detection_height = g_params.models[j].HOG_WINDOW_HEIGHT;

		for(size_t i=0; i < det[j].size(); i++) {
			Detection d = det[j][i];

// format x,y,ux,uy,score
			int ux = d.x + detection_width * d.scale;
			int uy = d.y + detection_height * d.scale;
			fprintf(fp, "%d\t%d\t%d\t%d\t%f\n", d.x, d.y, ux, uy, d.score);

// format x,y,scale,score
//			fprintf(fp,"%d\t%d\t%f\t%f\n", ptr.x, ptr.y, ptr.scale, ptr.score);
//			printf("%d\t%d\t%f\t%f\n", ptr.x, ptr.y, ptr.scale, ptr.score);
		}
	}
	fclose(fp);
}

}	 // end of namespace cudaHOG
