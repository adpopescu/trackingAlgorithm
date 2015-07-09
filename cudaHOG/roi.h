#ifndef __ROI_H__
#define __ROI_H__

struct _ROI;
typedef struct _ROI ROI;

struct _ROI {
	int min_x;
	int min_y;
	int max_x;
	int max_y;
	float scale;
};

#endif // __ROI_H__
