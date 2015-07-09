#include "global.h"

// pad the image in host memory while copying it over to the device
int pad_image(uchar4** d_pPaddedImg, uchar4* h_pImg,
				int width, int height, int padX,int padY);

#ifdef DEBUG_PAD_IMAGE
void test_pad_image(uchar4* d_pPaddedImg, int paddedWidth, int paddedHeight);
#endif
