
//extern "C++" __global__ void d_compute_gradients(int width, int height, float2* d_pGradMag);
extern __host__ int compute_gradients(int paddedWidth, int paddedHeight,
										int padX, int padY,
										int min_x, int min_y, int max_x, int max_y,
										float2* d_pGradMag);


int prepare_image(const unsigned char* h_pImg, int width, int height);
int destroy_image();
int test_prepared_image(float scale, int origwidth, int origheight, int padX, int padY);

