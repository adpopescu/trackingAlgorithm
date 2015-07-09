#include <stdio.h>	// only for test routine!

__global__ void uchar4_to_float4(float4* output, uchar4* input, int width, int height)
{
	int posX = blockDim.x * blockIdx.x + threadIdx.x;
	int posY = blockDim.y * blockIdx.y + threadIdx.y;

	if(posX < width && posY < height) {
		// compute position in image
		int i = posY * width + posX;
		output[i].x = input[i].x;
		output[i].y = input[i].y;
		output[i].z = input[i].z;
		output[i].w = input[i].w;
	}
}


int convert_uchar4_to_float4(float4** d_pOutput, uchar4* d_pInput,
								int width, int height)
{
	cudaError_t e;
	cudaMalloc((void**)d_pOutput, sizeof(float4) * width * height);
		e = cudaGetLastError();
		if(e) return -1;

	dim3 threads(16,16);
	dim3 grid( (int)ceil(width / (float)threads.x), (int)ceil(height / (float)threads.y));
	uchar4_to_float4<<< grid, threads >>>(*d_pOutput, d_pInput, width, height);
		e = cudaGetLastError();
		if(e) return -2;

	return 0;
}

void test_convert_uchar4_to_float4(float4* d_pFloatImg, int width, int height)
{
	float4* h_pFloatImg = (float4*)malloc(sizeof(float4) * width * height);
	if(!h_pFloatImg) {
		printf("test_convert_float4_to_float4: malloc failed\n");
		return;
	}

	cudaMemcpy(h_pFloatImg, d_pFloatImg, sizeof(float4) * width * height, cudaMemcpyDeviceToHost);

	FILE* fp = fopen("float_image.txt","w");
	if(!fp) {
		printf("test_convert_float4_to_float4: failed to open file\n");
		return;
	}
	for(int y=0; y < height; y++) {
		for(int x=0; x < width; x++) {
			fprintf(fp, "(%.0f,%.0f,%.0f,%.0f)",
					h_pFloatImg[y*width + x].x,
					h_pFloatImg[y*width + x].y,
					h_pFloatImg[y*width + x].z,
					h_pFloatImg[y*width + x].w);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}
