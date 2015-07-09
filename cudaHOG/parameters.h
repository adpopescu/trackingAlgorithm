#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <vector>
#include <string>

namespace cudaHOG {


// Model specific parameters for the HOG descriptor
class ModelParameters
{
public:
	ModelParameters(int dw = 7, int dh = 15, int ww = 64, int wh = 128)
		:	identifier("unnamed"), filename(""),
			HOG_DESCRIPTOR_WIDTH(dw), HOG_DESCRIPTOR_HEIGHT(dh),
			HOG_WINDOW_WIDTH(ww), HOG_WINDOW_HEIGHT(wh),
			min_scale(0.f), max_scale(100000.f)
			{};

public:
	std::string identifier;
	std::string filename;
	int HOG_DESCRIPTOR_WIDTH;	// 7
	int HOG_DESCRIPTOR_HEIGHT;	// 15
	int HOG_WINDOW_WIDTH;		// 64
	int HOG_WINDOW_HEIGHT;		// 128
	float min_scale;			// 0.f
	float max_scale;			// 100000.f
	int dimension();
};


// This class holds global parameters for the HOG descriptor
class Parameters
{

public:
	int load_from_file(std::string& fn);
	std::string path;		// path to the files (model&config)

// parameters for each model
	std::vector<ModelParameters> models;

	int min_descriptor_height();
	int min_descriptor_width();
	int min_window_height();
	int min_window_width();

	int max_window_height();
	int max_window_width();
};

// the global parameters object, accessible from everywhere in the library
extern Parameters g_params;

}	// end of namespace cudaHOG
#endif
