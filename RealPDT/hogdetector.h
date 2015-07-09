#ifndef HOGDETECTOR_H
#define HOGDETECTOR_H

#include "Globals.h"
#include "Vector.h"
#include "Matrix.h"
#include "camera/Camera.h"
#include <QImage>
#include "cudaHOG.h"


namespace cudaHOG {
class cudaHOGManager; // forward definition
}

class HOGDetector
{
public:
    HOGDetector();
    ~HOGDetector();

    Vector<Vector<double> > runHog(int frame, unsigned char *image, const Camera &cam, Vector<Vector<double> > &detected_bounding_boxes);
    Vector<Vector<double> > runHogPr2(int frame, unsigned char *image, const Camera &cam, Vector<Vector<double> > &detected_bounding_boxes);

    bool rescore;
private:
    Vector<double> fromCamera2World(Vector<double> posInCamera, Camera cam);
    double computeDepth(Vector<double> vbbox, Camera cam);
    Vector<double> projectPlaneToCam(Vector<double> p, Camera cam);
    cudaHOG::cudaHOGManager *hog;
};

#endif // HOGDETECTOR_H
