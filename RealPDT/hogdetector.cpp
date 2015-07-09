#include "hogdetector.h"

HOGDetector::HOGDetector()
{
    hog = new cudaHOG::cudaHOGManager("svm_model_plain_j1_42704");
    hog->set_max_sclae(Globals::hog_max_scale);
    int minimum_height=1500; // in centimeters
    int maximum_height=2000; // in centimeters
    hog->set_groundplane_corridor(minimum_height, maximum_height);
    rescore=true;
}

HOGDetector::~HOGDetector()
{
    delete hog;
}

Vector<Vector<double> > HOGDetector::runHogPr2(int frame, unsigned char* image, const Camera& cam, Vector<Vector< double > >& detected_bounding_boxes)
{
    //----------------------//
    // CUDA Hog             //
    //----------------------//
    std::vector<cudaHOG::Detection> detHog;

    QImage tt = QImage(image, Globals::dImWidth, Globals::dImHeight, QImage::Format_RGB888);

    int returnPrepare = hog->prepare_image(tt.convertToFormat(QImage::Format_ARGB32).bits(), tt.width(), tt.height());

    Vector<Vector<double> > foundDet;
    if(returnPrepare)
    {
        return foundDet;
    }

    Matrix<float> R(3,3);
    Matrix<float> K(3,3);
    Vector<float> t(3);
    Vector<float> GPN(3);
    float gp_d = -cam.get_GP()[3] * 1000;

    for(int i = 0; i < 9; i++)
    {
        R.data()[i] = cam.get_R().data()[i];
        K.data()[i] = cam.get_K().data()[i];
    }

    for(int i = 0; i < 3; i++)
    {
        t.data()[i] = cam.get_t().data()[i];
        GPN.data()[i] = cam.get_GPN().data()[i];
    }

    t *= 1000;


    hog->set_camera(R.data(), K.data(), t.data());
    hog->set_groundplane(GPN.data(), &gp_d);
    try
    {
        hog->prepare_roi_by_groundplane();
    }
    catch(...)
    {
        return foundDet;
    }

    hog->test_image(detHog);
    hog->release_image();

    /////////////////////////////////////////////////////////////////////////
    Vector<double> camPos(3, 0.0);
    Camera camI(cam.get_K(), Eye<double>(3), camPos, projectPlaneToCam(cam.get_GP(), cam));
    /////////////////////////////////////////////////////////////////////////

    Vector<double> oneDet(9);
    int w = 64;
    int h = 128;
    Vector<double> bbox(4);
    Vector<double> detected_bounding_box(5);
    for(unsigned int i=0;i<detHog.size();i++)
    {
        float scale = detHog[i].scale;

        float width = (w - 32.0f)*scale;
        float height = (h - 32.0f)*scale/3.0f;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        bbox(0) = x;
        bbox(1) = y;
        bbox(2) = width;
        bbox(3) = height;


        if(detHog[i].score > Globals::hog_score_thresh){
            float score = detHog[i].score;

            if(rescore && score>0.5f)
            {
                score = (score-0.5f)/(3.0f-0.5f)*0.3f+0.7f;
            }

            oneDet(0) = (frame);
            oneDet(1) = (i);
            oneDet(2) = (scale);
            oneDet(3) = (score);
            oneDet(4) = (x);
            oneDet(5) = (y);
            oneDet(6) = (width);
            oneDet(7) = (height);
            oneDet(8) = (computeDepth(bbox,camI));
            foundDet.pushBack(oneDet);

            detected_bounding_box(0)=x;
            detected_bounding_box(1)=y;
            detected_bounding_box(2)=width;
            detected_bounding_box(3)=height;
            detected_bounding_box(4)=1-score;
            detected_bounding_boxes.pushBack(detected_bounding_box);
        }
    }
    return foundDet;
}

Vector<Vector<double> > HOGDetector::runHog(int frame, unsigned char* image, const Camera& cam, Vector<Vector< double > >& detected_bounding_boxes)
{
    //----------------------//
    // CUDA Hog             //
    //----------------------//
    std::vector<cudaHOG::Detection> detHog;

    QImage tt = QImage(image, Globals::dImWidth, Globals::dImHeight, QImage::Format_RGB888);

    int returnPrepare = hog->prepare_image(tt.convertToFormat(QImage::Format_ARGB32).bits(), tt.width(), tt.height());

    Vector<Vector<double> > foundDet;
    if(returnPrepare)
    {
        return foundDet;
    }

    Matrix<float> R(3,3);
    Matrix<float> K(3,3);
    Vector<float> t(3);
    Vector<float> GPN(3);
    float gp_d = -cam.get_GP()[3] * 1000;

    for(int i = 0; i < 9; i++)
    {
        R.data()[i] = cam.get_R().data()[i];
        K.data()[i] = cam.get_K().data()[i];
    }

    for(int i = 0; i < 3; i++)
    {
        t.data()[i] = cam.get_t().data()[i];
        GPN.data()[i] = cam.get_GPN().data()[i];
    }

    t *= 1000;


    hog->set_camera(R.data(), K.data(), t.data());
    hog->set_groundplane(GPN.data(), &gp_d);
    try
    {
        hog->prepare_roi_by_groundplane();
    }
    catch(...)
    {
        return foundDet;
    }

    hog->test_image(detHog);
    hog->release_image();

    /////////////////////////////////////////////////////////////////////////
    Vector<double> camPos(3, 0.0);
    Camera camI(cam.get_K(), Eye<double>(3), camPos, projectPlaneToCam(cam.get_GP(), cam));
    /////////////////////////////////////////////////////////////////////////

    Vector<double> oneDet(9);
    int w = 64;
    int h = 128;
    Vector<double> bbox(4);
    Vector<double> detected_bounding_box(5);
    for(unsigned int i=0;i<detHog.size();i++)
    {
        float scale = detHog[i].scale;

        float width = (w - 32.0f)*scale;
        float height = (h - 32.0f)*scale;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        bbox(0) = x;
        bbox(1) = y;
        bbox(2) = width;
        bbox(3) = height;


        if(detHog[i].score > Globals::hog_score_thresh){

            oneDet(0) = (frame);
            oneDet(1) = (i);
            oneDet(2) = (scale);
            oneDet(3) = (detHog[i].score);
            oneDet(4) = (x);
            oneDet(5) = (y);
            oneDet(6) = (width);
            oneDet(7) = (height);
            oneDet(8) = (computeDepth(bbox,camI));
            foundDet.pushBack(oneDet);

            detected_bounding_box(0)=x;
            detected_bounding_box(1)=y;
            detected_bounding_box(2)=width;
            detected_bounding_box(3)=height;
            detected_bounding_box(4)=detHog[i].score;
            detected_bounding_boxes.pushBack(detected_bounding_box);
        }
    }
    return foundDet;
}

Vector<double> HOGDetector::fromCamera2World(Vector<double> posInCamera, Camera cam)
{
    Matrix<double> rotMat = cam.get_R();

    Vector<double> posCam = cam.get_t();

    Matrix<double> trMat(4,4,0.0);
    trMat(3,3) = 1;
    trMat(0,0) = rotMat(0,0);
    trMat(0,1) = rotMat(0,1);
    trMat(0,2) = rotMat(0,2);
    trMat(1,0) = rotMat(1,0);
    trMat(1,1) = rotMat(1,1);
    trMat(1,2) = rotMat(1,2);
    trMat(2,0) = rotMat(2,0);
    trMat(2,1) = rotMat(2,1);
    trMat(2,2) = rotMat(2,2);

    posCam *= Globals::WORLD_SCALE;

    trMat(3,0) = posCam(0);
    trMat(3,1) = posCam(1);
    trMat(3,2) = posCam(2);

    Vector<double> transpoint = trMat*posInCamera;
    return transpoint;

}

double HOGDetector::computeDepth(Vector<double> vbbox, Camera cam)
{
    Vector<double> pos3D;
    double distance;

    cam.bbToDetection(vbbox, pos3D, Globals::WORLD_SCALE, distance);

    Vector<double> posInCamCord(pos3D(0), pos3D(1), pos3D(2), 1);

    Vector<double> posInWorld = fromCamera2World(posInCamCord, cam);

    return posInWorld(2);
}

Vector<double> HOGDetector::projectPlaneToCam(Vector<double> p, Camera cam)
{
    Vector<double> gpInCam(4, 0.0);

    Vector<double> pv;
    pv.pushBack(p(0));
    pv.pushBack(p(1));
    pv.pushBack(p(2));

    Vector<double> camPos = cam.get_t();

    Matrix<double> camRot = cam.get_R();

    pv = Transpose(camRot)*pv;
    camRot *= -1.0;
    Vector<double> t = Transpose(camRot)*camPos;

    double d = p(3) - (pv(0)*t(0) + pv(1)*t(1) + pv(2)*t(2));

    gpInCam(0) = pv(0);
    gpInCam(1) = pv(1);
    gpInCam(2) = pv(2);
    gpInCam(3) = d;

    return gpInCam;
}
