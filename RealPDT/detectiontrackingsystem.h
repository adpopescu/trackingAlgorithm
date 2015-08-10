//
// Created by dtenty on 06/08/15.
//

#ifndef REALPDT_DETECTIONTRACKINGSYSTEM_H_
#define REALPDT_DETECTIONTRACKINGSYSTEM_H_


#include "Eigen/Core"
#include "Streaming/StreamingApp.h"
#include "Grabber.h"
#include "Globals.h"
#include "Vector.h"
#include "AncillaryMethods.h"
#include "config_file/ConfigFile.h"
#include "detector.h"
#include "depthdetector.h"
#include "depthdetector_lm.h"
#include "detector_seg.h"
#include "depthdetector_seg.h"
#include "depthdetector_lm_seg.h"
#include "groundplaneestimator.h"
#include "timing.h"
#include "fovis/fovis.hpp"
#include "main.h"
#include "Streaming/StreamingApp.h"
#include <RealPDT/CImg/CImg.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <queue>






class DetectionTrackingSystem
{
public:
    static DetectionTrackingSystem *getInstance();

    void get_image(unsigned char* b_image, uint w, uint h, CImg<unsigned char>& cim);

    void get_depth(const Matrix<double>& depth_map, uint w, uint h,
                   CImg<unsigned char>& cim, PointCloud pc, Vector<double> gp);

    void draw_roi(const Matrix<int>& roi_mat, uint w, uint h, CImg<unsigned char>& cim);

    void draw_hist(const Matrix<int>& hist_mat, uint w, uint h, CImg<unsigned char>& cim);

    void draw_hist(const Matrix<double>& hist_mat, uint w, uint h, CImg<unsigned char>& cim);

    void vis_gp(CImg<unsigned char>& cim, const Camera& camera);

    string isometryToString(const Eigen::Isometry3d& m);

    struct HogParams
    {
        HogParams(DetectionTrackingSystem* obj, int f, unsigned char* img, Camera cam,
                  Vector<Vector<double> >* detected_bb, Vector<Vector < double > >* OutputHOGdet)
        {
            _this = obj;
            frame = f;
            image = img;
            camera = cam;
            detected_bounding_boxes = detected_bb;
            OutputHOGdetL = OutputHOGdet;
        }

        DetectionTrackingSystem* _this;
        int frame;
        unsigned char* image;
        Camera camera;
        Vector<Vector<double> >* detected_bounding_boxes;
        Vector<Vector < double > >* OutputHOGdetL;
    };

    struct UpperParams
    {
        UpperParams(DetectionTrackingSystem* obj,Camera cam, Matrix<double>* dp,
                    PointCloud* pc,Vector<Vector<double> >* detected_bb)
        {
            _this = obj;
            camera = cam;
            depth_map = dp;
            point_cloud = pc;
            detected_bounding_boxes = detected_bb;
        }

        DetectionTrackingSystem* _this;
        Camera camera;
        Matrix<double>* depth_map;
        PointCloud* point_cloud;
        Vector<Vector<double> >* detected_bounding_boxes;
    };

    static void* hog_thread_function(void* params);

    static void* upper_thread_function(void* params);

    int gp_count_down;
    int motion_not_valid_count;

    void main_process(unsigned char* b_image, float* b_depth, uint w, uint h);



    static void grabber_callback(const float *depth, const unsigned char *image);

    void get_from_file(long int frm);

    static void* get_files(void* obj);

    // send to stream, save images and depth and clean memory
    void sink_frame(long int& frm, bool is_runing = true);

    void run ();

    void check_keys();

    void SetTitle();

    void init();



    ~DetectionTrackingSystem();

    CImg<unsigned char> odimg;
    double lodx, lody, lodz;

    queue<unsigned char*> img_queue;
    queue<float*> dp_queue;
    queue<unsigned char*> result_image_queue;
    bool record_sequence;

    Camera base_camera;
    Vector<double> last_gp;
    GroundPlaneEstimator GPEstimator;
    bool is_last_gp_valid;
    bool is_gp_estim_ignored;
    Matrix<double> motion_xyz;
    Matrix<double> motion_rpy;

    //////////////////Detectors//////////////////
    Detector* detector;
    DepthDetector depth_detector;
    DepthDetector_LM depth_detector_lm;

    Detector_Seg* detector_seg;
    DepthDetector_Seg depth_detector_seg;
    DepthDetector_LM_Seg depth_detector_lm_seg;

    bool use_HOG;
    HOGDetector hog_detector;

    //////////// Time ////////////////////////
    double cpu_time_start, cpu_time_end;
    double t_fps, fps_l;

    ///Streaming//////////////////////////////
    StreamingApp streaming;

    ///FOVIS//////////////////////////////////
    fovis::CameraIntrinsicsParameters cam_params;
    fovis::VisualOdometry* odom;
    fovis::Rectification* fovis_rect;
    bool is_first, is_odom_valid;
    Matrix<double> mm;

    ///////TRACKING///////////////////////////
    Detections det_comb;
    Tracker tracker;
    Vector< Hypo > HyposAll;
    long int cnt;

    //////////////////////////////////////////
    Matrix<double> upper_body_template;
    CImgDisplay display;
    bool capture;
    char capture_path[128];
    char path[128];
    char help_string[1000];
    bool show_help;
    bool show_stat;
    bool is_seg;

    enum DISPLAY_MODE
    {
        IMAGE_MODE,
        DEPTH_MODE,
        ROI_MODE,
        ODOM_MODE
    } display_mode;

    enum DETECTOR_MODE
    {
        DEPTH_DETECTOR,
        DEPTH_LM_DETECTOR
    } detector_mode;

 private:
    DetectionTrackingSystem();
    static DetectionTrackingSystem* _this;

    void RenderBBox2D(const Vector<double>& bbox, CImg<unsigned char>& image, int r, int g, int b);

    void ReadUpperBodyTemplate(Matrix<double> &upper_body_template);
};

#endif //REALPDT_DETECTIONTRACKINGSYSTEM_H_
