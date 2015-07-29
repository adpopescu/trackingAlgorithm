#include "Grabber.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include "Globals.h"
#include "Vector.h"
#include "AncillaryMethods.h"
#include "config_file/ConfigFile.h"

#include <stdio.h>

#include "detector.h"
#include "depthdetector.h"
#include "depthdetector_lm.h"
#include "detector_seg.h"
#include "depthdetector_seg.h"
#include "depthdetector_lm_seg.h"

#include "pointcloud.h"
#include "groundplaneestimator.h"

#include "fovis/fovis.hpp"
#include "main.h"
#include "Eigen/Core"

#include <queue>

#include "Streaming/StreamingApp.h"

string path_config_file = "config_Asus.inp";

///////////////////////////////////////////////////////////////////////
// for timing the code
///////////////////////////////////////////////////////////////////////
//time_t  user_time_start, user_time_end;
//double  cpu_time_start, cpu_time_end;

inline double CPUTime()
{
    struct rusage ruse;
    //    getrusage(RUSAGE_SELF,&ruse);
    getrusage(RUSAGE_THREAD,&ruse);

    return ( ruse.ru_utime.tv_sec + ruse.ru_stime.tv_sec +
             1e-6 * (ruse.ru_utime.tv_usec + ruse.ru_stime.tv_usec) );
}

///////////////////////////////////////////////////////////////////////
// ReadConfigFile:
//      Reads the config file for setting up the global variables.
///////////////////////////////////////////////////////////////////////
void ReadConfigFile()
{

    ConfigFile config(path_config_file);


    Globals::preload = config.read("preload",false);

    //=====================================
    // Input paths
    //=====================================
    config.readInto(Globals::camPath_left, "camPath_left");
    config.readInto(Globals::sImagePath_left, "sImagePath_left");
    config.readInto(Globals::tempDepthL, "tempDepthL");
    config.readInto(Globals::path_to_planes, "path_to_planes");
    Globals::is_depth_disparity_map = config.read("is_depth_disparity_map", true);

    //=====================================
    // Distance Range Accepted Detections
    //=====================================
    Globals::distance_range_accepted_detections = config.read<double>("distance_range_accepted_detections", 7);

    //======================================
    // ROI
    //======================================
    Globals::inc_width_ratio = config.read<double>("inc_width_ratio");
    Globals::inc_height_ratio = config.read<double>("inc_height_ratio");
    Globals::region_size_threshold = config.read<double>("region_size_threshold", 10);

    Globals::max_height = config.read<double>("max_height", 2.0);
    Globals::min_height = config.read<double>("min_height", 1.4);

    //======================================
    // Freespace Parameters
    //======================================
    Globals::freespace_scaleZ = config.read<double>("freespace_scaleZ", 20);
    Globals::freespace_scaleX = config.read<double>("freespace_scaleX", 20);
    Globals::freespace_minX = config.read<double>("freespace_minX", -20);
    Globals::freespace_minZ = config.read<double>("freespace_minZ", 0);
    Globals::freespace_maxX = config.read<double>("freespace_maxX", 20);
    Globals::freespace_maxZ = config.read<double>("freespace_maxZ", 30);
    Globals::freespace_threshold = config.read<double>("freespace_threshold", 120);
    Globals::freespace_max_depth_to_cons = config.read<int>("freespace_max_depth_to_cons", 20);
    Globals::log_weight = config.read("log_weight", false);
    Globals::freespace_max_height = config.read<double>("freespace_max_height", 2.5);

    //======================================
    // Evaluation Parameters
    //======================================
    Globals::evaluation_NMS_threshold = config.read<double>("evaluation_NMS_threshold",0.4);
    Globals::evaluation_NMS_threshold_LM = config.read<double>("evaluation_NMS_threshold_LM",0.4);
    Globals::evaluation_NMS_threshold_Border = config.read<double>("evaluation_NMS_threshold_Border",0.4);
    Globals::evaluation_inc_height_ratio = config.read<double>("evaluation_inc_height_ratio",0.2);
    Globals::evaluation_stride = config.read<int>("evaluation_stride",3);
    Globals::evaluation_scale_stride = config.read<double>("evaluation_scale_stride",1.03);
    Globals::evaluation_nr_scales = config.read<int>("evaluation_nr_scales",1);
    Globals::evaluation_inc_cropped_height = config.read<int>("evaluation_inc_cropped_height",20);
    Globals::evaluation_greedy_NMS_overlap_threshold = config.read<double>("evaluation_greedy_NMS_overlap_threshold", 0.1);
    Globals::evaluation_greedy_NMS_threshold = config.read<double>("evaluation_greedy_NMS_threshold", 0.25);
    //======================================
    // World scale
    //======================================
    config.readInto(Globals::WORLD_SCALE, "WORLD_SCALE");

    //======================================
    // height and width of images
    //======================================
    Globals::dImHeight = config.read<int>("dImHeight");
    Globals::dImWidth = config.read<int>("dImWidth");

    //======================================
    // Camera
    //======================================
    Globals::baseline = config.read<double>("baseline");

    //====================================
    // Number of Frames / offset
    //====================================
    Globals::numberFrames = config.read<int>("numberFrames");
    Globals::nOffset = config.read<int>("nOffset");

    //======================================
    // Console output
    //======================================
    Globals::verbose = config.read("verbose", false);

    //=====================================
    // Determines if save bounding boxes or not
    //=====================================
    Globals::export_bounding_box = config.read("export_bounding_box", false);
    // Path of exported bounding boxes
    config.readInto(Globals::bounding_box_path, "bounding_box_path");

    //=====================================
    // Determines if save result images or not
    //=====================================
    Globals::export_result_images = config.read("export_result_images", false);
    config.readInto(Globals::result_images_path, "result_images_path");

    //====================================
    // Size of Template
    //====================================
    Globals::template_size = config.read<int>("template_size");


    /////////////////////////////////TRACKING PART/////////////////////////
    //======================================
    // Detections
    //======================================
    Globals::cutDetectionsUsingDepth = config.read("cutDetectionsUsingDepth", false);

    Globals::frameRate = config.read<int>("frameRate");

    //======================================
    // Camera
    //======================================
    Globals::farPlane = config.read<double>("farPlane");

    //======================================
    // World scale
    //======================================
    config.readInto(Globals::binSize, "binSize");

    //======================================
    // Pedestrians width and height
    //======================================
    Globals::pedSizeWVis = config.read<double>("pedSizeWVis");
    Globals::pedSizeWCom = config.read<double>("pedSizeWCom");
    Globals::pedSizeHCom = config.read<double>("pedSizeHCom");

    //======================================
    // History
    //======================================
    Globals::history = config.read<int>("history");

    //======================================
    // Pedestrians parameter
    //======================================
    Globals::dObjHeight = config.read<double>("dObjHeight");
    Globals::dObjHVar = config.read<double>("dObjHVar");

    //======================================
    // Adjustment for HOG detections
    //======================================
    Globals::cutHeightBBOXforColor = config.read<double>("cutHeightBBOXforColor");
    Globals::cutWidthBBOXColor = config.read<double>("cutWidthBBOXColor");
    Globals::posponeCenterBBOXColor = config.read<double>("posponeCenterBBOXColor");

    //======================================
    // Thresholds for combining the detection from left and right camera
    //======================================
    Globals::probHeight = config.read<double>("probHeight");

    //======================================
    // Visualisation
    //======================================
    Globals::render_bbox3D = config.read("render_bbox3D", true);
    Globals::render_bbox2D = config.read("render_bbox2D", false);
    Globals::render_tracking_numbers = config.read("render_tracking_numbers", false);

    //======================================
    // MDL parameters for trajectories
    //======================================
    Globals::k1 = config.read<double>("k1");
    Globals::k2 = config.read<double>("k2");
    Globals::k3 = config.read<double>("k3");
    Globals::k4 = config.read<double>("k4");

    //======================================
    // Threshold for distinction between static/moving object
    //======================================
    Globals::minvel = config.read<double>("minvel");
    Globals::dMaxPedVel = config.read<double>("dMaxPedVel");

    //======================================
    // Threshold for identity management
    //======================================
    Globals::dSameIdThresh = config.read<double>("dSameIdThresh");

    //======================================
    // Trajectory
    //======================================
    Globals::threshLengthTraj = config.read<int>("threshLengthTraj");

    //======================================
    // Thresholds for accepted and displayed hypotheses
    //======================================
    Globals::dTheta2 = config.read<double>("dTheta2");

    //======================================
    // Time ant for temporal decay
    //======================================
    Globals::dTau = config.read<double>("dTau");

    //======================================
    // Time horizon for event cone search
    //======================================
    Globals::coneTimeHorizon = config.read<int>("coneTimeHorizon");
    Globals::maxHoleLen = config.read<int>("maxHoleLen");
    Globals::dHolePenalty = config.read<double>("dHolePenalty");

    // Q - the system covariance
    Globals::sysUncX = config.read<double>("sysUncX");
    Globals::sysUncY = config.read<double>("sysUncY");
    Globals::sysUncRot = config.read<double>("sysUncRot");
    Globals::sysUncVel = config.read<double>("sysUncVel");
    Globals::sysUncAcc = config.read<double>("sysUncAcc");

    Globals::kalmanObsMotionModelthresh = config.read<double>("kalmanObsMotionModelthresh");
    Globals::kalmanObsColorModelthresh = config.read<double>("kalmanObsColorModelthresh");

    /////////////////////////////////GP Estimator/////////////////////////
    Globals::nrInter_ransac = config.read<int>("nrInter_ransac");
    Globals::numberOfPoints_reconAsObstacle = config.read<int>("numberOfPoints_reconAsObstacle");

    //======================================
    // ROI Segmentation
    //======================================
    // Blurring parameters
    Globals::sigmaX = config.read<double>("sigmaX", 2.0);
    Globals::precisionX = config.read<double>("precisionX", 2.0);
    Globals::sigmaZ = config.read<double>("sigmaZ", 3.0);
    Globals::precisionZ = config.read<double>("precisionZ", 2.0);

    ///////////////////////////Recording /////////////////////
    Globals::from_camera = config.read("from_camera", true);
    config.readInto(Globals::from_file_path, "from_file_path");

    //////////////////////////Streaming///////////////////////
    config.readInto(Globals::stream_dest_IP, "stream_dest_IP");

    ////////////////////////HOG Detector////////////////////////
    Globals::hog_max_scale = config.read<float>("hog_max_scale",1.9);
    Globals::hog_score_thresh = config.read<float>("hog_score_thresh",0.4);

    ///////////////////////Components///////////////////////////
    Globals::use_hog = config.read("use_hog", false);
    Globals::use_segmentation_roi = config.read("use_segmentation_roi", false);
    Globals::use_local_max = config.read("use_local_max", true);
}

void PrintUsage( const std::string &sProgName)
{
    std::cout << "\nUsage:  " << sProgName << " <options>";
    std::cout << "\n---------------------------------------";
    std::cout << "\n   -c                <name>          (config_file_name)";
    std::cout << "\n" << std::endl;
}

///////////////////////////////////////////////////////////////////////
// ProcessCommandArgs:
//      Processes command line arguments.
///////////////////////////////////////////////////////////////////////
void ProcessCommandArgs(int argc, char **argv)
{
    for( int i=0; i<argc; ++i )
    {
        if( argv[i][0]=='-' )
        {
            if( (strcmp(argv[i],"-help")==0) || (strcmp(argv[i],"--help")==0)
                    || (strcmp(argv[i],"-h")==0) || (strcmp(argv[i],"-?")==0) )
            {
                PrintUsage(argv[0]);
                exit(0);
            }
            else if( strcmp(argv[i],"-c")==0 )
            {
                if( argc>(i+1) )
                    path_config_file = argv[++i];
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////
// ReadUpperBodyTemplate:
//      Reads template of upper body from file and resizes it to
//      Global::template_size which is determined in ConfigFile.
//
// parameters:
//      output:
//          upper_body_template -   Template of upper body.
///////////////////////////////////////////////////////////////////////
void ReadUpperBodyTemplate(Matrix<double>& upper_body_template)
{
    // read template from file
    upper_body_template.ReadFromTXT("upper_temp_n.txt", 150, 150);

    // resize it to the fixed size that is defined in Config File
    if(upper_body_template.x_size() > Globals::template_size)
    {
        upper_body_template.DownSample(Globals::template_size, Globals::template_size);
    }
    else if(upper_body_template.x_size() < Globals::template_size)
    {
        upper_body_template.UpSample(Globals::template_size, Globals::template_size);
    }
}

void RenderBBox2D(const Vector<double>& bbox, CImg<unsigned char>& image, int r, int g, int b)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    const unsigned char color[] = {r,g,b};
    image.draw_rectangle_1(x,y,x+w,y+h,color,3);
}

enum DETECTOR_MODE
{
    DEPTH_DETECTOR,
    DEPTH_LM_DETECTOR
} detector_mode;

class DetectionTrackingSystem
{
public:

    void get_image(unsigned char* b_image, uint w, uint h, CImg<unsigned char>& cim)
    {
        unsigned char* ptr = b_image;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                // access the viewerImage as column, row
                cim(col,row,0,0) = *(ptr++); // red component
                cim(col,row,0,1) = *(ptr++); // green
                cim(col,row,0,2) = *(ptr++); // blue
            }
        }
    }

    void get_depth(const Matrix<double>& depth_map, uint w, uint h, CImg<unsigned char>& cim, PointCloud pc, Vector<double> gp)
    {
        double min=depth_map.data()[0];
        double max=min;
        for(uint i=1; i<w*h; ++i)
        {
            double value = depth_map.data()[i];
            if(value > max)
                max=value;
            if(value < min)
                min=value;
        }

        int i=0;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                unsigned char value = (unsigned char)((depth_map(col,row)-min)*255/(max-min));
                double d = fabs(pc.X(i)*gp(0)+pc.Y(i)*gp(1)+pc.Z(i)*gp(2)+gp(3));
                if(d>0.02)
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = value; // red component
                    cim(col,row,0,1) = value; // green
                    cim(col,row,0,2) = value; // blue
                }
                else
                {
                    // access the viewerImage as column, row
                    cim(col,row,0,0) = 0; // red component
                    cim(col,row,0,1) = 0; // green
                    cim(col,row,0,2) = value; // blue
                }
                ++i;
            }
        }
    }

    void draw_roi(const Matrix<int>& roi_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(roi_mat.x_size()<w || roi_mat.y_size()<h) return;

        unsigned char color_array[][3] = {   {0  ,     0,     0},
                                             {204,     0,   255},
                                             {255,     0,     0},
                                             {0,   178,   255},
                                             {255,     0,   191},
                                             {255,   229,     0},
                                             {0,   255,   102},
                                             {89,   255,     0},
                                             {128,     0,   255},
                                             {242,     0,   255},
                                             {242,   255,     0},
                                             {255,     0,    77},
                                             {51,     0,   255},
                                             {0,   255,   140},
                                             {0,   255,    25},
                                             {204,   255,     0},
                                             {255,   191,     0},
                                             {89,     0,   255},
                                             {0,   217,   255},
                                             {0,    64,   255},
                                             {255,   115,     0},
                                             {255,     0,   115},
                                             {166,     0,   255},
                                             {13,     0,   255},
                                             {0,    25,   255},
                                             {0,   255,   217},
                                             {0,   255,    64},
                                             {255,    38,     0},
                                             {255,     0,   153},
                                             {0,   140,   255},
                                             {255,    77,     0},
                                             {255,   153,     0},
                                             {0,   255,   179},
                                             {0,   102,   255},
                                             {255,     0,    38},
                                             {13,   255,     0},
                                             {166,   255,     0},
                                             {0,   255,   255},
                                             {128,   255,     0},
                                             {255,     0,   230},
                                             {51,   255,     0}
                                         };
        int ind;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                ind = roi_mat(col,row);
                cim(col,row,0,0) = color_array[ind][0]; // red component
                cim(col,row,0,1) = color_array[ind][1]; // green
                cim(col,row,0,2) = color_array[ind][2]; // blue
            }
        }
    }

    void draw_hist(const Matrix<int>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(hist_mat.x_size()<w || hist_mat.y_size()<h) return;

        unsigned char color_array[][3] = {   {0  ,     0,     0},
                                             {204,     0,   255},
                                             {255,     0,     0},
                                             {0,   178,   255},
                                             {255,     0,   191},
                                             {255,   229,     0},
                                             {0,   255,   102},
                                             {89,   255,     0},
                                             {128,     0,   255},
                                             {242,     0,   255},
                                             {242,   255,     0},
                                             {255,     0,    77},
                                             {51,     0,   255},
                                             {0,   255,   140},
                                             {0,   255,    25},
                                             {204,   255,     0},
                                             {255,   191,     0},
                                             {89,     0,   255},
                                             {0,   217,   255},
                                             {0,    64,   255},
                                             {255,   115,     0},
                                             {255,     0,   115},
                                             {166,     0,   255},
                                             {13,     0,   255},
                                             {0,    25,   255},
                                             {0,   255,   217},
                                             {0,   255,    64},
                                             {255,    38,     0},
                                             {255,     0,   153},
                                             {0,   140,   255},
                                             {255,    77,     0},
                                             {255,   153,     0},
                                             {0,   255,   179},
                                             {0,   102,   255},
                                             {255,     0,    38},
                                             {13,   255,     0},
                                             {166,   255,     0},
                                             {0,   255,   255},
                                             {128,   255,     0},
                                             {255,     0,   230},
                                             {51,   255,     0}
                                         };
        int ind, x_size = hist_mat.x_size()-100;
        for (unsigned int row = 0; row < h; ++row)
        {
            for (unsigned int col = 0; col < w; ++col)
            {
                ind = hist_mat(x_size-col,row);
                cim(col,row,0,0) = color_array[ind][0]; // red component
                cim(col,row,0,1) = color_array[ind][1]; // green
                cim(col,row,0,2) = color_array[ind][2]; // blue
            }
        }
    }

    void draw_hist(const Matrix<double>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
    {
        if(hist_mat.x_size()<w || hist_mat.y_size()<h) return;

        int ind;//, x_size = hist_mat.x_size()-100, y_size = hist_mat.y_size()-110;
        double max = hist_mat(0,0), min = max;
        for(int i=1;i<hist_mat.total_size();++i)
        {
            if(max<hist_mat.data()[i]) max = hist_mat.data()[i];
            if(min>hist_mat.data()[i]) min = hist_mat.data()[i];
        }
        max-=min;
        int w1=200, h1=200;
        CImg<unsigned char> cim1(w1,h1,1,3);
        for (unsigned int row = 0; row < h1; ++row)
        {
            for (unsigned int col = 0; col < w1; ++col)
            {
                ind = (int)((hist_mat(col+300,row)-min)*255/max);
                cim1(col,row,0,0) = ind;//color_array[ind][0]; // red component
                cim1(col,row,0,1) = ind;//color_array[ind][1]; // green
                cim1(col,row,0,2) = ind;//color_array[ind][2]; // blue
            }
        }
        cim1.resize(cim);
        cim.draw_image(cim1);
    }

    void vis_gp(CImg<unsigned char>& cim, const Camera& camera)
    {
        double lx = -1.0, ux = 1.0;
        double lz = 0.5, uz = 8.0;
        double z2 = uz/4, z3 = uz/2, z4 = uz/4*3;
        double X[]={lx, lx, 0., 0., ux, ux, lx, ux, lx, ux, lx, ux, lx, ux};
        double Z[]={lz, uz, lz, uz, lz, uz, uz, uz, z4, z4, z3, z3,  z2,  z2};
        double c20 = camera.K()(2,0);
        double c00 = camera.K()(0,0);
        double c21 = camera.K()(2,1);
        double c11 = camera.K()(1,1);

        unsigned char color[3] = {255,255,0};
        char dis[100];
        for(int i=0; i<14; ++i)
        {
            double Y = (-last_gp(3)-last_gp(0)*X[i]-last_gp(2)*Z[i])/last_gp(1);
            int x1 = X[i]*c00/Z[i]+c20;
            int y1 = Y*c11/Z[i]+c21;
            ++i;
            Y = (-last_gp(3)-last_gp(0)*X[i]-last_gp(2)*Z[i])/last_gp(1);
            int x2 = X[i]*c00/Z[i]+c20;
            int y2 = Y*c11/Z[i]+c21;
            if(i>6)
            {
                sprintf(dis,"%0.1f",Z[i]);
                cim.draw_text(335,y2-21,dis,color,0,1,20);
            }
            cim.draw_line(x1,y1,x2,y2,color);
        }
    }

    std::string isometryToString(const Eigen::Isometry3d& m)
    {
        char result[80];
        memset(result, 0, sizeof(result));
        Eigen::Vector3d xyz = m.translation();
        Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
        snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
                 xyz(0), xyz(1), xyz(2),
                 rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
        return std::string(result);
    }

    struct HogParams
    {
        HogParams(DetectionTrackingSystem* obj, int f, unsigned char* img, Camera cam, Vector<Vector<double> >* detected_bb, Vector<Vector < double > >* OutputHOGdet)
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
        UpperParams(DetectionTrackingSystem* obj,Camera cam, Matrix<double>* dp, PointCloud* pc,Vector<Vector<double> >* detected_bb)
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

    static void* hog_thread_function(void* params)
    {
        HogParams* hog_params = (struct HogParams*) params;
        *(hog_params->OutputHOGdetL) = hog_params->_this->hog_detector.runHog(hog_params->frame,hog_params->image,hog_params->camera, *(hog_params->detected_bounding_boxes));
        pthread_exit(NULL);
    }

    static void* upper_thread_function(void* params)
    {
        UpperParams* upper_params = (struct UpperParams*) params;
        if(upper_params->_this->is_seg)
        {
            upper_params->_this->detector_seg->ProcessFrame(upper_params->camera, *(upper_params->depth_map), *(upper_params->point_cloud), upper_params->_this->upper_body_template, *(upper_params->detected_bounding_boxes));
        }
        else
        {
            upper_params->_this->detector->ProcessFrame(upper_params->camera, *(upper_params->depth_map), *(upper_params->point_cloud), upper_params->_this->upper_body_template, *(upper_params->detected_bounding_boxes));
        }
        pthread_exit(NULL);
    }

    int gp_count_down;
    int motion_not_valid_count;

    void main_process(unsigned char* b_image, float* b_depth, uint w, uint h)
    {
        cpu_time_start = CPUTime();
        double ct, fovis_time, PC_time, GP_time, detection_time, tracking_time;

        // Construct depth_map Matrix
        Matrix<double> depth_map(w,h);
        uint total_pixels = w*h;
        for(uint i=0;i<total_pixels;++i)
        {
            double v =(double)b_depth[i];
            if(isnan(v))
                depth_map.data()[i]=0.0;
            else
                depth_map.data()[i]=v;
        }

        ////////////////// FOVIS /////////////////////////////////////////////////////////////////
        ct=CPUTime();

        if(is_first)
        {
            fovis_rect = new fovis::Rectification(cam_params);
            fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
            odom = new fovis::VisualOdometry(fovis_rect, options);
            is_first=false;

            lodx=320;
            lodz=240;
            unsigned char cl[]={255,255,255};
            odimg.draw_circle((int)lodx,(int)lodz,2,cl);
        }
        if(!is_odom_valid)
        {
            fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
            delete odom;
            odom = new fovis::VisualOdometry(fovis_rect, options);

            lodx=320;
            lodz=240;
            odimg = CImg<unsigned char>(640,480,1,3,0);
            unsigned char cl[]={255,255,255};
            odimg.draw_circle((int)lodx,(int)lodz,2,cl);
        }

        fovis::DepthImage* fv_dp = new fovis::DepthImage(cam_params,w,h);
        fv_dp->setDepthImage(b_depth);

        //Computing Gray Image
        unsigned char* b_gray_image = new unsigned char[total_pixels];
        unsigned char* ptr_rgb = b_image;
        for(int i=0; i<total_pixels;++i)
        {
            b_gray_image[i] = (unsigned char)round(0.2125 * *(ptr_rgb++) +
                                                   0.7154 * *(ptr_rgb++) +
                                                   0.0721 * *(ptr_rgb++));
            //            b_gray_image[i] = (unsigned char)((*(ptr_rgb++)+*(ptr_rgb++)+*(ptr_rgb++))/3);
        }

        odom->processFrame(b_gray_image, fv_dp);

        if(!odom->getMotionEstimator()->isMotionEstimateValid())
        {
            gp_count_down = 10;
            dp_queue.push(b_depth);
            img_queue.push(b_image);
            delete fv_dp;
            is_last_gp_valid=false;
            is_odom_valid = true;
            if(motion_not_valid_count>10)
            {
                motion_not_valid_count = 0;
                is_odom_valid = false;
            }
            ++motion_not_valid_count;
            cout<<"-----------------motion is not valid."<<motion_not_valid_count<<"--------------------------------------"<<endl;
            return;
        }

        Eigen::Isometry3d cam_to_local = odom->getPose();
        Eigen::Isometry3d motion =  odom->getMotionEstimate();

        Eigen::Matrix4d m1 = cam_to_local.matrix().inverse();
        Eigen::Matrix4d m2 = motion.matrix().inverse();
        mm = Matrix<double>(4,4,m1.data());
        Matrix<double> motion_matrix(4,4,m2.data());

        is_odom_valid = true;
        for(int i = 0; i<mm.total_size(); ++i)
        {
            if(isnan(mm.data()[i]))
            {
                is_odom_valid = false;
                break;
            }
        }

        //if(!is_odom_valid)
        if(!odom->getMotionEstimator()->isMotionEstimateValid() || !is_odom_valid)
        {
            gp_count_down = 10;
            //            delete[] b_depth;
            dp_queue.push(b_depth);
            //            delete[] b_image;
            img_queue.push(b_image);
            delete fv_dp;
            is_last_gp_valid=false;
            cout<<"-----------------odom is not valid.--------------------------------------"<<endl;
            return;
        }

        ///////Just for visualising odometry///////////////////////////
        unsigned char cl[]={255,0,255};
        double nodx = lodx+motion_matrix(0,3)*100,
               nodz = lodz+motion_matrix(2,3)*100;
        odimg.draw_line_1((int)lodx,(int)lodz,(int)nodx,(int)nodz,cl,1);
        lodx=nodx; lodz=nodz;
        //////////////////////////////////////////////////////////////////

        Matrix<double> R(mm, 0,2,0,2);
        Vector<double> t(mm(3,0), mm(3,1), mm(3,2));

        fovis_time = CPUTime() - ct;
        ///////////////////////////////////////////////////////////////////////////////////////////////////


        /////////////////////// Point Cloud ///////////////////////////////////
        ct = CPUTime();
        PointCloud point_cloud(base_camera, depth_map);
        PC_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////


        //////////////////////// GP Estimation //////////////////////////////
        ct = CPUTime();
        //        motion_xyz += motion.translation();
        //        motion_rpy += motion.rotation().eulerAngles(0, 1, 2);
        //        double rx = fabs(motion_rpy(0) * 180/M_PI), ry = fabs(motion_rpy(1) * 180/M_PI), rz = fabs(motion_rpy(2) * 180/M_PI);

        //        double dot = fabs(motion_xyz(0)*last_gp(0)+motion_xyz(1)*last_gp(1)+motion_xyz(2)*last_gp(2))/motion_xyz.norm();
        //        cout<<dot<<endl;
        if(display.is_keyG() || !is_last_gp_valid || gp_count_down>0)// ||rx>1 || ry>1 || rz>1 ||/*fabs(motion_xyz(0))>0.01 || */fabs(motion_xyz(1))>0.01 /*|| fabs(motion_xyz(2))>0.01*/)
        {
            cout<<"gp estimation "<<gp_count_down<<endl;
            --gp_count_down;
            last_gp = GPEstimator.ComputeGroundPlane(point_cloud);
            is_last_gp_valid = true;
            //            motion_xyz = Eigen::Vector3d();
            //            motion_rpy = Eigen::Vector3d();
        }
        else
        {
            Matrix<double> rotate(motion_matrix, 0,2,0,2);
            Vector<double> t(motion_matrix(0,3), motion_matrix(1,3), motion_matrix(2,3));
            Vector<double> pv(last_gp(0), last_gp(1), last_gp(2));
            rotate.Transpose();
            pv = rotate * pv;

            double d = last_gp(3) - DotProduct(pv, t);

            last_gp(0) = pv(0)/pv.norm();
            last_gp(1) = pv(1)/pv.norm();
            last_gp(2) = pv(2)/pv.norm();
            last_gp(3) = d;
        }

        Camera camera(base_camera.K(), R, t, last_gp);
        Vector<double> gp1 = AncillaryMethods::PlaneToWorld(camera, last_gp);
        camera = Camera(camera.K(), R, t, gp1);
        GP_time = CPUTime()-ct;
        ////////////////////////////////////////////////////////////////////

        ////////////////////////// Visualization Part I/////////////////////
        CImg<unsigned char> cim_tmp(w,h,1,3);
        get_image(b_image,w,h,cim_tmp);
        CImg<unsigned char> cim_final(w*2,h,1,3);
        if(display_mode == IMAGE_MODE)
        {
            cim_final.draw_image(cim_tmp);
        }
        else if(display_mode == DEPTH_MODE)
        {
            get_depth(depth_map,w,h,cim_final,point_cloud,last_gp);
        }
        else if (display_mode == ODOM_MODE)
        {
            cim_final.draw_image(odimg);
            unsigned char cl[]={0,255,0};
            cim_final.draw_circle((int)lodx,(int)lodz,2,cl);
        }

        /////////////////////////Detection////////////////////////////////////////////
        Vector<Vector< double > > detected_bounding_boxes1;
        Vector<Vector < double > > OutputHOGdetL;
//        cout<<"--------------------------------------"<<endl;
        ct=CPUTime();
        pthread_t thog,tupp;

        if(use_HOG)
        {
            OutputHOGdetL = hog_detector.runHog(cnt,cim_tmp.get_permute_axes("cxyz").data(),camera, detected_bounding_boxes1);
//            HogParams hog_params(this,cnt,cim.get_permute_axes("cxyz").data(),camera, &detected_bounding_boxes1, &OutputHOGdetL);
//            pthread_create(&thog,NULL,SimpleOpenNIViewer::hog_thread_function,(void*)&hog_params);
        }

        Vector<Vector< double > > detected_bounding_boxes;

        UpperParams upper_params(this ,camera, &depth_map, &point_cloud, &detected_bounding_boxes);
        pthread_create(&tupp,NULL,DetectionTrackingSystem::upper_thread_function,(void*)&upper_params);

//        pthread_join(thog, NULL);
        pthread_join(tupp, NULL);

        Vector<double> oneDet(9);
        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            oneDet(0) = cnt;
            oneDet(1) = j;
            oneDet(2) = 1;
            oneDet(3) = 1 - detected_bounding_boxes(j)(4)+1; // make sure that the score is always positive
            oneDet(4) = detected_bounding_boxes(j)(0);
            oneDet(5) = detected_bounding_boxes(j)(1);
            oneDet(6) = detected_bounding_boxes(j)(2);
            oneDet(7) = detected_bounding_boxes(j)(3) * 3;
            oneDet(8) = detected_bounding_boxes(j)(5);
            OutputHOGdetL.pushBack(oneDet);
        }
        detected_bounding_boxes.append(detected_bounding_boxes1);

        detection_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////TRACKING///////////////////////////
        ct=CPUTime();
        tracker.process_tracking_oneFrame(HyposAll, det_comb, cnt, OutputHOGdetL, cim_tmp, camera);
        tracking_time = CPUTime()-ct;
        //////////////////////////////////////////////////////////////////////////////

        ////////////////////////// Visualization Part II////////////////////
        if(display_mode == ROI_MODE)
        {
            if(is_seg)
            {
                draw_roi(detector_seg->roi_image, w,h,cim_final);
                draw_hist(detector_seg->labeledROIs, w,h,cim_tmp);
//                draw_hist(detector_seg->occ_map, w,h,cim);
            }
            else
            {
                draw_roi(detector->roi_image, w,h,cim_final);
                draw_hist(detector->labeledROIs, w,h,cim_tmp);
            }
        }
        if(display_mode != ODOM_MODE)
        {
            vis_gp(cim_final, camera);
            for(int jj = 0; jj < detected_bounding_boxes.getSize(); ++jj)
                RenderBBox2D(detected_bounding_boxes(jj), cim_final, 0, 255, 0);
        }

        const unsigned char color[] = {0,255,0};
        const unsigned char bcolor[] = {50,50,50};
        cim_tmp.draw_text(10,10, "Tracking",color,0,1,20);
        cim_final.draw_text(10,10, "Detection",color,0,1,20);
        cim_final.draw_image(w,cim_tmp);

        if(show_help)
            cim_final.draw_text(650,200, help_string, color,bcolor,0.7,20);
        cim_final.draw_text(700,450,"Press F1 for more help!",color,bcolor,0.7,20);



        // Time
        char str[50];
        float fps=0;
        cpu_time_end = CPUTime();
        double diff = cpu_time_end - cpu_time_start;
        fps = 1.0/(diff);
        cpu_time_start = cpu_time_end;

        int l = 20;
        if(cnt%l==0)
        {
            fps_l = t_fps/l;
            t_fps=0;
        }
        t_fps +=fps;
        if(show_stat)
        {
            sprintf(str,"Total time per frame: %d", int(diff*1000));
            cim_final.draw_text(10,380,str,color,bcolor,0.7,20);
            sprintf(str,"%fFps", fps);
            cim_final.draw_text(10,410,str,color,bcolor,0.7,20);
            sprintf(str,"%fFps (average of %d frames)", fps_l, l);
            cim_final.draw_text(10,440,str,color,bcolor,0.7,20);

            if(display_mode != ODOM_MODE)
            {
                sprintf(str,"%d", int(fovis_time*1000));
                cim_final.draw_text(10,200,"Odometry",color,bcolor,0.7,20);
                cim_final.draw_text(170,200,str,color,bcolor,0.7,20);
                sprintf(str,"%d  (# of points %0.2f)", int(PC_time*1000), point_cloud.number_of_points/307200.0);
                cim_final.draw_text(10,225,"PointCloud",color,bcolor,0.7,20);
                cim_final.draw_text(170,225,str,color,bcolor,0.7,20);
                sprintf(str,"%d", int(GP_time*1000));
                cim_final.draw_text(10,250,"GP Estimation",color,bcolor,0.7,20);
                cim_final.draw_text(170,250,str,color,bcolor,0.7,20);
                sprintf(str,"%d", int(detection_time*1000));
                cim_final.draw_text(10,275,"Detector",color,bcolor,0.7,20);
                cim_final.draw_text(170,275,str,color,bcolor,0.7,20);
                sprintf(str,"%d", int(tracking_time*1000));
                cim_final.draw_text(10,300,"Tracker",color,bcolor,0.7,20);
                cim_final.draw_text(170,300,str,color,bcolor,0.7,20);
                // Fovis
//                cim1.draw_text(10,100,isometryToString(cam_to_local).c_str(),color,bcolor,0.7,20);
            }
        }

        const unsigned char rcolor[3]= {255,0,0};
        if(record_sequence)
            cim_final.draw_text(10,150,"RECORDING ...", rcolor, bcolor, 0.7, 30);

        display.display(cim_final);
        //1024 x 768
        cim_final.resize(1024);
        CImg<unsigned char> rimg(1024,768,1,3,0);
        rimg.draw_image(0,(rimg.height() - cim_final.height())/2,cim_final);

        unsigned char* res = new unsigned char[rimg.size()];
        memcpy(res,rimg.get_permute_axes("cxyz").data(), rimg.size());
        result_image_queue.push(res);

        if(capture || display.is_keyC())
        {
            static unsigned int oi=0;
            sprintf(capture_path,path,++oi);
            cim_final.save(capture_path);
            capture = false;
        }
        /////////////////////////////////////////////////////////////////////////

        ++cnt;

        dp_queue.push(b_depth);
        img_queue.push(b_image);

        delete[] b_gray_image;
        delete fv_dp;
    }

    static DetectionTrackingSystem* _this;

    static void grabber_callback(const float *depth, const unsigned char *image)
    {
        uint w = Globals::dImWidth, h = Globals::dImHeight;

        // Get RGB Image
        unsigned char* b_image = new unsigned char[w*h*3];
        memcpy(b_image, image, w*h*3);

        // Get DepthMap
        float* b_depth = new float[h*w];
        memcpy(b_depth, depth, w*h*sizeof(float));

        _this->main_process(b_image, b_depth, w, h);
    }

    void get_from_file(long int frm)
    {
        char pth[200];
        sprintf(pth,"%s/img_%08d",Globals::from_file_path.c_str(),frm);
        ifstream imgf(pth, ios::binary);
        sprintf(pth,"%s/dp_%08d",Globals::from_file_path.c_str(),frm);
        ifstream dpf(pth,ios::binary);
        if(!imgf.is_open() || !dpf.is_open())
            return;

        uint w = Globals::dImWidth, h = Globals::dImHeight;

        unsigned char* b_image = new unsigned char[w*h*3];
        imgf.read((char*)b_image, Globals::dImWidth*Globals::dImHeight*3);
        imgf.close();

        float* b_depth = new float[h*w];
        dpf.read((char*)b_depth, Globals::dImWidth*Globals::dImHeight*sizeof(float));
        dpf.close();

        main_process(b_image, b_depth, Globals::dImWidth, Globals::dImHeight);
    }

    static void* get_files(void* obj)
    {
        DetectionTrackingSystem* o = (DetectionTrackingSystem*)obj;
        long int frm1 = 0;
        while(1)
            o->get_from_file(frm1++);
    }

    // send to stream, save images and depth and clean memory
    void sink_frame(long int& frm, bool is_runing = true)
    {
        char pth[200];

        // Try to send the result_image to the stream(s)
        if(result_image_queue.size()>0)
        {
            unsigned char* ptr2 = result_image_queue.front();
            if(ptr2)
            {
                result_image_queue.pop();
                try{
                if(is_runing)
                    streaming.sendImage(ptr2, 3*1024*768);
                delete[] ptr2;
                }catch(...){}
            }
        }

        // Record image and depth map (in recording mode only,
        // then delete image and depth from memory.
        if(img_queue.size()>0 && dp_queue.size()>0)
        {
            unsigned char* ptr = img_queue.front();
            float* ptr1 = dp_queue.front();
            img_queue.pop();
            dp_queue.pop();

            if(record_sequence)
            {
                sprintf(pth,"rec/img_%08d",frm);
                ofstream imgf(pth, ios::binary);
                sprintf(pth,"rec/dp_%08d",frm);
                ofstream dpf(pth, ios::binary);
                imgf.write((char*)ptr,640*480*3);
                dpf.write((char*)ptr1,640*480*sizeof(float));
                imgf.close();
                dpf.close();
            }

            try
            {
                if(ptr)
                    delete[] ptr;
                if(ptr1)
                    delete[] ptr1;
            }
            catch(...){}
            ++frm;
        }
    }

    void run ()
    {
        Grabber* grabber=0;
        pthread_t th;

        if(Globals::from_camera)
        {
            grabber = new Grabber(grabber_callback);
            grabber->start();
        }
        else
            pthread_create(&th,NULL,&DetectionTrackingSystem::get_files, (void*)this);

        long int frm = 0;

        while(!display.is_closed())
        {
            check_keys();
            sink_frame(frm);
        }

        if(Globals::from_camera)
            grabber->stop();
        else
            pthread_cancel(th);

        while(dp_queue.size()>0)
        {
            sink_frame(frm,false);
        }
    }

    void check_keys()
    {
        if(display.is_keyC())
        {
            capture = true;
        }
        else if(display.is_keyI())
        {
            display_mode = IMAGE_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyD())
        {
            display_mode = DEPTH_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyO())
        {
            display_mode = ODOM_MODE;
            detector_seg->visualize_roi=false;
            detector->visualize_roi=false;
        }
        else if(display.is_keyR())
        {
            display_mode = ROI_MODE;
            detector_seg->visualize_roi=true;
            detector->visualize_roi=true;
        }
        else if(display.is_keyW())
        {
            if(is_seg)
            {
                is_seg = false;
                SetTitle();
            }
        }
        else if(display.is_keyQ())
        {
            if(!is_seg)
            {
                is_seg = true;
                SetTitle();
            }
        }
        else if(display.is_keyA())
        {
            use_HOG = true;
            SetTitle();
        }
        else if(display.is_keyS())
        {
            use_HOG = false;
            SetTitle();
        }
        else if(display.is_key1())
        {
            detector_mode = DEPTH_DETECTOR;
            depth_detector.visualize_roi = detector->visualize_roi;
            detector = &depth_detector;
            depth_detector_seg.visualize_roi = detector_seg->visualize_roi;
            detector_seg = &depth_detector_seg;
            SetTitle();
        }
        else if(display.is_key2())
        {
            detector_mode = DEPTH_LM_DETECTOR;
            depth_detector_lm.visualize_roi = detector->visualize_roi;
            detector = &depth_detector_lm;
            depth_detector_lm_seg.visualize_roi = detector_seg->visualize_roi;
            detector_seg = &depth_detector_lm_seg;
            SetTitle();
        }
        else if(display.is_keyF1())
        {
            show_help = true;
        }
        else if(display.is_keyF2())
        {
            show_help = false;
        }
        else if(display.is_keyF5())
        {
            record_sequence = true;
        }
        else if(display.is_keyF6())
        {
            record_sequence = false;
        }
        else if(display.is_keyZ())
            show_stat = true;
        else if(display.is_keyX())
            show_stat = false;
    }

    void SetTitle()
    {
        char title[300];
        char pattern[] = "Upper Body Detector - %s <without ROI-Segmentation> - %s";
        char patern_seg[] = "Upper Body Detector - %s <with ROI-Segmentation> - %s";
        char* p = is_seg ? patern_seg : pattern;
        char hog_title[100];
        if(use_HOG)
            sprintf(hog_title,"%s","With groundHOG");
        else
            sprintf(hog_title,"%s","Without groundHOG");

        switch(detector_mode)
        {
        case DEPTH_DETECTOR:
            sprintf(title,p,"Depth Detector",hog_title);
            break;
        case DEPTH_LM_DETECTOR:
            sprintf(title,p,"Local Max Depth Detector",hog_title);
            break;
        }

        display.set_title(title);
    }

    void init()
    {
        use_HOG = Globals::use_hog;
        gp_count_down=0;
        motion_not_valid_count = 0;
        cnt = 0;
        t_fps=0;
        fps_l=0;

        strcpy(path,"%d.png");
        strcpy(help_string,
               "1, 2       Select detector: 1)Depth  2)Local Maximum Depth\n"
               "q/w       Activate/Deactivate Segmentation ROI\n"
               "a/s       Activate/Deactivate groundHOG\n"
               "d         Show Depth-Map\n"
               "i         Show RGB Image\n"
               "r         Show ROI\n"
               "o         Show Odometry\n"
               "c         Capture frame\n"
               "F5/F6     Start/Stop recording\n"
               "z/x       Show/Hide Statistics\n"
               "F1/F2     Show/Hide Help\n");

        ReadUpperBodyTemplate(upper_body_template);
        capture=false;
        display_mode = IMAGE_MODE;
        if(Globals::use_local_max)
            detector_mode = DEPTH_LM_DETECTOR;
        else
            detector_mode = DEPTH_DETECTOR;
        show_help = false;
        show_stat = false;
        is_seg=Globals::use_segmentation_roi;

        is_first=true;
        is_odom_valid = true;

        cpu_time_start = CPUTime();
        detector = &depth_detector;
        detector_seg = &depth_detector_seg;

        last_gp = base_camera.get_GP();
        is_last_gp_valid = false;
        is_gp_estim_ignored = false;

        record_sequence = false;

        memset(&cam_params, 0, sizeof(fovis::CameraIntrinsicsParameters));
        cam_params.width = Globals::dImWidth;
        cam_params.height = Globals::dImHeight;
        cam_params.fx = base_camera.K()(0,0);
        cam_params.fy = base_camera.K()(1,1);
        cam_params.cx = base_camera.K()(2,0);
        cam_params.cy = base_camera.K()(2,1);

        SetTitle();
    }

    DetectionTrackingSystem() : display(640*2*1.4,480*1.4), det_comb(23,0),
        base_camera(Globals::camPath_left.c_str(),true),
        streaming(90,15,"iPad"),
        odimg(640,480,1,3,0)
    {
        init();
    }

    ~DetectionTrackingSystem()
    {
        delete odom;
        delete fovis_rect;
    }

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
    Eigen::Vector3d motion_xyz;
    Eigen::Vector3d motion_rpy;

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
};

DetectionTrackingSystem* DetectionTrackingSystem::_this = NULL;

///////////////////////////////////////////////////////////////////////
// PreLoadData:
//      Reads required data from files.
// parameters:
//      output:
//          cameras     -   Camera settings that is related to each frame.
//          images      -   Input images for each frame.
//          depth_maps  -   Depth maps of input images.
///////////////////////////////////////////////////////////////////////
void PreLoadData(Vector<Camera>& cameras, Vector<CImg<unsigned char> >& images, Vector<Matrix<double> >& depth_maps)
{
    char path[128];
    char pToGp[128];

    cameras.setSize(Globals::numberFrames);
    depth_maps.setSize(Globals::numberFrames);
    //    if(Globals::export_result_images)
    images.setSize(Globals::numberFrames);

    for (int i = Globals::nOffset, j = 0; i < Globals::nOffset+Globals::numberFrames; i++, j++)
    {

        if(i%10==0)
        {
            cout << "Loading " << i << " out of " << Globals::nOffset+Globals::numberFrames << endl;
        }

        // ------------------- LOAD CAMERAS
        sprintf(path, Globals::camPath_left.c_str(), i);
        Camera left_camera(path, 1);

        if(Globals::is_project_plane_to_world)
        {
            Matrix<double> left_camera_R = left_camera.get_R();
            Vector<double> left_camera_T = left_camera.get_t();
            Matrix<double> left_camera_K = left_camera.get_K();

            sprintf(pToGp, Globals::path_to_planes.c_str(), i);
            Vector<double> gp;
            gp.readTXT(pToGp, 4);

            gp(3) *= 1.0/Globals::WORLD_SCALE;
            gp = AncillaryMethods::PlaneToWorld(left_camera, gp);
            left_camera = Camera(left_camera_K, left_camera_R, left_camera_T, gp);
        }

        cameras(j) = left_camera;

        // ------------------- LOAD RGB IMAGES
        //        if(Globals::export_result_images)
        //        {
        sprintf(path, Globals::sImagePath_left.c_str(), i);
        //            QImage imageLeft(path);
        //            images(j) = imageLeft;
        images(j).load(path);
        //        }

        // ------------------- LOAD DEPTH
        depth_maps(j) = AncillaryMethods::GetDepthLibelas(i, left_camera, Globals::baseline);
    }
}

Vector<Vector<double> > BboxNMS(Vector<Vector<double> >& detected_bounding_boxes, double thresh)
{
    Vector<int> remove(detected_bounding_boxes.getSize(), -1);
    Vector<double> interRect;
    for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
    {

        if(remove(i) > 0)
            continue;

        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            if(j == i || remove(j) > 0)
                continue;

            AncillaryMethods::IntersetRect(detected_bounding_boxes(i), detected_bounding_boxes(j), interRect);
            double intersection = interRect(2)*interRect(3);
            double unionRect = detected_bounding_boxes(i)(2)*detected_bounding_boxes(i)(3)+
                    detected_bounding_boxes(j)(2)*detected_bounding_boxes(j)(3) - intersection;



            if(intersection/unionRect> thresh)
            {
                // Compare distances (reverse of score)
                if(detected_bounding_boxes(i)(4) < detected_bounding_boxes(j)(4))
                {
                    remove(j) = 1;
                }
                else
                {
                    remove(i) = 1;
                    break;
                }
            }
        }
    }

    Vector<Vector<double> > detected_bounding_boxes_after_nms;
    for(int i = 0; i < detected_bounding_boxes.getSize(); ++i)
    {
        if(remove(i) < 0)
        {
            detected_bounding_boxes_after_nms.pushBack(detected_bounding_boxes(i));
        }
    }
    return detected_bounding_boxes_after_nms;
}

void WriteToFile(Vector<Vector< double > > detected_bounding_boxes, int index, ofstream& det_file, bool thriple=true, bool reverse_score=true)
{
    char filename[50];
    sprintf(filename,"image_%08d_0.png",index);

    if(!det_file.is_open())
    {
        cout<<"bad file path."<<endl;
        return;
    }

    double x,y,sc,x2,y2;
    int w,h;

    det_file<<"\""<<filename<<"\"";

    for(int i =0; i<detected_bounding_boxes.getSize();++i)
    {
        x = detected_bounding_boxes(i)(0);
        y = detected_bounding_boxes(i)(1);
        w = detected_bounding_boxes(i)(2);
        h = detected_bounding_boxes(i)(3);
        if(thriple)
            h*=3;
        sc = detected_bounding_boxes(i)(4);
        if(reverse_score)
            sc=1-sc;
        x2=(x+w)>640?640:x+w;
        y2=(y+h)>480?480:y+h;

        if(i==0)
            det_file<<":("<<(int)x<<","<<(int)y<<","<<(int)x2<<","<<(int)y2<<"):"<<sc;
        else
            det_file<<",("<<(int)x<<","<<(int)y<<","<<(int)x2<<","<<(int)y2<<"):"<<sc;
    }
    det_file<<";"<<endl;
}

Vector<double> getBBoxFrom3DPos(Camera cam, Vector<double> pos, double height, bool occFlag)
{

    Vector<double> bbox;

    //******************************
    // Get Ground Plane
    //******************************
    Vector<double> gpn=cam.get_GPN();
    Vector<double> copyGPN;

    //*******************************
    // Get further Camera parameter
    //*******************************

    Vector<double> VPN=cam.get_VPN();

    //*******************************
    // Init bbox and score
    //*******************************
    Vector<double> x(pos);
    Vector<double> xTop;

    //*************************************
    // Project x on the Ground Plane
    //*************************************

    cam.ProjectToGP(x, Globals::WORLD_SCALE, x);

    //**************************************
    // Compute x Top
    //**************************************
    xTop = x;
    copyGPN = gpn;
    copyGPN *=height;
    xTop -= copyGPN;

    //***************************************
    // Check if x is in front of the Camera
    //***************************************

    double invWorldScale = 1.0/Globals::WORLD_SCALE;
    Vector<double> auxX = x * invWorldScale;
    Vector<double> auxXTop = xTop * invWorldScale;

    if(!(cam.isPointInFrontOfCam(auxX) && cam.isPointInFrontOfCam(auxXTop)))
    {
        bbox.clearContent();
        return bbox;
    }

    //***************************************
    // Projection on Screen
    //***************************************

    Vector<double> p1;
    Vector<double> p2;


    cam.WorldToImage(x, Globals::WORLD_SCALE, p1);

    //    if(p1(0) < 0 || p1(0) >= Globals::dImWidth || p1(1) < 0 || p1(1) >= Globals::dImHeight)
    //    {
    //        bbox.clearContent();
    //        return bbox;
    //    }
    cam.WorldToImage(xTop, Globals::WORLD_SCALE, p2);

    Vector<double> diffP2P1(2);
    diffP2P1(0)= fabs(p2(0)) - fabs(p1(0));
    diffP2P1(1)= fabs(p2(1)) - fabs(p1(1));

    double ht = diffP2P1.norm();

    double wt_half;
    if(occFlag)
    {
        wt_half = ht / 4;
    }
    else
    {
        wt_half = ht / 5;
    }

    bbox.clearContent();;
    bbox.pushBack(floor(p1(0) - wt_half));
    bbox.pushBack(floor(p1(1) - ht));
    bbox.pushBack(floor(p1(0) + wt_half));
    bbox.pushBack(floor(p1(1)));

    bbox(0) = max(0.0,bbox(0));
    bbox(1) = max(0.0,bbox(1));

    bbox(2) = min((double)Globals::dImWidth-1,bbox(2));
    bbox(2) = fabs(bbox(2)) - fabs(bbox(0));

    bbox(3) = min((double)Globals::dImHeight-1,bbox(3));
    bbox(3) = fabs(bbox(3)) - fabs(bbox(1));

    //******************************************
    // Check if the bbox out of frame
    //******************************************

    if(bbox(0) < 0) bbox(0) = 0;
    if(bbox(0) + bbox(2)>=Globals::dImWidth) bbox(2) = Globals::dImWidth-bbox(0)-1;

    if(bbox(1) < 0) bbox(1) = 0;
    if(bbox(1)+bbox(3) >= Globals::dImHeight) bbox(3) = Globals::dImHeight - bbox(1) - 1;

    //    if(bbox(2) >= Globals::dImWidth) bbox(2) = Globals::dImWidth-1;
    //    if(bbox(2) < 0) bbox(2) = 0;

    //    if(bbox(0) >= Globals::dImWidth) bbox(0) = Globals::dImWidth-1;
    //    if(bbox(0) < 0) bbox(0) = 0;

    //    if(bbox(1) >= Globals::dImHeight) bbox(1) = Globals::dImHeight-1;
    //    if(bbox(1) < 0) bbox(1) = 0;

    //    if(bbox(3) >= Globals::dImHeight) bbox(3) = Globals::dImHeight-1;
    //    if(bbox(3) < 0) bbox(3) = 0;

    //    if(bbox(2)-bbox(0) == 0 || bbox(3)-bbox(1) == 0) bbox.clearContent();

    return bbox;
}

void exportBBOX(Vector<Hypo> Hypos, Camera cam, int frame, Vector<Vector<double> >& bboxes)
{

    // How create a ofstream
    // ofstream aStream('filename');

    Matrix<double> mat( 13, Hypos.getSize(), 0.0);
    Vector<Vector<double> >TrajPts;
    Vector<double> score;
    Vector<double> bb;

    Hypo hypo;
    Matrix<double> allP;

    for(int i = 0; i < Hypos.getSize(); i++)
    {
        hypo = Hypos(i);

        hypo.getXProj(allP);
        Vector<double> vX = allP.getRow(allP.y_size()-1);
        vX(2) = vX(1);
        vX(1) = vX(3);
        vX.resize(3);

        double height = hypo.getHeight();

        bb = getBBoxFrom3DPos(cam, vX, min(1.98,height), false);
        if(bb.getSize()==4)
        {
            score.pushBack(hypo.getScoreMDL());
            bb.pushBack(hypo.getScoreMDL());
            bboxes.pushBack(bb);
        }

    }
}

///////////////////////////////////////////////////////////////////////
// for timing the preload part
///////////////////////////////////////////////////////////////////////
time_t  user_time_start, user_time_end;
double  cpu_time_start, cpu_time_end;

int main_preload()
{
    Vector<Camera> cameras;
    Vector<CImg<unsigned char> > images;
    Vector<Matrix<double> > depth_maps;
    PreLoadData(cameras, images, depth_maps);

    Matrix<double> upper_body_template;

    ReadUpperBodyTemplate(upper_body_template);

    // Timing Code - Start
    time(&user_time_start);
    cpu_time_start = CPUTime();

    //    Detector detector;
    Detector* detector=0;
    Detector_Seg* detector_seg=0;
    if(Globals::use_segmentation_roi)
    {
        if(Globals::use_local_max)
            detector_seg = new DepthDetector_LM_Seg();
        else
            detector_seg = new DepthDetector_Seg();
    }
    else
    {
        if(Globals::use_local_max)
            detector = new DepthDetector_LM();
        else
            detector = new DepthDetector();
    }
    HOGDetector hog_detector;
    hog_detector.rescore=true;

    GroundPlaneEstimator GPEstimator;
    Tracker tracker;
    Vector< Hypo > HyposAll;
    Detections det_comb(23,0);


    ofstream* det_file;
    if(Globals::export_bounding_box)
    {
        cout<<Globals::bounding_box_path.c_str()<<endl;
        det_file = new ofstream(Globals::bounding_box_path.c_str());
    }

    CImg<unsigned char> cim_out(Globals::dImWidth*2,Globals::dImHeight,1,3);

    for(int current_frame = 0; current_frame < Globals::numberFrames; current_frame++)
    {
        int cnt = current_frame+Globals::nOffset;
        if(Globals::verbose){
            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
            cout << "\33[33;40;1m" <<"                                  Processing image " << current_frame + Globals::nOffset << "\33[0m"<< endl;
            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
        }

        ///////////////////////////////////////////////////////////////////////////
        // main Process
        PointCloud point_cloud(cameras(current_frame), depth_maps(current_frame));
        //        Vector<double> gp = GPEstimator.ComputeGroundPlane(point_cloud);
        Vector<Vector< double > > detected_bounding_boxes;
        if(Globals::use_segmentation_roi)
            detector_seg->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
        else
            detector->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
        ///////////////////////////////////////////////////////////////////////////

        CImg<unsigned char> cnt_image=images(current_frame);

        Vector<Vector < double > > OutputHOGdetL;
        if(Globals::use_hog)
            OutputHOGdetL = hog_detector.runHogPr2(cnt,cnt_image.get_permute_axes("cxyz").data(),cameras(current_frame), detected_bounding_boxes);

        bool multiply_by_3 = false;
        for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
        {
            detected_bounding_boxes(i)(3) = detected_bounding_boxes(i)(3) *3.0;
        }

        if(Globals::use_hog)
            detected_bounding_boxes = BboxNMS(detected_bounding_boxes,0.5);

        // Tracking ////////////////////////////////////////////////////
        Vector<double> oneDet(9);
        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
        {
            oneDet(0) = cnt;
            oneDet(1) = j;
            oneDet(2) = 1;
            oneDet(3) = 1 - detected_bounding_boxes(j)(4)+1; // make sure that the score is always positive
            oneDet(4) = detected_bounding_boxes(j)(0);
            oneDet(5) = detected_bounding_boxes(j)(1);
            oneDet(6) = detected_bounding_boxes(j)(2);
            oneDet(7) = detected_bounding_boxes(j)(3);
            oneDet(8) = detected_bounding_boxes(j)(5);
            OutputHOGdetL.pushBack(oneDet);
        }
        tracker.process_tracking_oneFrame(HyposAll, det_comb, cnt, OutputHOGdetL, cnt_image, cameras(current_frame));

        ////////////////////////////////////////////////////////////////


        if(Globals::export_bounding_box)
        {
            // without tracking
//            WriteToFile(detected_bounding_boxes,current_frame+Globals::nOffset,*det_file,multiply_by_3);

            // with tracking
            Vector<Vector<double> > bboxes;
            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
            WriteToFile(bboxes,current_frame+Globals::nOffset,*det_file,multiply_by_3,false);
        }

        if(Globals::export_result_images)
        {
            //without tracking
//            for(int jj = 0; jj < detected_bounding_boxes.getSize(); jj++)
//            {
//                detected_bounding_boxes(jj)(3)/=3;
//                RenderBBox2D(detected_bounding_boxes(jj), images[current_frame], 255, 0, 0);
//            }

            // with tracker
            Vector<Vector<double> > bboxes;
            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
            for(int jj = 0; jj < bboxes.getSize(); jj++)
            {
                bboxes(jj)(3)/=3.0;
                RenderBBox2D(bboxes(jj), images[current_frame], 255, 0, 0);
            }

            char path[128];
            sprintf(path, Globals::result_images_path.c_str(), current_frame + Globals::nOffset);
            cim_out.draw_image(images(current_frame));
            cim_out.draw_image(Globals::dImWidth,cnt_image);
            cim_out.save(path);
        }
    }
    if(Globals::export_bounding_box)
    {
        det_file->close();
    }

    // Timing Code - End
    time(&user_time_end);
    cpu_time_end = CPUTime();
    cout << "TIMING :" << cpu_time_end-cpu_time_start << "s (system), "
         << user_time_end-user_time_start << "s (user)" << endl;

    delete detector;
    delete detector_seg;

    return 0;
}

int main(int argc, char** argv)
{
    ProcessCommandArgs(argc, argv);
    ReadConfigFile();

    if(Globals::preload)
        main_preload();
    else
    {
        DetectionTrackingSystem v;
        DetectionTrackingSystem::_this = &v;
        v.run();
    }

    cout<<"end"<<endl;
    exit(0);
    return 0;
}
