#include "Grabber.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <fstream>
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

#include "detectiontrackingsystem.h"
#include "timing.h"

string path_config_file = "config_Asus.inp";

///////////////////////////////////////////////////////////////////////
// for timing the code
///////////////////////////////////////////////////////////////////////
//time_t  user_time_start, user_time_end;
//double  cpu_time_start, cpu_time_end;

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
    Globals::use_raw_format = config.read("use_raw_format",true);

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

//int main_preload()
//{
//    Vector<Camera> cameras;
//    Vector<CImg<unsigned char> > images;
//    Vector<Matrix<double> > depth_maps;
//    PreLoadData(cameras, images, depth_maps);
//
//    Matrix<double> upper_body_template;
//
//    ReadUpperBodyTemplate(upper_body_template);
//
//    // Timing Code - Start
//    time(&user_time_start);
//    cpu_time_start = CPUTime();
//
//    //    Detector detector;
//    Detector* detector=0;
//    Detector_Seg* detector_seg=0;
//    if(Globals::use_segmentation_roi)
//    {
//        if(Globals::use_local_max)
//            detector_seg = new DepthDetector_LM_Seg();
//        else
//            detector_seg = new DepthDetector_Seg();
//    }
//    else
//    {
//        if(Globals::use_local_max)
//            detector = new DepthDetector_LM();
//        else
//            detector = new DepthDetector();
//    }
//    HOGDetector hog_detector;
//    hog_detector.rescore=true;
//
//    GroundPlaneEstimator GPEstimator;
//    Tracker tracker;
//    Vector< Hypo > HyposAll;
//    Detections det_comb(23,0);
//
//
//    ofstream* det_file;
//    if(Globals::export_bounding_box)
//    {
//        cout<<Globals::bounding_box_path.c_str()<<endl;
//        det_file = new ofstream(Globals::bounding_box_path.c_str());
//    }
//
//    CImg<unsigned char> cim_out(Globals::dImWidth*2,Globals::dImHeight,1,3);
//
//    for(int current_frame = 0; current_frame < Globals::numberFrames; current_frame++)
//    {
//        int cnt = current_frame+Globals::nOffset;
//        if(Globals::verbose){
//            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
//            cout << "\33[33;40;1m" <<"                                  Processing image " << current_frame + Globals::nOffset << "\33[0m"<< endl;
//            cout << "\33[33;40;1m" << "---------------------------------------------------------------------------------------------------------------" << "\33[0m" << endl;
//        }
//
//        ///////////////////////////////////////////////////////////////////////////
//        // main Process
//        PointCloud point_cloud(cameras(current_frame), depth_maps(current_frame));
//        //        Vector<double> gp = GPEstimator.ComputeGroundPlane(point_cloud);
//        Vector<Vector< double > > detected_bounding_boxes;
//        if(Globals::use_segmentation_roi)
//            detector_seg->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
//        else
//            detector->ProcessFrame(cameras(current_frame), depth_maps(current_frame), point_cloud, upper_body_template, detected_bounding_boxes);
//        ///////////////////////////////////////////////////////////////////////////
//
//        CImg<unsigned char> cnt_image=images(current_frame);
//
//        Vector<Vector < double > > OutputHOGdetL;
//        if(Globals::use_hog)
//            OutputHOGdetL = hog_detector.runHogPr2(cnt,cnt_image.get_permute_axes("cxyz").data(),cameras(current_frame), detected_bounding_boxes);
//
//        bool multiply_by_3 = false;
//        for(int i = 0; i < detected_bounding_boxes.getSize(); i++)
//        {
//            detected_bounding_boxes(i)(3) = detected_bounding_boxes(i)(3) *3.0;
//        }
//
//        if(Globals::use_hog)
//            detected_bounding_boxes = BboxNMS(detected_bounding_boxes,0.5);
//
//        // Tracking ////////////////////////////////////////////////////
//        Vector<double> oneDet(9);
//        for(int j = 0; j < detected_bounding_boxes.getSize(); ++j)
//        {
//            oneDet(0) = cnt;
//            oneDet(1) = j;
//            oneDet(2) = 1;
//            oneDet(3) = 1 - detected_bounding_boxes(j)(4)+1; // make sure that the score is always positive
//            oneDet(4) = detected_bounding_boxes(j)(0);
//            oneDet(5) = detected_bounding_boxes(j)(1);
//            oneDet(6) = detected_bounding_boxes(j)(2);
//            oneDet(7) = detected_bounding_boxes(j)(3);
//            oneDet(8) = detected_bounding_boxes(j)(5);
//            OutputHOGdetL.pushBack(oneDet);
//        }
//        tracker.process_tracking_oneFrame(HyposAll, det_comb, cnt, OutputHOGdetL, cnt_image, cameras(current_frame));
//
//        ////////////////////////////////////////////////////////////////
//
//
//        if(Globals::export_bounding_box)
//        {
//            // without tracking
////            WriteToFile(detected_bounding_boxes,current_frame+Globals::nOffset,*det_file,multiply_by_3);
//
//            // with tracking
//            Vector<Vector<double> > bboxes;
//            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
//            WriteToFile(bboxes,current_frame+Globals::nOffset,*det_file,multiply_by_3,false);
//        }
//
//        if(Globals::export_result_images)
//        {
//            //without tracking
////            for(int jj = 0; jj < detected_bounding_boxes.getSize(); jj++)
////            {
////                detected_bounding_boxes(jj)(3)/=3;
////                RenderBBox2D(detected_bounding_boxes(jj), images[current_frame], 255, 0, 0);
////            }
//
//            // with tracker
//            Vector<Vector<double> > bboxes;
//            exportBBOX(tracker.HyposMDL, cameras(current_frame), current_frame, bboxes);
//            for(int jj = 0; jj < bboxes.getSize(); jj++)
//            {
//                bboxes(jj)(3)/=3.0;
//                RenderBBox2D(bboxes(jj), images[current_frame], 255, 0, 0);
//            }
//
//            char path[128];
//            sprintf(path, Globals::result_images_path.c_str(), current_frame + Globals::nOffset);
//            cim_out.draw_image(images(current_frame));
//            cim_out.draw_image(Globals::dImWidth,cnt_image);
//            cim_out.save(path);
//        }
//    }
//    if(Globals::export_bounding_box)
//    {
//        det_file->close();
//    }
//
//    // Timing Code - End
//    time(&user_time_end);
//    cpu_time_end = CPUTime();
//    cout << "TIMING :" << cpu_time_end-cpu_time_start << "s (system), "
//         << user_time_end-user_time_start << "s (user)" << endl;
//
//    delete detector;
//    delete detector_seg;
//
//    return 0;
//}

int main(int argc, char** argv)
{
    ProcessCommandArgs(argc, argv);
    ReadConfigFile();

    if (Globals::preload) {
        // main_preload();
        cerr << "Preload mode not supported" << endl;
        exit(-1);  // FIXME: This is broken
    } else {
        DetectionTrackingSystem *v=DetectionTrackingSystem::getInstance();
        v->run();
    }

    cout << "end" << endl;
    exit(0);
    return 0;
}
