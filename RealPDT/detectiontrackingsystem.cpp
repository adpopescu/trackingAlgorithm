//
// Created by dtenty on 06/08/15.
//

#include "detectiontrackingsystem.h"

DetectionTrackingSystem* DetectionTrackingSystem::_this = NULL;

void DetectionTrackingSystem::get_image(unsigned char* b_image, uint w, uint h, CImg<unsigned char>& cim)
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

void DetectionTrackingSystem::get_depth(const Matrix<double>& depth_map, uint w, uint h, CImg<unsigned char>& cim, PointCloud pc, Vector<double> gp)
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

void DetectionTrackingSystem::draw_roi(const Matrix<int>& roi_mat, uint w, uint h, CImg<unsigned char>& cim)
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

void DetectionTrackingSystem::draw_hist(const Matrix<int>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
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

void DetectionTrackingSystem::draw_hist(const Matrix<double>& hist_mat, uint w, uint h, CImg<unsigned char>& cim)
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

void DetectionTrackingSystem::vis_gp(CImg<unsigned char>& cim, const Camera& camera)
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

string DetectionTrackingSystem::isometryToString(const Eigen::Isometry3d &m) {
    char result[80];
    memset(result, 0, sizeof(result));
    Eigen::Vector3d xyz = m.translation();
    Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
    snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
             xyz(0), xyz(1), xyz(2),
             rpy(0) * 180 / M_PI, rpy(1) * 180 / M_PI, rpy(2) * 180 / M_PI);
    return std::string(result);
}


void* DetectionTrackingSystem::hog_thread_function(void* params)
{
    HogParams* hog_params = (struct HogParams*) params;
    *(hog_params->OutputHOGdetL) = hog_params->_this->hog_detector.runHog(hog_params->frame,hog_params->image,hog_params->camera, *(hog_params->detected_bounding_boxes));
    pthread_exit(NULL);
}

void* DetectionTrackingSystem::upper_thread_function(void* params)
{
    UpperParams* upper_params = (struct UpperParams*) params;
    if(upper_params->_this->is_seg)
    {
        upper_params->_this->detector_seg->ProcessFrame(upper_params->camera,
                                                        *(upper_params->depth_map),
                                                        *(upper_params->point_cloud),
                                                        upper_params->_this->upper_body_template,
                                                        *(upper_params->detected_bounding_boxes));
    }
    else
    {
        upper_params->_this->detector->ProcessFrame(upper_params->camera, *(upper_params->depth_map), *(upper_params->point_cloud), upper_params->_this->upper_body_template, *(upper_params->detected_bounding_boxes));
    }
    pthread_exit(NULL);
}

void* DetectionTrackingSystem::get_files(void* obj)
{
    DetectionTrackingSystem* o = (DetectionTrackingSystem*)obj;
    long int frm1 = 0;
    while(1)
        o->get_from_file(frm1++);
}

void DetectionTrackingSystem::run ()
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

DetectionTrackingSystem::DetectionTrackingSystem() : display(640*2*1.4,480*1.4), det_comb(23,0),
                            base_camera(Globals::camPath_left.c_str(),true),
                            streaming(90,15,"iPad"),
                            odimg(640,480,1,3,0)
{
    init();
}

DetectionTrackingSystem::~DetectionTrackingSystem()
{
    delete odom;
    delete fovis_rect;
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
void DetectionTrackingSystem::ReadUpperBodyTemplate(Matrix<double>& upper_body_template)
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

void DetectionTrackingSystem::RenderBBox2D(const Vector<double>& bbox, CImg<unsigned char>& image, int r, int g, int b)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    const unsigned char color[] = {r,g,b};
    image.draw_rectangle_1(x,y,x+w,y+h,color,3);
}

void DetectionTrackingSystem::init()
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
    is_seg= Globals::use_segmentation_roi;

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

void DetectionTrackingSystem::SetTitle()
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

void DetectionTrackingSystem::check_keys()
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

void DetectionTrackingSystem::sink_frame(long int& frm, bool is_runing)
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
            ofstream imgf(pth, ios_base::binary);
            sprintf(pth,"rec/dp_%08d",frm);
            ofstream dpf(pth, ios_base::binary);
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

void DetectionTrackingSystem::get_from_file(long int frm)
{
    char pth[200];
    sprintf(pth,"%s/img_%08d", Globals::from_file_path.c_str(),frm);
    ifstream imgf(pth, ios_base::binary);
    sprintf(pth,"%s/dp_%08d", Globals::from_file_path.c_str(),frm);
    ifstream dpf(pth, ios_base::binary);
    if(!imgf.is_open() || !dpf.is_open())
        return;

    uint w = Globals::dImWidth, h = Globals::dImHeight;

    unsigned char* b_image = new unsigned char[w*h*3];
    imgf.read((char*)b_image, Globals::dImWidth* Globals::dImHeight*3);
    imgf.close();

    float* b_depth = new float[h*w];
    dpf.read((char*)b_depth, Globals::dImWidth* Globals::dImHeight*sizeof(float));
    dpf.close();

    main_process(b_image, b_depth, Globals::dImWidth, Globals::dImHeight);
}

void DetectionTrackingSystem::grabber_callback(const float *depth, const unsigned char *image)
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

void DetectionTrackingSystem::main_process(unsigned char* b_image, float* b_depth, uint w, uint h)
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

    if (true) {
        cout << "Depth map:\n";
        const int stride = 10;
        for(int i = 0; i < Globals::dImWidth; i+=stride)
        {
            for(int j = 0; j < Globals::dImHeight; j+=stride)
            {
                cout << depth_map(i,j) << ",";
            }
            cout <<"|"<< endl;
        }
        cout << endl;
    }

    ////////////////// FOVIS /////////////////////////////////////////////////////////////////
    ct= CPUTime();

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
    ct= CPUTime();
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
    ct= CPUTime();
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

DetectionTrackingSystem *DetectionTrackingSystem::getInstance() {
    if (_this==NULL)
        _this=new DetectionTrackingSystem();

    return  _this;
}
