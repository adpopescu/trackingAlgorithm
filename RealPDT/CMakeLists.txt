project( RealPDT )
cmake_minimum_required( VERSION 2.6 )
add_definitions ( -Wall )
find_package ( Qt4 REQUIRED )
find_package (Threads)
find_package(ImageMagick COMPONENTS Magick++)
find_library(xlib X11)
include ( ${QT_USE_FILE} )
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
	/usr/include/eigen3/
	/usr/include/boost/
	/usr/lib/
	/home/mmp/Projects/europa/code/oxford/MOOS/Core/
	/home/mmp/Projects/europa/code/oxford/MrgCore/
	/usr/include/ImageMagick/
	../OpenNI-Linux-x64-2.2/Include
	../cudaHOG
	../RealPDT)
set ( EXECUTABLE_OUTPUT_PATH bin )
set ( RealPDT_HDRS
	AncillaryMethods.h
	Globals.h
	Vector.h
	Matrix.h
	ROI.h
	KConnectedComponentLabeler.h
	detectiontrackingsystem.h
	CPoint.h
	camera/Camera.h
	config_file/ConfigFile.h
	CImg/CImg.h
	Detections.h
	math/Math.h
	Volume.h
	Tracker.h
	Hypo.h
	FrameInlier.h
	Visualization.h
	MDL.h
	EKalman.h
	Kalman.h
	main.h
	pointcloud.h
	depthdetector.h
	groundplaneestimator.h
	detector.h
	segmentation_roi/SegmentedObj.h
	segmentation_roi/SegmentationROI.h
	segmentation_roi/ROI_SEG.h
	segmentation_roi/CFilter.h
	detector_seg.h
	fovis/visual_odometry.hpp
	fovis/tictoc.hpp
	fovis/stereo_rectify.hpp
	fovis/stereo_frame.hpp
	fovis/stereo_depth.hpp
	fovis/sad.hpp
	fovis/refine_motion_estimate.hpp
	fovis/refine_feature_match.hpp
	fovis/rectification.hpp
	fovis/pyramid_level.hpp
	fovis/primesense_depth.hpp
	fovis/options.hpp
	fovis/normalize_image.hpp
	fovis/motion_estimation.hpp
	fovis/keypoint.hpp
	fovis/internal_utils.hpp
	fovis/intensity_descriptor.hpp
	fovis/initial_homography_estimation.hpp
	fovis/grid_filter.hpp
	fovis/gauss_pyramid.h
	fovis/frame.hpp
	fovis/fovis.hpp
	fovis/feature_matcher.hpp
	fovis/feature_match.hpp
	fovis/fast.hpp
	fovis/depth_source.hpp
	fovis/depth_image.hpp
	fovis/camera_intrinsics.hpp
	fovis/absolute_orientation_horn.hpp
	depthdetector_lm.h
	depthdetector_seg.h
	depthdetector_lm_seg.h
	Streaming/StreamingApp.h
	hogdetector.h
	Grabber.h
	)

set ( RealPDT_SRCS
	main.cpp
	AncillaryMethods.cpp
	Globals.cpp
	ROI.cpp
	KConnectedComponentLabeler.cpp
	CPoint.cpp
	camera/Camera.cpp
	config_file/ConfigFile.cpp
	Detections.cpp
	math/Math.cpp
	Tracker.cpp
	Hypo.cpp
	FrameInlier.cpp
	Visualization.cpp
	MDL.cpp
	EKalman.cpp
	Kalman.cpp
	pointcloud.cpp
	depthdetector.cpp
	groundplaneestimator.cpp
	detectiontrackingsystem.cpp
	detector.cpp
	segmentation_roi/SegmentedObj.cpp
	segmentation_roi/SegmentationROI.cpp
	segmentation_roi/ROI_SEG.cpp
	detector_seg.cpp
	fovis/visual_odometry.cpp
	fovis/tictoc.cpp
	fovis/stereo_rectify.cpp
	fovis/stereo_frame.cpp
	fovis/stereo_depth.cpp
	fovis/refine_motion_estimate.cpp
	fovis/refine_feature_match.cpp
	fovis/rectification.cpp
	fovis/pyramid_level.cpp
	fovis/primesense_depth.cpp
	fovis/normalize_image.cpp
	fovis/motion_estimation.cpp
	fovis/internal_utils.cpp
	fovis/intensity_descriptor.cpp
	fovis/initial_homography_estimation.cpp
	fovis/grid_filter.cpp
	fovis/gauss_pyramid.c
	fovis/frame.cpp
	fovis/feature_matcher.cpp
	fovis/fast.cpp
	fovis/depth_image.cpp
	depthdetector_lm.cpp
	depthdetector_seg.cpp
	depthdetector_lm_seg.cpp
	Streaming/StreamingApp.cpp
	hogdetector.cpp
	Grabber.cpp
	)

if(!ImageMagick_FOUND)
    message( FATAL_ERROR "Imagemagick not found!" )
endif()
add_executable ( RealPDT ${RealPDT_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} )
target_link_libraries ( RealPDT cudaHOG ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${xlib} ${ImageMagick_LIBRARIES} ${OPENNI2_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
