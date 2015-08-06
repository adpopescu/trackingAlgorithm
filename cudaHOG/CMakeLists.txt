cmake_minimum_required( VERSION 2.6 )
set ( CMAKE_BUILD_TYPE Debug )
add_definitions ( -Wall )
find_package ( Qt4 REQUIRED )
find_package(CUDA QUIET REQUIRED)
include ( ${QT_USE_FILE} )
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
	$(CUDA_INC_PATH))
set ( cudaHOG_HDRS
	cudaHOG.h
	gradients.h
	padding.h
	conversions.h
	blocks.h
	global.h
	detections.h
	parameters.h
	)

set ( cudaHOG_SRCS
	cudaHOG.cpp
	parameters.cpp
	)
set ( cudaHOG_CU_SRCS
    gradients.cu
    padding.cu
    conversions.cu
    blocks.cu
    hog.cu
    descriptor.cu
    svm.cu
    timer.cu
    detections.cu
    nms.cu )

cuda_add_library ( cudaHOG SHARED ${cudaHOG_SRCS} ${cudaHOG_CU_SRCS} ${UIS} ${RSCS} ${TRS} ${MOCS} )
target_link_libraries ( cudaHOG  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} )
