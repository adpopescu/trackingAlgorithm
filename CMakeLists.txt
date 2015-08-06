cmake_minimum_required( VERSION 2.6 )
include(cmake/FindOpenNI2.cmake)
add_definitions ( -Wall )
find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
	)
add_subdirectory( cudaHOG )
add_subdirectory( RealPDT )
