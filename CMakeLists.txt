cmake_minimum_required( VERSION 2.8 )
project( gps_orb )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

include_directories( "/usr/include/eigen3" )

# OpenCV
# #set(OpenCV_DIR /home/feng/3rdparty/opencv3.4.3/share/OpenCV)
# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_DIRS} )


include_directories(${PROJECT_SOURCE_DIR}/include)


add_executable( gps main.cpp )

target_link_libraries( gps
    ${OpenCV_LIBS}
    # ${PROJECT_NAME}
    ${G2O_TYPES_DATA}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM3D_ADDONS}
)
