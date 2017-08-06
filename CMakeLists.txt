##### CMake entry point

cmake_minimum_required (VERSION 2.6)
project (mld_thesis)



##### use external cmake codes

if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( FATAL_ERROR "Please select another Build Directory ! (and give it a clever name, like bin_Visual2012_64bits/)" )
endif()
if( CMAKE_SOURCE_DIR MATCHES " " )
  message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()
if( CMAKE_BINARY_DIR MATCHES " " )
  message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()



##### make release version

set(CMAKE_BUILD_TYPE Release)



##### external library setting

# OpenCV
set( OPENCV_VER       310 )
set( OPENCV_PATH "C:/programming/lib/opencv-master/build-test/install")
set( OPENCV_INC_DIR "${OPENCV_PATH}/include" )
set( OPENCV_LIB_DIR "${OPENCV_PATH}/x64/vc14/lib" )
set( OPENCV_LIB   
	optimized opencv_world${OPENCV_VER}      debug opencv_world${OPENCV_VER}d
	optimized opencv_xfeatures2d${OPENCV_VER}   debug opencv_xfeatures2d${OPENCV_VER}d
)


# JSONCPP
set( JSONCPP_PATH "C:/programming/lib/jsoncpp-master")
set( JSONCPP_INC_DIR "${JSONCPP_PATH}/include" )
set( JSONCPP_LIB_DIR "${JSONCPP_PATH}/bld/src/lib_json/Release" )
set( JSONCPP_LIB   
	optimized jsoncpp
)



########################## NO CHANGES BEYOND THIS POINT ##########################


##### include & link
  
# main project 
include_directories(
  ${OPENCV_INC_DIR}
  ${JSONCPP_INC_DIR}
)
link_directories(
  ${OPENCV_LIB_DIR}
  ${JSONCPP_LIB_DIR}
)



##### project

FILE(GLOB MLD_CRF_SRC_FILES "src/*.cpp" "src/*.c" "src/*.h")



##### build demo program

add_executable(mld_thesis 
  ${MLD_CRF_SRC_FILES}
)
target_link_libraries(mld_thesis
  ${OPENCV_LIB}
  ${JSONCPP_LIB}
)

