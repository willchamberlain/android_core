# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

include_directories(/mnt/nixbig/downloads/opencv_android_sdk/OpenCV-android-sdk_3_1_0/sdk/native/jni/include)
include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/)
include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/AprilTags/)
include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/)
include_directories(/mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/)
include_directories(/mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/common/)
include_directories(/mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/example/)
# apriltags_kaess is too slow, so try umich AprilTags


add_library( lib_opencv  SHARED IMPORTED )
# add_library( lib_eigen SHARED IMPORTED )
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION /mnt/nixbig/build_workspaces/androidapriltagsros/app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)
# set_target_properties(lib_eigen PROPERTIES IMPORTED_LOCATION /Users/will/projects/OpencvTestAndroidStudio/app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)
# find_package(Eigen3 REQUIRED)
include_directories(/usr/include/eigen3/)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.

add_library(
             apriltags_kaess                # Sets the name of the library.
             SHARED                         # Sets the library as a shared library.
                                            # Provides a relative path to your source file(s).  Associated headers in the same location as their source file are automatically included.  #  SEE  https://github.com/RIVeR-Lab/apriltags_ros/blob/indigo-devel/apriltags/CMakeLists.txt
             /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Edge.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/FloatImage.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GLine2D.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GLineSegment2D.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Gaussian.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GrayModel.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Homography33.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/MathUtil.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Quad.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Segment.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagDetection.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagDetector.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagFamily.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/UnionFindSimple.cc
             src/main/cpp/native-apriltags.cpp  # JNI calls - core of this library
            )

add_library(
             native-lib                     # Sets the name of the library.
             SHARED                         # Sets the library as a shared library.
                                            # Provides a relative path to your source file(s).  Associated headers in the same location as their source file are automatically included.
             src/main/cpp/native-lib.cpp    # JNI calls - core of this library
             )

set(apriltags_umich_dir "/mnt/nixbig/downloads/apriltags_umich/apriltag-2016-12-01__0_9_8/apriltag-2016-12-01/" )
add_library(
            apriltags_umich                 # Sets the name of the library.
            SHARED                          # Sets the library as a shared library.
            ${apriltags_umich_dir}/apriltag.c ${apriltags_umich_dir}/apriltag_quad_thresh.c ${apriltags_umich_dir}/tag16h5.c ${apriltags_umich_dir}/tag25h7.c ${apriltags_umich_dir}/tag25h9.c ${apriltags_umich_dir}/tag36artoolkit.c ${apriltags_umich_dir}/tag36h10.c ${apriltags_umich_dir}/tag36h11.c ${apriltags_umich_dir}/v4l2uvc.c
            ${apriltags_umich_dir}/apriltag.h ${apriltags_umich_dir}/apriltag_math.h ${apriltags_umich_dir}/tag16h5.h ${apriltags_umich_dir}/tag25h7.h ${apriltags_umich_dir}/tag25h9.h ${apriltags_umich_dir}/tag36artoolkit.h ${apriltags_umich_dir}/tag36h10.h ${apriltags_umich_dir}/tag36h11.h ${apriltags_umich_dir}/v4l2uvc.h
            ${apriltags_umich_dir}/common/doubles_floats_impl.h ${apriltags_umich_dir}/common/g2d.h ${apriltags_umich_dir}/common/image_f32.h ${apriltags_umich_dir}/common/image_u8x3.h ${apriltags_umich_dir}/common/math_util.h ${apriltags_umich_dir}/common/pnm.h ${apriltags_umich_dir}/common/svd22.h ${apriltags_umich_dir}/common/time_util.h ${apriltags_umich_dir}/common/zarray.h
            ${apriltags_umich_dir}/common/doubles.h ${apriltags_umich_dir}/common/getopt.h ${apriltags_umich_dir}/common/image_types.h ${apriltags_umich_dir}/common/image_u8x4.h ${apriltags_umich_dir}/common/pam.h ${apriltags_umich_dir}/common/postscript_utils.h ${apriltags_umich_dir}/common/thash_impl.h ${apriltags_umich_dir}/common/unionfind.h ${apriltags_umich_dir}/common/zhash.h
            ${apriltags_umich_dir}/common/floats.h ${apriltags_umich_dir}/common/homography.h ${apriltags_umich_dir}/common/image_u8.h ${apriltags_umich_dir}/common/matd.h ${apriltags_umich_dir}/common/pjpeg.h ${apriltags_umich_dir}/common/string_util.h ${apriltags_umich_dir}/common/timeprofile.h ${apriltags_umich_dir}/common/workerpool.h ${apriltags_umich_dir}/common/zmaxheap.h
            ${apriltags_umich_dir}/common/g2d.c ${apriltags_umich_dir}/common/homography.c ${apriltags_umich_dir}/common/image_u8.c ${apriltags_umich_dir}/common/image_u8x4.c ${apriltags_umich_dir}/common/pam.c ${apriltags_umich_dir}/common/pjpeg-idct.c ${apriltags_umich_dir}/common/string_util.c ${apriltags_umich_dir}/common/time_util.c ${apriltags_umich_dir}/common/workerpool.c ${apriltags_umich_dir}/common/zhash.c
            ${apriltags_umich_dir}/common/getopt.c ${apriltags_umich_dir}/common/image_f32.c ${apriltags_umich_dir}/common/image_u8x3.c ${apriltags_umich_dir}/common/matd.c ${apriltags_umich_dir}/common/pjpeg.c ${apriltags_umich_dir}/common/pnm.c ${apriltags_umich_dir}/common/svd22.c ${apriltags_umich_dir}/common/unionfind.c ${apriltags_umich_dir}/common/zarray.c ${apriltags_umich_dir}/common/zmaxheap.c
            ${apriltags_umich_dir}/example/apriltag_v4l_integration_demo.c
            src/main/cpp/native-apriltags-umich.cpp    # JNI calls - core of this library
            src/main/cpp/native-apriltags-umich-oneshot.cpp    # JNI calls - core of this library
           )

# set(apriltags_swatbotics_dir "/mnt/nixbig/downloads/apriltags_swatbotics/apriltags-cpp/src/" )
# add_library(
#             apriltags_swatbotics            # Sets the name of the library.
#             SHARED                          # Sets the library as a shared library.
#             ${apriltags_swatbotics_dir}/AprilTypes.h ${apriltags_swatbotics_dir}/CameraUtil.cpp ${apriltags_swatbotics_dir}/CameraUtil.h ${apriltags_swatbotics_dir}/camtest.cpp ${apriltags_swatbotics_dir}/CMakeLists.txt ${apriltags_swatbotics_dir}/DebugImage.cpp ${apriltags_swatbotics_dir}/DebugImage.h ${apriltags_swatbotics_dir}/Geometry.cpp ${apriltags_swatbotics_dir}/Geometry.h ${apriltags_swatbotics_dir}/gltest.cpp ${apriltags_swatbotics_dir}/GrayModel.cpp ${apriltags_swatbotics_dir}/GrayModel.h ${apriltags_swatbotics_dir}/maketags.cpp ${apriltags_swatbotics_dir}/MathUtil.cpp ${apriltags_swatbotics_dir}/MathUtil.h ${apriltags_swatbotics_dir}/Profiler.h ${apriltags_swatbotics_dir}/quadtest.cpp ${apriltags_swatbotics_dir}/Refine.cpp ${apriltags_swatbotics_dir}/Refine.h ${apriltags_swatbotics_dir}/TagDetection.h ${apriltags_swatbotics_dir}/TagDetector.cpp ${apriltags_swatbotics_dir}/TagDetector.h ${apriltags_swatbotics_dir}/TagFamilies.cpp ${apriltags_swatbotics_dir}/TagFamily.cpp ${apriltags_swatbotics_dir}/TagFamily.h ${apriltags_swatbotics_dir}/tagtest.cpp ${apriltags_swatbotics_dir}/UnionFindSimple.cpp ${apriltags_swatbotics_dir}/UnionFindSimple.h
#             )


# Searches for a specified prebuilt library and stores the path as a
# variable. Because system libraries are included in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library(
              log_library                   # Sets the name of the path variable to set with the library path.
              log                           # Specifies the name of the NDK library that you want CMake to locate.
             )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in the
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries(
                       apriltags_kaess      # Specifies the target library.
                       ${log_library}       # Links the target library to the log library included in the NDK.
                       lib_opencv           # Links the target library to OpenCV
                            ### lib_eigen           # Links the target library to Eigen
                       )

target_link_libraries(
                       native-lib           # Specifies the target library.
                       ${log_library}       # Links the target library to the log library included in the NDK.
                       lib_opencv           # Links the target library to OpenCV
                       )

target_link_libraries(
                       apriltags_umich      # Specifies the target library.
                       ${log_library}       # Links the target library to the log library included in the NDK.
                       )

# target_link_libraries(
#                        apriltags_swatbotics # Specifies the target library.
#                        ${log_library}       # Links the target library to the log library included in the NDK.
#                        lib_opencv           # Links the target library to OpenCV
#                        )