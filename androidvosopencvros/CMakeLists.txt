# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

include_directories(/mnt/nixbig/downloads/opencv_android_sdk/OpenCV-android-sdk_3_1_0/sdk/native/jni/include)
include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/)
include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/AprilTags/)

include_directories(/mnt/nixbig/downloads/apriltags_kaess/apriltags/)

add_library( lib_opencv SHARED IMPORTED )
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
             apriltags_kaess    # Sets the name of the library.
             SHARED             # Sets the library as a shared library.

             # Provides a relative path to your source file(s).  Associated headers in the same location as their source file are automatically included.
             #  SEE  https://github.com/RIVeR-Lab/apriltags_ros/blob/indigo-devel/apriltags/CMakeLists.txt
             /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Edge.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/FloatImage.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GLine2D.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GLineSegment2D.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Gaussian.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/GrayModel.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Homography33.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/MathUtil.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Quad.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/Segment.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagDetection.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagDetector.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/TagFamily.cc /mnt/nixbig/downloads/apriltags_kaess/apriltags/src/UnionFindSimple.cc
             src/main/cpp/native-apriltags.cpp
            )

add_library(
             native-lib     # Sets the name of the library.
             SHARED         # Sets the library as a shared library.
             src/main/cpp/native-lib.cpp  # Provides a relative path to your source file(s).  Associated headers in the same location as their source file are automatically included.
             )


# Searches for a specified prebuilt library and stores the path as a
# variable. Because system libraries are included in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library(
              log-lib   # Sets the name of the path variable to set with the library path.
              log       # Specifies the name of the NDK library that you want CMake to locate.
             )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in the
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries(
                       apriltags_kaess      # Specifies the target library.
                       ${log-lib}           # Links the target library to the log library included in the NDK.

                       lib_opencv           # OpenCV

                       #lib_eigen           # Eigen
                       )




target_link_libraries(
                       native-lib           # Specifies the target library.

                       ${log-lib}           # Links the target library to the log library included in the NDK.

                       lib_opencv           # OpenCV
                       )