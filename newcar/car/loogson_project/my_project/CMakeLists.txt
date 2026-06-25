# cmake_minimum_required(VERSION 3.10)

# project(SMARTCAR)

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# #此处可修改交叉编译工具路径
# set(CROSSTOOL_DIR "/opt/loongson-gnu-toolchain-13.2")
# set(CMAKE_C_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-g++)

# set(CMAKE_SYSTEM_NAME Linux)
# set(CMAKE_SYSTEM_PROCESSOR loongarch64)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# include_directories(${PROJECT_SOURCE_DIR}/smartCar/code)
# include_directories(${PROJECT_SOURCE_DIR}/smartCar/task)
# include_directories(${PROJECT_SOURCE_DIR}/smartCar/include)
# include_directories(${PROJECT_SOURCE_DIR}/smartCar/wuwu_library)

# #编译的库已经放在工程里啦
# include_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/include/opencv4)
# include_directories(${PROJECT_SOURCE_DIR}/cross_lib/json/include)
# include_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/include/ncnn)

# file(GLOB_RECURSE MYSRC
#     ${PROJECT_SOURCE_DIR}/smartCar/*.cc
#     ${PROJECT_SOURCE_DIR}/smartCar/*.cpp  
#     ${PROJECT_SOURCE_DIR}/smartCar/*.c
# )

# link_directories(${CROSSTOOL_DIR}/loongarch64-unknown-linux-gnu/lib)
# link_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/lib)
# link_directories(${PROJECT_SOURCE_DIR}/cross_lib/json/lib)
# link_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/lib)

# add_executable(main ${MYSRC})
# target_link_libraries(main PRIVATE 
#                         opencv_core 
#                         opencv_imgcodecs
#                         opencv_imgproc
#                         opencv_videoio
#                         jsoncpp
#                         ncnn
#                         gomp)



cmake_minimum_required(VERSION 3.10)

project(SMARTCAR)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#此处可修改交叉编译工具路径
set(CROSSTOOL_DIR "/opt/loongson-gnu-toolchain-13.2")
set(CMAKE_C_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${CROSSTOOL_DIR}/bin/loongarch64-unknown-linux-gnu-g++)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

option(ENABLE_NCNN "Link ncnn into main" ON)
option(NCNN_WITH_OPENMP "Link libgomp for ncnn builds compiled with OpenMP" ON)

include_directories(${PROJECT_SOURCE_DIR}/smartCar/code)
include_directories(${PROJECT_SOURCE_DIR}/smartCar/include)
include_directories(${PROJECT_SOURCE_DIR}/smartCar/wuwu_library)

#编译的库已经放在工程里啦
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/include/opencv4)
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/json/include)
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/include/ncnn)
include_directories(${PROJECT_SOURCE_DIR}/cross_lib/lq_camera/include)

file(GLOB_RECURSE MYSRC CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/smartCar/*.cc
    ${PROJECT_SOURCE_DIR}/smartCar/*.cpp  
    ${PROJECT_SOURCE_DIR}/smartCar/*.c
)

link_directories(${CROSSTOOL_DIR}/loongarch64-unknown-linux-gnu/lib)
link_directories(${PROJECT_SOURCE_DIR}/cross_lib/opencv/lib)
link_directories(${PROJECT_SOURCE_DIR}/cross_lib/json/lib)
link_directories(${PROJECT_SOURCE_DIR}/cross_lib/lq_camera/lib)

if(ENABLE_NCNN)
    link_directories(${PROJECT_SOURCE_DIR}/cross_lib/ncnn/lib)
endif()

add_executable(main ${MYSRC})
target_link_libraries(main PRIVATE 
                        opencv_core 
                        opencv_imgcodecs
                        opencv_imgproc
                        opencv_videoio
                        jsoncpp)

if(ENABLE_NCNN)
    target_link_libraries(main PRIVATE ncnn)
    if(NCNN_WITH_OPENMP)
        target_link_libraries(main PRIVATE gomp)
    endif()
endif()

#龙邱摄像头库
target_link_libraries(main PRIVATE lq_camera_ex)
