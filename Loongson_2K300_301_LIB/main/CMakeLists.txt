cmake_minimum_required(VERSION 3.16)

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/toolchain_path.cmake")
endif()

if (DEFINED CMAKE_TOOLCHAIN_PATH AND EXISTS "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-gcc")
    set(CMAKE_C_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-gcc")
    set(CMAKE_CXX_COMPILER "${CMAKE_TOOLCHAIN_PATH}/bin/loongarch64-linux-gnu-g++")
else()
    find_program(LOONGSON_GCC loongarch64-linux-gnu-gcc)
    find_program(LOONGSON_GXX loongarch64-linux-gnu-g++)
    if (LOONGSON_GCC AND LOONGSON_GXX)
        set(CMAKE_C_COMPILER "${LOONGSON_GCC}")
        set(CMAKE_CXX_COMPILER "${LOONGSON_GXX}")
    endif()
endif()

project(main)

set(MAIN_SRC "${PROJECT_SOURCE_DIR}/main.cpp")
file(GLOB APP_SRC "${PROJECT_SOURCE_DIR}/../libraries/app/*.cpp")
file(GLOB APP_GYRO_SRC "${PROJECT_SOURCE_DIR}/../libraries/app/gyro/*.cpp")
file(GLOB APP_DISPLAY_SRC "${PROJECT_SOURCE_DIR}/../libraries/app/display/*.cpp")
file(GLOB COMMON_SRC "${PROJECT_SOURCE_DIR}/../libraries/common/src/*.cpp")
file(GLOB DRV_SRC "${PROJECT_SOURCE_DIR}/../libraries/drv/src/*.cpp")
file(GLOB EXAMPLE_SRC "${PROJECT_SOURCE_DIR}/../example/src/*.cpp")

set(USER_APP_SRC)
if (EXISTS "${PROJECT_SOURCE_DIR}/../user_app")
    file(GLOB USER_APP_SRC "${PROJECT_SOURCE_DIR}/../user_app/*.cpp")
endif()

find_package(PkgConfig QUIET)

option(LQ_ENABLE_OPENCV "Enable OpenCV-dependent modules" OFF)
option(LQ_ENABLE_NCNN "Enable NCNN-dependent modules" OFF)
set(LQ_OPENCV_ROOT "" CACHE PATH "Path to target OpenCV install root")
set(LQ_NCNN_ROOT "" CACHE PATH "Path to target NCNN install root")

set(LQ_HAVE_OPENCV FALSE)
set(LQ_HAVE_NCNN FALSE)
set(LQ_OPENCV_INCLUDE_DIRS)
set(LQ_OPENCV_LIBRARY_DIRS)
set(LQ_OPENCV_LINK_LIBS)
set(LQ_NCNN_INCLUDE_DIRS)
set(LQ_NCNN_LIBRARY_DIRS)
set(LQ_NCNN_LINK_LIBS)

if (LQ_ENABLE_OPENCV)
    if (LQ_OPENCV_ROOT AND EXISTS "${LQ_OPENCV_ROOT}/include/opencv4" AND EXISTS "${LQ_OPENCV_ROOT}/lib")
        set(LQ_HAVE_OPENCV TRUE)
        set(LQ_OPENCV_INCLUDE_DIRS "${LQ_OPENCV_ROOT}/include/opencv4")
        set(LQ_OPENCV_LIBRARY_DIRS "${LQ_OPENCV_ROOT}/lib")
        set(LQ_OPENCV_LINK_LIBS
            opencv_highgui
            opencv_ml
            opencv_objdetect
            opencv_photo
            opencv_stitching
            opencv_video
            opencv_calib3d
            opencv_features2d
            opencv_dnn
            opencv_flann
            opencv_videoio
            opencv_imgcodecs
            opencv_imgproc
            opencv_core
            dl
            m
            rt
        )
    elseif (PkgConfig_FOUND)
        pkg_check_modules(OPENCV QUIET opencv4)
        if (OPENCV_FOUND)
            set(LQ_HAVE_OPENCV TRUE)
            set(LQ_OPENCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIRS})
            set(LQ_OPENCV_LIBRARY_DIRS ${OPENCV_LIBRARY_DIRS})
            set(LQ_OPENCV_LINK_LIBS ${OPENCV_LINK_LIBRARIES})
        endif()
    endif()
endif()

if (LQ_ENABLE_NCNN AND LQ_HAVE_OPENCV)
    if (LQ_NCNN_ROOT AND EXISTS "${LQ_NCNN_ROOT}/include/ncnn" AND EXISTS "${LQ_NCNN_ROOT}/lib")
        set(LQ_HAVE_NCNN TRUE)
        set(LQ_NCNN_INCLUDE_DIRS "${LQ_NCNN_ROOT}/include")
        set(LQ_NCNN_LIBRARY_DIRS "${LQ_NCNN_ROOT}/lib")
        set(LQ_NCNN_LINK_LIBS ncnn)
    elseif (PkgConfig_FOUND)
        pkg_check_modules(NCNN QUIET ncnn)
        if (NCNN_FOUND)
            set(LQ_HAVE_NCNN TRUE)
            set(LQ_NCNN_INCLUDE_DIRS ${NCNN_INCLUDE_DIRS})
            set(LQ_NCNN_LIBRARY_DIRS ${NCNN_LIBRARY_DIRS})
            set(LQ_NCNN_LINK_LIBS ${NCNN_LINK_LIBRARIES})
        endif()
    endif()
endif()

if (LQ_HAVE_OPENCV)
    add_compile_definitions(LQ_HAVE_OPENCV)
else()
    list(REMOVE_ITEM EXAMPLE_SRC
        "${PROJECT_SOURCE_DIR}/../example/src/12_lq_udp_img_trans_demo.cpp"
        "${PROJECT_SOURCE_DIR}/../example/src/18_lq_ips20_show_img_demo.cpp"
    )
endif()

if (LQ_HAVE_OPENCV AND LQ_HAVE_NCNN)
    add_compile_definitions(LQ_HAVE_NCNN)
else()
    list(REMOVE_ITEM DRV_SRC "${PROJECT_SOURCE_DIR}/../libraries/drv/src/lq_ncnn.cpp")
    list(REMOVE_ITEM EXAMPLE_SRC "${PROJECT_SOURCE_DIR}/../example/src/07_lq_ncnn_demo.cpp")
endif()

set(SRC
    ${MAIN_SRC}
    ${APP_SRC}
    ${APP_GYRO_SRC}
    ${APP_DISPLAY_SRC}
    ${COMMON_SRC}
    ${DRV_SRC}
    ${EXAMPLE_SRC}
    ${USER_APP_SRC}
)

add_executable(${PROJECT_NAME} ${SRC})

include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/../libraries/app/
    ${PROJECT_SOURCE_DIR}/../libraries/app/gyro
    ${PROJECT_SOURCE_DIR}/../libraries/app/display
    ${PROJECT_SOURCE_DIR}/../libraries/common/inc
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc
    ${PROJECT_SOURCE_DIR}/../example/inc
)

if (EXISTS "${PROJECT_SOURCE_DIR}/../user_app")
    include_directories(${PROJECT_SOURCE_DIR}/../user_app)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/../libraries/drv/inc
)

if (LQ_HAVE_OPENCV)
    target_include_directories(${PROJECT_NAME} PRIVATE ${LQ_OPENCV_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PRIVATE ${LQ_OPENCV_LIBRARY_DIRS})
endif()

if (LQ_HAVE_OPENCV AND LQ_HAVE_NCNN)
    target_include_directories(${PROJECT_NAME} PRIVATE ${LQ_NCNN_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PRIVATE ${LQ_NCNN_LIBRARY_DIRS})
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

if (LQ_HAVE_OPENCV)
    if (EXISTS "${PROJECT_SOURCE_DIR}/../libraries/drv/src/liblq_camera_ex.a")
        target_link_libraries(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/../libraries/drv/src/liblq_camera_ex.a")
    endif()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LQ_OPENCV_LINK_LIBS})
endif()

if (LQ_HAVE_OPENCV AND LQ_HAVE_NCNN)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LQ_NCNN_LINK_LIBS})
endif()

message(STATUS "Loongson toolchain path: ${CMAKE_TOOLCHAIN_PATH}")
message(STATUS "Loongson sysroot path: ${CMAKE_SYSROOT}")
message(STATUS "OpenCV requested: ${LQ_ENABLE_OPENCV}")
message(STATUS "OpenCV root: ${LQ_OPENCV_ROOT}")
message(STATUS "OpenCV enabled: ${LQ_HAVE_OPENCV}")
message(STATUS "NCNN requested: ${LQ_ENABLE_NCNN}")
message(STATUS "NCNN root: ${LQ_NCNN_ROOT}")
message(STATUS "NCNN enabled: ${LQ_HAVE_NCNN}")
