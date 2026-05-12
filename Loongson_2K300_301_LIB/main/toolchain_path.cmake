set(_lq_toolchain_candidates
    "$ENV{LQ_TOOLCHAIN_PATH}"
    "/home/china/Loongson_2k300_Library4.10(comment out)/Loongson_2k300_Library4.10/Loongson_2k300_301_Library/Loongson_2K300_301_LIB/tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
    "/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1"
    "${CMAKE_CURRENT_LIST_DIR}/../tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
    "${CMAKE_CURRENT_LIST_DIR}/../tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.4"
)

if(NOT DEFINED CMAKE_TOOLCHAIN_PATH OR CMAKE_TOOLCHAIN_PATH STREQUAL "")
    foreach(_candidate IN LISTS _lq_toolchain_candidates)
        if(_candidate AND EXISTS "${_candidate}/bin/loongarch64-linux-gnu-gcc")
            set(CMAKE_TOOLCHAIN_PATH "${_candidate}" CACHE PATH "Loongson toolchain path" FORCE)
            break()
        endif()
    endforeach()
endif()

if((NOT DEFINED CMAKE_TOOLCHAIN_PATH OR CMAKE_TOOLCHAIN_PATH STREQUAL "") AND NOT DEFINED _lq_found_gcc)
    find_program(_lq_found_gcc loongarch64-linux-gnu-gcc)
    if(_lq_found_gcc)
        get_filename_component(_lq_toolchain_bin "${_lq_found_gcc}" DIRECTORY)
        get_filename_component(_lq_toolchain_root "${_lq_toolchain_bin}" DIRECTORY)
        set(CMAKE_TOOLCHAIN_PATH "${_lq_toolchain_root}" CACHE PATH "Loongson toolchain path" FORCE)
    endif()
endif()

if(DEFINED CMAKE_TOOLCHAIN_PATH AND NOT CMAKE_TOOLCHAIN_PATH STREQUAL "")
    set(CMAKE_TOOLCHAIN_PATH "${CMAKE_TOOLCHAIN_PATH}" CACHE PATH "Loongson toolchain path")
endif()

if(DEFINED CMAKE_TOOLCHAIN_PATH AND EXISTS "${CMAKE_TOOLCHAIN_PATH}/loongarch64-linux-gnu/sysroot")
    set(CMAKE_SYSROOT "${CMAKE_TOOLCHAIN_PATH}/loongarch64-linux-gnu/sysroot" CACHE PATH "Loongson sysroot path" FORCE)
endif()

if(DEFINED CMAKE_SYSROOT AND EXISTS "${CMAKE_SYSROOT}")
    set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT};${CMAKE_TOOLCHAIN_PATH}" CACHE STRING "Loongson root paths" FORCE)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER CACHE STRING "Program search mode" FORCE)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY CACHE STRING "Library search mode" FORCE)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY CACHE STRING "Include search mode" FORCE)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY CACHE STRING "Package search mode" FORCE)
endif()
