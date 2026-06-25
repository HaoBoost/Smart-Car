#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
TARGET_HOST="root@192.168.179.33"
#  TARGET_HOST="root@10.96.193.33"
TARGET_PATH="/root"

echo "==> 编译中..."
# 缓存路径不匹配时自动清理
if [ -f "${BUILD_DIR}/CMakeCache.txt" ]; then
    CACHED_DIR=$(grep "CMAKE_HOME_DIRECTORY" "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
    if [ "${CACHED_DIR}" != "${PROJECT_DIR}" ]; then
        echo "==> 检测到旧缓存，清理 build 目录..."
        rm -rf "${BUILD_DIR}"
    fi
fi
cmake -B "${BUILD_DIR}" -S "${PROJECT_DIR}" -DENABLE_NCNN=ON
make -C "${BUILD_DIR}" -j$(nproc)

echo "==> 编译成功，传输文件..."
scp "${BUILD_DIR}/main" "${TARGET_HOST}:${TARGET_PATH}"
scp "${PROJECT_DIR}/tiny_classifier_fp32.ncnn.param" "${TARGET_HOST}:${TARGET_PATH}"
scp "${PROJECT_DIR}/tiny_classifier_fp32.ncnn.bin" "${TARGET_HOST}:${TARGET_PATH}"
scp "${PROJECT_DIR}/labels.txt" "${TARGET_HOST}:${TARGET_PATH}"

echo "==> 完成！"
