#!/bin/bash
# ============================================================
#  NCNN 模型测试 — 本机安装 + 编译 + 运行
#  前提: 电脑有摄像头 (USB内置均可)
# ============================================================
set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
err()   { echo -e "${RED}[ERR]${NC}  $*"; }

# ---------- Step 1: 系统依赖 ----------
info "Step 1/4: 安装系统包 (需要 sudo 密码)"
sudo apt-get update -qq
sudo apt-get install -y -qq \
    build-essential cmake git wget pkg-config \
    libopencv-dev v4l-utils

# ---------- Step 2: 编译 NCNN ----------
info "Step 2/4: 下载并编译 NCNN (约2分钟)"
NCNN_DIR="$HOME/ncnn_local"
if [ ! -d "$NCNN_DIR" ]; then
    git clone --depth=1 https://github.com/Tencent/ncnn.git "$NCNN_DIR"
fi
cd "$NCNN_DIR"
mkdir -p build install && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$NCNN_DIR/install" \
      -DNCNN_VULKAN=OFF \
      -DNCNN_BUILD_EXAMPLES=OFF \
      -DNCNN_BUILD_TOOLS=OFF \
      -DNCNN_BUILD_BENCHMARK=OFF \
      ..
make -j$(nproc) install
info "NCNN 安装到 $NCNN_DIR/install"

# ---------- Step 3: 准备模型 ----------
info "Step 3/4: 复制模型文件"
TEST_DIR="/home/china/car/native_test"
mkdir -p "$TEST_DIR"
cp /home/china/car/loogson_project/my_project/tiny_classifier_fp32.ncnn.param "$TEST_DIR/"
cp /home/china/car/loogson_project/my_project/tiny_classifier_fp32.ncnn.bin   "$TEST_DIR/"
cp /home/china/car/loogson_project/my_project/labels.txt                       "$TEST_DIR/"
info "模型文件 -> $TEST_DIR"

# ---------- Step 4: 编译测试程序 ----------
info "Step 4/4: 编译测试程序"
cat > "$TEST_DIR/test_camera.cpp" << 'CPPEOF'
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cstdio>

static const char* kModelParam = "tiny_classifier_fp32.ncnn.param";
static const char* kModelBin   = "tiny_classifier_fp32.ncnn.bin";
static const char* kLabelsFile = "labels.txt";
static const int   kInputW = 40, kInputH = 40;
static const float kMean[3] = { 123.675f, 116.28f, 103.53f };
static const float kNorm[3] = { 0.01712475f, 0.017507f, 0.01742919f };
static const int   kRedHueLow1 = 0,   kRedHueHigh1 = 10;
static const int   kRedHueLow2 = 156, kRedHueHigh2 = 180;
static const int   kRedSatMin = 60,  kRedValMin = 50;
static const int   kMinRedArea = 100;
static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running = false; }

static bool load_labels(const std::string& path, std::vector<std::string>& labels) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    labels.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) labels.push_back(line);
    }
    return !labels.empty();
}

static bool detect_red_roi(const cv::Mat& bgr, cv::Rect& out_roi) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(kRedHueLow1, kRedSatMin, kRedValMin),
                     cv::Scalar(kRedHueHigh1, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(kRedHueLow2, kRedSatMin, kRedValMin),
                     cv::Scalar(kRedHueHigh2, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    int best = 0;
    double best_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) { best_area = a; best = i; }
    }
    if (best_area < kMinRedArea) return false;
    out_roi = cv::boundingRect(contours[best]);
    int s = std::max(out_roi.width, out_roi.height);
    int cx = out_roi.x + out_roi.width / 2;
    int cy = out_roi.y + out_roi.height / 2;
    out_roi.x = std::max(0, cx - s / 2);
    out_roi.y = std::max(0, cy - s / 2);
    out_roi.width  = std::min(s, bgr.cols - out_roi.x);
    out_roi.height = std::min(s, bgr.rows - out_roi.y);
    return out_roi.width >= 10 && out_roi.height >= 10;
}

static bool classify_roi(ncnn::Net& net, const cv::Mat& roi,
                         std::vector<float>& probs, int& top, float& top_s) {
    cv::Mat input;
    cv::resize(roi, input, cv::Size(kInputW, kInputH));
    ncnn::Mat in = ncnn::Mat::from_pixels(input.data, ncnn::Mat::PIXEL_BGR,
                                          input.cols, input.rows);
    in.substract_mean_normalize(kMean, kNorm);
    ncnn::Extractor ex = net.create_extractor();
    ex.input("input", in);
    ncnn::Mat out;
    if (ex.extract("output", out) != 0) return false;
    int n = out.w;
    probs.resize(n);
    top = 0; top_s = -1.0f;
    float sum = 0;
    for (int i = 0; i < n; i++) { probs[i] = std::exp(out[i]); sum += probs[i]; }
    for (int i = 0; i < n; i++) {
        probs[i] /= sum;
        if (probs[i] > top_s) { top_s = probs[i]; top = i; }
    }
    return true;
}

int main() {
    signal(SIGINT, sig_handler);
    printf("========================================\n");
    printf("  NCNN 模型测试 (本机摄像头)\n");
    printf("========================================\n\n");

    ncnn::Net net;
    net.opt.use_vulkan_compute = false;
    net.opt.num_threads = 2;
    printf("[1/3] 加载模型...\n");
    if (net.load_param(kModelParam) != 0) { fprintf(stderr,"param FAIL\n"); return -1; }
    if (net.load_model(kModelBin) != 0)   { fprintf(stderr,"bin FAIL\n"); return -1; }
    std::vector<std::string> labels;
    if (!load_labels(kLabelsFile, labels)) { fprintf(stderr,"labels FAIL\n"); return -1; }
    printf("  %zu 类别: [", labels.size());
    for (size_t i=0;i<labels.size();i++) printf("%s%s",labels[i].c_str(),i+1<labels.size()?", ":"");
    printf("]\n");

    printf("[2/3] 打开摄像头...\n");
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        fprintf(stderr,"摄像头失败!\n");
        fprintf(stderr,"试试: cap(1), cap(2)... 或检查 /dev/video*\n");
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("  分辨率 %dx%d OK\n", w, h);

    printf("\n[3/3] 运行中\n");
    printf("  [Space/S] = 对当前画面分类\n");
    printf("  [Q/Esc]   = 退出并显示统计\n");
    printf("----------------------------------------\n");

    cv::Mat frame;
    int total=0, detected=0, classified=0;
    std::vector<int> cnt(labels.size(),0);
    std::vector<double> conf_sum(labels.size(),0.0);

    while (g_running) {
        cap >> frame;
        if (frame.empty()) { cv::waitKey(10); continue; }
        total++;

        // 找红色 ROI 并画框提示
        cv::Rect red_roi;
        bool has_red = detect_red_roi(frame, red_roi);
        if (has_red) {
            cv::rectangle(frame, red_roi, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "RED FOUND - press S", cv::Point(red_roi.x, red_roi.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

        // 状态条
        char buf[256];
        snprintf(buf,sizeof(buf),"Frames:%d  Classified:%d  %s  [S]=classify [Q]=quit",
                total, classified, has_red ? "** RED **" : "no red");
        cv::putText(frame,buf,cv::Point(5,frame.rows-10),
                   cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(200,200,200),1);
        cv::imshow("NCNN Classifier Test - press S to classify", frame);

        int key = cv::waitKey(10) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 's' || key == ' ') {
            if (!has_red) {
                printf("[#%d] 未检测到红色, 跳过\n", total);
                fflush(stdout);
                continue;
            }
            detected++;
            std::vector<float> probs;
            int top; float top_s;
            if (!classify_roi(net, frame(red_roi), probs, top, top_s)) {
                printf("[#%d] 推理失败\n", total); fflush(stdout); continue;
            }
            classified++;
            cnt[top]++; conf_sum[top]+=top_s;
            printf("[#%d] ROI=(%d,%d %dx%d) -> %s (%.1f%%)",
                   total, red_roi.x, red_roi.y, red_roi.width, red_roi.height,
                   top<(int)labels.size()?labels[top].c_str():"?", top_s*100);
            for(size_t i=0;i<probs.size();i++)
                printf(" | %s:%.0f%%",labels[i].c_str(),probs[i]*100);
            printf("\n"); fflush(stdout);
        }
    }

    cv::destroyAllWindows();
    printf("\n========================================\n");
    printf("  统计: 总%d帧  检出红%d  分类%d\n",total,detected,classified);
    for(size_t i=0;i<labels.size();i++)
        printf("  %-12s: %d次 (%.1f%%)  均置%.1f%%\n",
               labels[i].c_str(),cnt[i],
               classified?100.0*cnt[i]/classified:0,
               cnt[i]?100.0*conf_sum[i]/cnt[i]:0.0);
    printf("========================================\n");
    return 0;
}
CPPEOF

cd "$TEST_DIR"
g++ -std=c++17 -O2 -o test_camera test_camera.cpp \
    $(pkg-config --cflags --libs opencv4) \
    -I"$NCNN_DIR/install/include/ncnn" \
    -L"$NCNN_DIR/install/lib" -lncnn \
    -lpthread

echo ""
info "========== 全部完成 =========="
echo ""
echo "  运行方式:"
echo "    cd $TEST_DIR"
echo "    ./test_camera"
echo ""
echo "  操作:"
echo "    把红色物体放到摄像头前"
echo "    按 S 键 → 对当前画面分类"
echo "    按 Q 键 → 退出 (显示统计)"
echo ""
echo "  如果摄像头打不开:"
echo "    检查设备:  ls /dev/video*"
echo "    虚拟机需要: 设备 → USB → 勾选摄像头"
echo "========================================"
