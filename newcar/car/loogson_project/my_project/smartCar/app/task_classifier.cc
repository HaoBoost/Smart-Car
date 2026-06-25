#include "app_context.h"

namespace {

const char* kModelParamName = "tiny_classifier_fp32.ncnn.param";
const char* kModelBinName = "tiny_classifier_fp32.ncnn.bin";
const char* kLabelsName = "labels.txt";

bool g_classifier_initialized = false;
bool g_classifier_available = false;
bool g_classifier_warned_unavailable = false;
std::string g_classifier_last_printed_label;
std::string g_model_param_path;
std::string g_model_bin_path;
std::string g_labels_path;
ncnn::Net g_classifier_net;
std::vector<std::string> g_classifier_labels;
const int kClassifierInputSize = 40;
const float kMeanVals[3] = {123.675f, 116.28f, 103.53f};
const float kNormVals[3] = {0.01712475f, 0.017507f, 0.01742919f};

void clear_last_classification_state()
{
    g_red_model_last_label.clear();
    g_red_model_last_score = 0.0f;
    g_red_model_last_probs.clear();
    g_red_model_last_probs_text.clear();
}

bool load_labels(std::vector<std::string>& labels)
{
    std::ifstream ifs(g_labels_path.c_str());
    if (!ifs.is_open()) return false;

    labels.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty()) {
            labels.push_back(line);
        }
    }

    return !labels.empty();
}

std::string dirname_of(const std::string& path)
{
    const size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return path.substr(0, pos);
}

std::string join_path(const std::string& dir, const std::string& name)
{
    if (dir.empty() || dir == ".") {
        return std::string("./") + name;
    }
    if (dir.back() == '/') {
        return dir + name;
    }
    return dir + "/" + name;
}

std::string resolve_executable_dir()
{
    char exe_path[PATH_MAX] = {};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0) return ".";
    exe_path[len] = '\0';
    return dirname_of(std::string(exe_path));
}

bool file_exists(const std::string& path)
{
    return access(path.c_str(), F_OK) == 0;
}

bool resolve_classifier_paths()
{
    const std::string exe_dir = resolve_executable_dir();
    const std::string cwd_dir = ".";

    const std::string param_in_exe_dir = join_path(exe_dir, kModelParamName);
    const std::string bin_in_exe_dir = join_path(exe_dir, kModelBinName);
    const std::string labels_in_exe_dir = join_path(exe_dir, kLabelsName);
    const std::string param_in_cwd = join_path(cwd_dir, kModelParamName);
    const std::string bin_in_cwd = join_path(cwd_dir, kModelBinName);
    const std::string labels_in_cwd = join_path(cwd_dir, kLabelsName);

    g_model_param_path = file_exists(param_in_exe_dir) ? param_in_exe_dir : param_in_cwd;
    g_model_bin_path = file_exists(bin_in_exe_dir) ? bin_in_exe_dir : bin_in_cwd;
    g_labels_path = file_exists(labels_in_exe_dir) ? labels_in_exe_dir : labels_in_cwd;

    printf("[NCNN] classifier asset probe: exe_dir=%s cwd=.\n", exe_dir.c_str());
    printf("[NCNN] classifier asset path: param=%s %s\n",
           g_model_param_path.c_str(), file_exists(g_model_param_path) ? "(found)" : "(missing)");
    printf("[NCNN] classifier asset path: bin=%s %s\n",
           g_model_bin_path.c_str(), file_exists(g_model_bin_path) ? "(found)" : "(missing)");
    printf("[NCNN] classifier asset path: labels=%s %s\n",
           g_labels_path.c_str(), file_exists(g_labels_path) ? "(found)" : "(missing)");

    return file_exists(g_model_param_path)
        && file_exists(g_model_bin_path)
        && file_exists(g_labels_path);
}

bool init_classifier_once()
{
    if (g_classifier_initialized) {
        return g_classifier_available;
    }

    g_classifier_initialized = true;
    g_classifier_net.opt.use_vulkan_compute = false;
    g_classifier_net.opt.num_threads = 1;

    if (!resolve_classifier_paths()) {
        fprintf(stderr, "[NCNN] classifier assets missing, please verify deployed model files\n");
        return false;
    }

    if (g_classifier_net.load_param(g_model_param_path.c_str()) != 0) {
        fprintf(stderr, "[NCNN] load param failed: %s\n", g_model_param_path.c_str());
        return false;
    }
    if (g_classifier_net.load_model(g_model_bin_path.c_str()) != 0) {
        fprintf(stderr, "[NCNN] load model failed: %s\n", g_model_bin_path.c_str());
        return false;
    }
    if (!load_labels(g_classifier_labels)) {
        fprintf(stderr, "[NCNN] load labels failed: %s\n", g_labels_path.c_str());
        return false;
    }

    const std::vector<const char*>& input_names = g_classifier_net.input_names();
    const std::vector<const char*>& output_names = g_classifier_net.output_names();
    if (input_names.empty() || output_names.empty()) {
        fprintf(stderr, "[NCNN] invalid model io definition\n");
        return false;
    }

    g_classifier_available = true;
    printf("[NCNN] classifier ready: input=%s output=%s labels=%zu\n",
           input_names[0], output_names[0], g_classifier_labels.size());
    return true;
}

int argmax_score(const ncnn::Mat& out, float& best_score)
{
    best_score = -1e30f;
    int best_idx = -1;
    for (int i = 0; i < out.w; ++i) {
        const float score = out[i];
        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }
    return best_idx;
}

std::vector<float> softmax_scores(const ncnn::Mat& out)
{
    std::vector<float> probs;
    if (out.w <= 0) return probs;

    probs.resize(out.w);
    float max_logit = out[0];
    for (int i = 1; i < out.w; ++i) {
        if (out[i] > max_logit) {
            max_logit = out[i];
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < out.w; ++i) {
        probs[i] = std::exp(out[i] - max_logit);
        sum += probs[i];
    }

    if (sum <= 0.0f) {
        probs.assign(out.w, 0.0f);
        return probs;
    }

    for (size_t i = 0; i < probs.size(); ++i) {
        probs[i] /= sum;
    }
    return probs;
}

std::string format_probabilities(const std::vector<float>& probs)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < probs.size(); ++i) {
        if (i > 0) {
            oss << " ";
        }
        const std::string label = i < g_classifier_labels.size()
            ? g_classifier_labels[i]
            : ("class_" + std::to_string(i));
        oss << label << "=" << probs[i];
    }
    return oss.str();
}

bool run_inference_with_pixel_type(int pixel_type, ncnn::Mat& out)
{
    const std::vector<const char*>& input_names = g_classifier_net.input_names();
    const std::vector<const char*>& output_names = g_classifier_net.output_names();
    if (input_names.empty() || output_names.empty()) {
        return false;
    }

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(
        g_red_model_input_40.data,
        pixel_type,
        g_red_model_input_40.cols,
        g_red_model_input_40.rows,
        kClassifierInputSize,
        kClassifierInputSize);
    in.substract_mean_normalize(kMeanVals, kNormVals);

    ncnn::Extractor ex = g_classifier_net.create_extractor();
    ex.set_light_mode(true);
    ex.input(input_names[0], in);
    return ex.extract(output_names[0], out) == 0 && !out.empty();
}

}  // namespace

bool run_red_model_classification_if_needed(void)
{
    if (!g_red_model_classify_enabled || !g_red_model_input_ready || g_red_model_input_40.empty()) {
        return false;
    }

    if (!init_classifier_once()) {
        if (!g_classifier_warned_unavailable) {
            g_classifier_warned_unavailable = true;
            fprintf(stderr, "[NCNN] classifier unavailable, skip inference\n");
        }
        clear_last_classification_state();
        return false;
    }

    ncnn::Mat out;
    if (!run_inference_with_pixel_type(ncnn::Mat::PIXEL_BGR2RGB, out)) {
        fprintf(stderr, "[NCNN] extract failed\n");
        clear_last_classification_state();
        return false;
    }

    float best_score = 0.0f;
    const int best_idx = argmax_score(out, best_score);
    if (best_idx < 0) {
        clear_last_classification_state();
        return false;
    }

    g_red_model_last_probs = softmax_scores(out);
    if (best_idx >= 0 && best_idx < static_cast<int>(g_red_model_last_probs.size())) {
        g_red_model_last_score = g_red_model_last_probs[best_idx];
    } else {
        g_red_model_last_score = 0.0f;
    }

    if (best_idx >= 0 && best_idx < static_cast<int>(g_classifier_labels.size())) {
        g_red_model_last_label = g_classifier_labels[best_idx];
    } else {
        g_red_model_last_label = "class_" + std::to_string(best_idx);
    }
    g_red_model_last_probs_text = format_probabilities(g_red_model_last_probs);

    printf("[NCNN] red roi probs: %s\n", g_red_model_last_probs_text.c_str());

    if (g_red_model_last_label != g_classifier_last_printed_label) {
        g_classifier_last_printed_label = g_red_model_last_label;
        printf("[NCNN] red roi classify: label=%s score=%.6f box=(%d,%d,%d,%d)\n",
               g_red_model_last_label.c_str(),
               g_red_model_last_score,
               g_red_model_box_full.x,
               g_red_model_box_full.y,
               g_red_model_box_full.width,
               g_red_model_box_full.height);
    }

    g_red_model_cycle_done = true;
    return true;
}
