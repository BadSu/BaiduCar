#include "predictor_api.h"
#include <cstdio>
#include <functional>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <sys/time.h>
using namespace cv;

void TransposeAndCopyToTensor(const Mat &src, NDTensor &dst) {
    Mat channels[3];
    split(src, channels);
    int offset = src.rows * src.cols;
    auto start = dst.value();
    for(int i = 0; i < 3; ++i) {
        std::memcpy(start + i * offset, channels[i].data,
                    offset * sizeof(float));
    }
}
std::unordered_map<std::string, NDTensor>
preprocess(const std::string &img_path,
           const std::vector<int64_t> &input_size = {320, 320}) {
    Mat x, frame;
    frame = imread(img_path);
    NDTensor scale_factor({1, 2}), img({1, 3, input_size[0], input_size[1]});
    scale_factor.value()[0] = static_cast<float>(input_size[0]) / frame.size[0];
    scale_factor.value()[1] = static_cast<float>(input_size[1]) / frame.size[1];
    NDTensor im_shape({1, 2});
    im_shape.value()[0] = input_size[0];
    im_shape.value()[1] = input_size[1];
    cvtColor(frame, x, COLOR_BGR2RGB);
    resize(x, x, Size(input_size[0], input_size[1]), 0, 0, 2);
    x.convertTo(x, CV_32FC3);
    x *= 1 / 255.0;
    subtract(x, Scalar(0.485, 0.456, 0.406), x);
    multiply(x, Scalar(1 / 0.229, 1 / 0.224, 1 / 0.225), x);
    TransposeAndCopyToTensor(x, img);

    return {{"image", img}, {"scale_factor", scale_factor}, {"im_shape", im_shape}};
}

void draw_box(const std::string &img_path, const NDTensor &res,
              float threshold = 0.2) {
    Mat img = imread(img_path);
    auto data = res.value();
    auto prod = std::accumulate(res.shape.begin(), res.shape.end(), 1,
                                std::multiplies<int64_t>());
    for(int i = 0; i < prod; i += 6) {
        // int label = data[i];
        float score = data[i + 1];
        if(score < threshold) {
            continue;
        }
        // std::cout << score << std::endl;
        int xmin = data[i + 2];
        int ymin = data[i + 3];
        int xmax = data[i + 4];
        int ymax = data[i + 5];
        rectangle(img, Point(xmin, ymin), Point(xmax, ymax), Scalar(0, 255, 0));
    }
    imwrite("render.jpg", img);
}
int main(int argc, char *argv[]) {
    assert(argc >= 3);
    std::string config_file(argv[1]), image_file(argv[2]);
    PPNCPredictor predictor(config_file);
    predictor.load();
    const std::vector<int64_t> feed_shape = {416, 416};
    auto feeds = preprocess(image_file, feed_shape);
    struct timeval tp;
    gettimeofday(&tp, NULL);
    double _sec = tp.tv_sec;
    double _usec = tp.tv_usec;
    predictor.set_inputs_zero_copy(feeds);
        // predictor.set_inputs(feeds);
    predictor.run();
    auto res = predictor.get_output(0);
    // for(int i = 0; i < 100; ++i) {
    //	predictor.set_inputs_zero_copy(feeds);
        // predictor.set_inputs(feeds);
    //	predictor.run();
    //	auto res = predictor.get_output(0);
    // }
    struct timeval tp_end;
    gettimeofday(&tp_end, NULL);
    std::cout << "time cost is " << (((double)(tp_end.tv_sec - _sec)) * 1000.0 + (tp_end.tv_usec - _usec) / 1000.0) / 101 << std::endl;
    draw_box(image_file, res);
    return 0;
}
