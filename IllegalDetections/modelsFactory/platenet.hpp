#ifndef PLATENET_H
#define PLATENET_H

#include <iostream>
#include <string>
#define USE_OPENCV 1
#include "bm_wrapper.hpp"
#include "bmnn_utils.h"
#include "utils.hpp"

#define BUFFER_SIZE (1024 * 500)

class PLATENET {
    std::shared_ptr<BMNNContext> m_bmContext;
    std::shared_ptr<BMNNNetwork> m_bmNetwork;
    std::vector<bm_image> m_resized_imgs;
    std::vector<bm_image> m_converto_imgs;
    int m_net_h, m_net_w;
    int max_batch;
    bmcv_convert_to_attr converto_attr;
    int output_num;
    int clas_char;
    int len_char;
    TimeStamp* ts_ = NULL;

public:
    PLATENET(std::shared_ptr<BMNNContext> context);
    ~PLATENET();
    int Init();
    int Detect(const std::vector<bm_image>& input_images, std::vector<std::string>& results);
    void enableProfile(TimeStamp* ts);
    int batch_size();

private:
    int pre_process(const std::vector<bm_image>& images);
    int post_process(const std::vector<bm_image>& images, std::vector<std::string>& results);
    int argmax(float* data, int dsize);
    std::string get_res(int pred_num[], int len_char, int clas_char);
};

#endif // PLATENET_H
