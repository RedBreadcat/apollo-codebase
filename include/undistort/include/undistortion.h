// Copyright 2017 Baidu Inc. All Rights Reserved.
// @author: GAO,Liang (gaoliang07@baidu.com)
// @author: WAN,Ji (wanji@baidu.com)
// @file: undistortion.h
// @brief: GPU undistortion

#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H

#include <stdint.h>
#include <stdlib.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>

#define CUDA_CHECK(condition) \
do { \
    cudaError_t error = condition; \
    if (error != cudaSuccess) { \
        std::cerr << cudaGetErrorString(error); \
        return (int)error; \
    } \
} while (0)

namespace apollo {
namespace perception {

class ImageGpuPreprocessHandler {
public:

    ImageGpuPreprocessHandler() {
        _inited = false;
        _d_mapx = NULL;
        _d_mapy = NULL;
        _d_rgb = NULL;
        _d_dst = NULL;
    };

    ~ImageGpuPreprocessHandler() {
        release();
    };

    inline int set_device(int dev) {
        CUDA_CHECK(cudaSetDevice(_dev_no));
        return 0;
    }
    int init(const std::string &intrinsics_path, int dev);
    int handle(uint8_t *src, uint8_t *dst);
    int release(void);

private:
    int load_camera_intrinsics(const std::string &intrinsics_path, int *width, int *height,
                               std::vector<double> *D, std::vector<double> *K);

    float *_d_mapx;
    float *_d_mapy;
    uint8_t *_d_rgb;
    uint8_t *_d_dst;

    int _width;     // image cols
    int _height;    // image rows
    int _in_size;   // size of the input image in byte
    int _out_size;  // size of the output image in byte
    int _dev_no;    // device number for gpu
    bool _inited;
    const int CHANNEL = 3;
};

}  // namespace perception
}  // namespace adu

#endif // ADU_PERCEPTION_OBSTACLE_CAMERA_UNDISTORTION_UNDISTORTION_H
