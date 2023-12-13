#ifndef _COMMON_SEMANTIC_H_
#define _COMMON_SEMANTIC_H_
#include <iostream>

#include "basics.h"
namespace jarvis {
namespace decision_lib {
template <typename T, int N_DIM>
class GridMapND {
public:
    enum ValType {
        OCCUPIED = 70,
        FREE = 0,
        SCANNED_OCCUPIED = 128,
        UNKNOWN = 0
    };
    /**
     * @brief Construct a new GridMapND object
     *
     */
    GridMapND();
    /**
     * @brief Construct a new GridMapND object
     *
     * @param dims_size size of each dimension
     * @param dims_resolution resolution of each dimension
     * @param dims_name name of each dimension
     */
    GridMapND(const std::array<int, N_DIM> &dims_size,
              const std::array<decimal_t, N_DIM> &dims_resolution,
              const std::array<std::string, N_DIM> &dims_name);

private:
    /**
     * @brief Set the steps of N-dim array
     * @brief E.g. A x-y-z map's steps are {1,x,x*y}
     *
     * @param dims_size Input the size of dimension
     * @return ErrorType
     */
    ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

    /**
     * @brief Get the Data size
     *
     * @param dims_size Input the size of dimension
     * @return ErrorType
     */
    ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

    std::array<int, N_DIM> dims_size_;
    std::array<int, N_DIM> dims_step_;
    std::array<decimal_t, N_DIM> dims_resolution_;
    std::array<std::string, N_DIM> dims_name_;
    std::array<decimal_t, N_DIM> origin_;

    int data_size_{0};
    std::vector<T> data_;
};

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<decimal_t, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
    dims_size_ = dims_size;
    dims_resolution_ = dims_resolution;
    dims_name_ = dims_name;
    std::cout << "ddddd" << std::endl;
    SetNDimSteps(dims_size_);
    SetDataSize(dims_size_);
    data_ = std::vector<T>(data_size_, 0);
    origin_.fill(0);
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
    int step = 1;
    for (int i = 0; i < N_DIM; ++i) {
        dims_step_[i] = step;
        step = step * dims_size[i];
    }
    return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
    int total_ele_num = 1;
    for (int i = 0; i < N_DIM; ++i) {
        total_ele_num = total_ele_num * dims_size[i];
    }
    data_size_ = total_ele_num;
    return kSuccess;
}

}  // namespace decision_lib
}  // namespace jarvis
#endif
