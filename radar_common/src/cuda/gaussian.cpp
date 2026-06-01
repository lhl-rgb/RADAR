/// @file gaussian.cpp
/// @brief CUDA 复高斯噪声生成器的桩实现（无 CUDA 时的回退）
///
/// 当编译时未启用 CUDA 支持时，本文件提供桩（stub）函数实现，
/// 所有函数返回 false 并设置相应的错误信息。

#include "cuda/gaussian.cuh"

namespace radar::cuda
{

    bool is_gaussian_generator_available(std::string *error)
    {
        if (error != nullptr)
        {
            *error = "CUDA Gaussian generator is not available in this build.";
        }
        return false;
    }

    bool generate_complex_gaussian_host(Complex *,
                                        std::size_t,
                                        Scalar,
                                        std::uint64_t,
                                        std::uint64_t,
                                        std::string *error)
    {
        if (error != nullptr)
        {
            *error = "CUDA Gaussian generator is not available in this build.";
        }
        return false;
    }

} // namespace radar::cuda
