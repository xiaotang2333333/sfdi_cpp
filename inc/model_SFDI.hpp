#ifndef MODEL_SFDI
#define MODEL_SFDI
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/CXX11/Tensor>
#include <unsupported/Eigen/SpecialFunctions>

namespace SFDI
{
    constexpr int IMG_HEIGHT = 512,
                  IMG_WIDTH = 672,
                  WAVELENGTH_NUM = 1,
                  TIME_BIN = 1000,
                  RHO_BIN = 1000,
                  FREQ_NUM = 2;
    using Tiff_img = Eigen::Tensor<double, 3, Eigen::RowMajor>;
    using SFDI_data = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, 3, 2>,
        Eigen::RowMajor>; // (H,W,C,P,F)
    using SFDI_AC = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, 2>,
        Eigen::RowMajor>; // (H,W,C,F) AC分量计算结果
    using Int_time = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<WAVELENGTH_NUM>,
        Eigen::RowMajor>;
    using Optical_prop_map = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM>,
        Eigen::RowMajor>;                                                 // 一维数组 包含每个波长的积分时间
    using Optical_prop = Eigen::Array<double, WAVELENGTH_NUM, 1>;         // 光学特性 与 波长相关
    using Freq = Eigen::Array<double, FREQ_NUM, 1>;                       // 固定长度一维向量
    using SFDI_Model = Eigen::Array<double, WAVELENGTH_NUM, FREQ_NUM>;    // 固定大小二维数组
    using MC_data = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>; // 固定大小二维数组(T,R)读取蒙特卡罗模拟结果
    extern Eigen::Map<const SFDI_Model> AC2Model(const SFDI_AC &ac, int h, int w);
    extern void Compute_AC(const SFDI_data &input, const Int_time &int_time, SFDI_AC &output);
    extern Tiff_img open_tiff(const std::string &filename);
    class model_SFDI
    {
    private:
        std::unique_ptr<SFDI_data> ref_data, sample_data;
        Optical_prop last_n, F_ratio_times_delta_t, v;
        Freq frequency;
        Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> v_t, term_same, term_noj;
        std::unique_ptr<Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>>> term_same_tensor_ptr, frequency_tensor_ptr, term_noj_tensor_ptr;
        std::unique_ptr<Eigen::TensorFixedSize<
            double,
            Eigen::Sizes<WAVELENGTH_NUM, FREQ_NUM, RHO_BIN>,
            Eigen::RowMajor>> Jterm_ptr; // (W,F,R)
    public:
        model_SFDI(
            const std::string &ref_folder = "reference_670",
            const std::string &sample_folder = "sample_670",
            const std::string &R_of_rho_time_mc_path = "R_of_rho_time_mc_670.bin");
        ~model_SFDI() = default;
        SFDI_Model diff_model_for_SFDI(const Optical_prop mua, const Optical_prop musp, const Optical_prop n, const Freq frequency);
        SFDI_Model mc_model_for_SFDI(const Optical_prop mua, const Optical_prop musp, const Optical_prop n, const Freq frequency);
    };
}

#endif