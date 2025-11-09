#ifndef MODEL_SFDI_HPP
#define MODEL_SFDI_HPP
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
                  TIME_BIN = 10000,
                  RHO_BIN = 1000,
                  FREQ_NUM = 2;
    using Tiff_img = Eigen::Tensor<double, 3, Eigen::RowMajor>;
    using SFDI_data = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, 3, FREQ_NUM>,
        Eigen::RowMajor>; // (H,W,C,P,F)
    using SFDI_AC = Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, FREQ_NUM>,
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
    using Reflect_wave_freq = Eigen::Array<double, WAVELENGTH_NUM, FREQ_NUM>;    // 固定大小二维数组
    using MC_data = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>; // 固定大小二维数组(T,R)读取蒙特卡罗模拟结果
    extern Eigen::Map<const Reflect_wave_freq> AC2Model(const SFDI_AC &ac, int h, int w);
    extern void Compute_AC(const SFDI_data &input, const Int_time &int_time, SFDI_AC &output);
    extern Tiff_img open_tiff(const std::string &filename);
    struct MC_Workspace
    {
        Eigen::ArrayXXd R_rho;
        Eigen::ArrayXXd term_noj;
        MC_Workspace(int wave_num, int rho_bin)
        {
            R_rho.resize(wave_num, rho_bin);
            term_noj.resize(wave_num, rho_bin);
        }
    };
    class model_SFDI
    {
    private:
        std::unique_ptr<SFDI_AC> ref_AC_ptr;
        std::unique_ptr<Reflect_wave_freq> ref_R_ptr; // (W,F,R)
        Optical_prop n, delta_t_div_fresnel, v;
        Freq frequency;
        Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> v_t, twopi_rho_drho; //(W,R)
        std::unique_ptr<Eigen::TensorFixedSize<
            double,
            Eigen::Sizes<WAVELENGTH_NUM, FREQ_NUM, RHO_BIN>,
            Eigen::RowMajor>>
            Jterm_ptr; // (W,F,R)
        std::unique_ptr<Int_time> int_time_ptr;
        std::vector<MC_Workspace> workspaces;
        void init_workspace(void);
    public:
        model_SFDI(
            const std::string &ref_folder = "reference_670",
            const std::string &R_of_rho_time_mc_path = "ROfRhoAndTime");
        ~model_SFDI() = default;
        Reflect_wave_freq diff_model_for_SFDI(const Optical_prop mua, const Optical_prop musp);
        void mc_model_for_SFDI(const Optical_prop mua, const Optical_prop musp, Reflect_wave_freq &dst);
        void LoadAndComputeAC(const std::string &folder, SFDI_AC &output_ac);
        void R_compute(const SFDI_AC &input_ac, SFDI_AC &output_R);
        void setFrequency(const Freq &freq);
        void setN(const Optical_prop &n);
        void setIntTime(const Int_time &int_time);
        void FreqTest(double start, double end, int num_points);
    };
}

#endif