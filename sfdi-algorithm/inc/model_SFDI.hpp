#pragma once
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <mutex>
#include <fstream>
#include <iostream>
#include <unsupported/Eigen/CXX11/Tensor>
#include <unsupported/Eigen/SpecialFunctions>
namespace SFDI
{

    constexpr int TIME_BIN = 200,
                  RHO_BIN = 200,
                  FREQ_NUM = 2;
    using Tiff_img = Eigen::Tensor<double, 3, Eigen::RowMajor>;
    using Freq = Eigen::Array2d;                                          // 固定长度一维向量
    using Reflect = Eigen::Array<double, 1, FREQ_NUM>;               // 固定大小一维数组
    using MC_data = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic>; // 固定大小二维数组(T,R)读取蒙特卡罗模拟结果
    struct SFDI_Result
    {
        double mua;               // 吸收系数
        double musp;              // 约化散射系数
        SFDI::Reflect model; // 计算结果 (WAVELENGTH_NUM × FREQ_NUM)
    };
    extern void Compute_Amplitude_Envelope(const Eigen::ArrayXXd &inputp0,
                                           const Eigen::ArrayXXd &inputp120,
                                           const Eigen::ArrayXXd &inputp240,
                                           const double int_time,
                                           Eigen::ArrayXXd &out);
    extern Tiff_img open_tiff(const std::string &filename);
    extern bool save_tiff(const std::string &filename, const Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &data);
    class mc_model
    {
    private:
        double n, delta_t_div_fresnel, v;
        Freq frequency;
        Eigen::ArrayXXd Jterm;               //(F,R)
        Eigen::ArrayXd v_t, twopi_rho_drho; //(T,1) (R,1)
    public:
        mc_model(
            const std::string &R_of_rho_time_mc_path = "ROfRhoAndTime");
        ~mc_model() = default;
        void mc_model_for_SFDI(const double mua, const double musp, Reflect &dst) const;
        void mc_model_for_SFDI_Dmua(const double mua, const double musp, Reflect &dst) const;
        void mc_model_for_SFDI_Dmusp(const double mua, const double musp, Reflect &dst) const;
        void setFrequency(const Freq &freq);
        void setN(const double n);
    };
};