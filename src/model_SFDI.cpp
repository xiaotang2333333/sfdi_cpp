#include "model_SFDI.hpp"
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace
{
    static SFDI::MC_data R_of_rho_time_mc;
    constexpr double light_speed = 299.792458; // 光速 mm/ns
    constexpr double PI = 3.14159265358979323846;
    constexpr double two_pi = 2.0 * PI;
    std::once_flag load_flag;
    void load_R_of_rho_time(const std::string &path)
    {
        std::ifstream fin(path, std::ios::binary);
        if (!fin.is_open())
            throw std::runtime_error("Monte Carlo open error:" + path);
        R_of_rho_time_mc.resize(SFDI::TIME_BIN, SFDI::RHO_BIN);
        fin.read(reinterpret_cast<char *>(R_of_rho_time_mc.data()),
                 sizeof(double) * (SFDI::TIME_BIN) * (SFDI::RHO_BIN));
        fin.close();
    }
    constexpr double delta_t = 0.1, delta_rho = 0.1;
    constexpr double mc_n = 1 - (1.4 - 1) / (1.4 + 1) * (1.4 - 1) / (1.4 + 1);
    static const Eigen::Array<double, SFDI::TIME_BIN, 1> time_var = Eigen::Array<double, SFDI::TIME_BIN, 1>::LinSpaced(0.05, 99.95);
    static const Eigen::Array<double, SFDI::RHO_BIN, 1> rho_var = Eigen::Array<double, SFDI::RHO_BIN, 1>::LinSpaced(0.05, 99.95);
    static Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> Rho_tensor_map(
        rho_var.data(), 1, 1, SFDI::RHO_BIN);
}
SFDI::model_SFDI::model_SFDI(
    const std::string &ref_folder,
    const std::string &R_of_rho_time_mc_path)
{
    std::cout << "Initializing SFDI model..." << std::endl;
    ref_AC_ptr = std::make_unique<SFDI_AC>();
    ref_R_ptr = std::make_unique<SFDI_Model>();
    Jterm_ptr = std::make_unique<Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<WAVELENGTH_NUM, FREQ_NUM, RHO_BIN>,
        Eigen::RowMajor>>();
    int_time_ptr = std::make_unique<Int_time>();
    int_time_ptr->constant(1.0); // 默认积分时间1s
    n.Zero();
    v.Constant(light_speed); // 光速 除以折射率
    frequency.Zero();
    std::cout << "Loading Monte Carlo data from: " << R_of_rho_time_mc_path << std::endl;
    std::call_once(load_flag, load_R_of_rho_time, R_of_rho_time_mc_path);
    std::cout << "Monte Carlo data loaded" << std::endl;
    v_t.resize(SFDI::WAVELENGTH_NUM, SFDI::TIME_BIN);
    term_same.resize(SFDI::WAVELENGTH_NUM, SFDI::RHO_BIN);
    term_same_tensor_ptr = std::make_unique<Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>>>(
        term_same.data(), SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN);
    frequency_tensor_ptr = std::make_unique<Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>>>(
        frequency.data(), 1, SFDI::FREQ_NUM, 1);
    term_same = two_pi * (rho_var.transpose().replicate(SFDI::WAVELENGTH_NUM, 1));
    setN(SFDI::Optical_prop().setConstant(1.37));
    setIntTime(SFDI::Int_time().setConstant(1)); // 默认积分时间1s
    SFDI::Freq freq_values;
    freq_values << 0.0, 0.2;
    setFrequency(freq_values);
    LoadAndComputeAC(ref_folder, *ref_AC_ptr);
    *ref_R_ptr = mc_model_for_SFDI(SFDI::Optical_prop().setConstant(0.0059), SFDI::Optical_prop().setConstant(0.9748));
}
SFDI::Tiff_img SFDI::open_tiff(const std::string &filename)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_UNCHANGED);
    if (img.empty())
    {
        std::cerr << "Error: Unable to open image file: " << filename << std::endl;
        return SFDI::Tiff_img(0, 0, 0);
    }

    // Ensure the image is 16-bit unsigned integer
    if (img.depth() != CV_16U)
    {
        img.convertTo(img, CV_16U);
    }
    img.convertTo(img, CV_64F);
    SFDI::Tiff_img temp;
    cv::cv2eigen(img, temp);
    return temp;
}
void SFDI::Compute_AC(const SFDI::SFDI_data &input, const SFDI::Int_time &int_time, SFDI::SFDI_AC &output)
{
    const double sqrt_2_over_3 = std::sqrt(2.0) / 3.0;

    Eigen::array<Eigen::Index, 4> reshape_dims = {1, 1, WAVELENGTH_NUM, 1};
    Eigen::array<Eigen::Index, 4> broadcast_dims = {IMG_HEIGHT, IMG_WIDTH, 1, FREQ_NUM};
    auto int_time_bcast = int_time.reshape(reshape_dims).broadcast(broadcast_dims);

    auto img_0 = input.chip(0, 3);
    auto img_120 = input.chip(1, 3);
    auto img_240 = input.chip(2, 3);

    auto numerator = (img_0 - img_120).square() +
                     (img_0 - img_240).square() +
                     (img_120 - img_240).square();

    output.device(Eigen::DefaultDevice()) =
        (numerator.sqrt() * sqrt_2_over_3 / int_time_bcast);
}
SFDI::SFDI_Model SFDI::model_SFDI::diff_model_for_SFDI(const SFDI::Optical_prop mua, const SFDI::Optical_prop musp)
{

    auto mutrans = mua + musp;
    auto Reff = -1.440 / this->n.square() + 0.710 / this->n + 0.668 + 0.0636 * this->n;
    auto A = (1 - Reff) / (2 * (1 + Reff));
    auto term1 = 3.0 * mua * mutrans;                     // shape (W)
    auto term2 = (2.0 * M_PI * this->frequency).square(); // shape (F)
    SFDI::SFDI_Model mueff_prime =
        term2.transpose().template replicate<WAVELENGTH_NUM, 1>(); // (W,F)
    mueff_prime.colwise() += term1;                                // 广播 term1 到每一列
    mueff_prime = mueff_prime.sqrt();                              // shape (W,F)
    SFDI::Optical_prop threeA = 3.0 * A;                           // (W,1)
    SFDI::Optical_prop num_col = threeA * musp * mutrans;          // (W,1)
    SFDI::SFDI_Model denom1 = mueff_prime;                         // (W,F)
    denom1.colwise() += mutrans;                                   // mueff + mutrans
    SFDI_Model denom2 = mueff_prime;                               // (W,F)
    denom2.colwise() += threeA * mutrans;                          // mueff + 3*A*mutrans
    SFDI::SFDI_Model reflectance_fx =
        num_col.template replicate<1, FREQ_NUM>() / (denom1 * denom2); // (W,F)
    return reflectance_fx;
}
SFDI::SFDI_Model SFDI::model_SFDI::mc_model_for_SFDI(const Optical_prop mua, const Optical_prop musp)
{
    Optical_prop musp_inv = (musp.inverse()).eval(); // (W, 1) 预计算倒数
    Eigen::TensorMap<Eigen::Tensor<double, 3, Eigen::RowMajor>> musp_inv_map(
        musp_inv.data(), SFDI::WAVELENGTH_NUM, 1, 1);
    // 计算时间域衰减并转换到空间域 R_rho
    auto decay = (-(v_t.colwise() * (mua * musp_inv))).exp();                                                    // (W, T)
    auto R_rho = (decay.matrix() * R_of_rho_time_mc.matrix()).array() * ((F_ratio_times_delta_t * musp.square()).replicate(1, RHO_BIN)); // (W, R)
    // term_same: rho * 2*PI / musp，形状 (W, R)
    auto term_same_scale = term_same * musp_inv.replicate(1, SFDI::RHO_BIN);

    auto Jterm = (*Jterm_ptr) * (musp_inv_map.broadcast(Eigen::array<Eigen::Index, 3>{1, FREQ_NUM, RHO_BIN}));
    // 构建积分权重 term_noj:  R_rho * term_same * delta_rho / musp
    Eigen::ArrayXXd term_noj = (R_rho * term_same_scale).colwise() * (delta_rho * musp_inv); // (W, R)

    Eigen::TensorMap<Eigen::Tensor<const double, 3, Eigen::RowMajor>>
        term_noj_tensor(term_noj.data(), SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN);
    // 执行汉克尔变换(沿 R 轴求和)
    auto result_tensor = (term_noj_tensor
                              .broadcast(Eigen::array<Eigen::Index, 3>{1, SFDI::FREQ_NUM, 1}) *
                          Jterm.bessel_j0())
                             .sum(Eigen::array<Eigen::Index, 1>{2}); // 对 R 轴求和
    // 映射结果到返回类型
    SFDI_Model result;
    Eigen::TensorMap<Eigen::Tensor<double, 2, Eigen::RowMajor>> result_map(
        result.data(), SFDI::WAVELENGTH_NUM, SFDI::FREQ_NUM);
    result_map = result_tensor;
    return result;
}
Eigen::Map<const SFDI::SFDI_Model> SFDI::AC2Model(const SFDI::SFDI_AC &ac, int h, int w)
{
    const double *ptr = ac.data();
    size_t offset = ((h * SFDI::IMG_WIDTH) + w) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM;
    return Eigen::Map<const SFDI::SFDI_Model>(ptr + offset);
}
void SFDI::model_SFDI::setFrequency(const Freq &freq_input)
{
    if (!freq_input.isApprox(this->frequency))
    {
        this->frequency = freq_input;
        (*Jterm_ptr) = (frequency_tensor_ptr->broadcast(Eigen::array<Eigen::Index, 3>{SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN}) *
                        Rho_tensor_map.broadcast(Eigen::array<Eigen::Index, 3>{WAVELENGTH_NUM, FREQ_NUM, 1}) * two_pi);
    }
}
void SFDI::model_SFDI::setN(const Optical_prop &input_n)
{
    if (!n.isApprox(input_n))
    {
        v = light_speed / input_n; // 光速除以折射率
        n = input_n;
        auto F = 1 - ((1 - input_n) / (1 + input_n)).square();
        F_ratio_times_delta_t = (F / mc_n * delta_t).eval();
        v_t = v.matrix() * time_var.transpose().matrix(); // (W,T) 预计算 v*t
    }
}
void SFDI::model_SFDI::setIntTime(const Int_time &int_time)
{
    this->int_time_ptr->operator=(int_time);
}
void SFDI::model_SFDI::LoadAndComputeAC(const std::string &folder, SFDI_AC &output_ac)
{
    if (folder.empty())
    {
        throw std::runtime_error("Error: Reference folder path is empty.");
    }
    std::unique_ptr<SFDI_data> inputdata_ptr = std::make_unique<SFDI_data>();
    Tiff_img img_0hz_0phase = open_tiff(folder + "/im01.tif"),
             img_0hz_120phase = open_tiff(folder + "/im02.tif"),
             img_0hz_240phase = open_tiff(folder + "/im03.tif"),
             img_02hz_0phase = open_tiff(folder + "/im04.tif"),
             img_02hz_120phase = open_tiff(folder + "/im05.tif"),
             img_02hz_240phase = open_tiff(folder + "/im06.tif"); //(H,W,C)
    Eigen::array<Eigen::Index, 5> extents = {
        SFDI::IMG_HEIGHT, SFDI::IMG_WIDTH, SFDI::WAVELENGTH_NUM, 1, 1};
    Eigen::array<Eigen::Index, 5> offsets_0hz_0phase = {0, 0, 0, 0, 0};
    inputdata_ptr->slice(offsets_0hz_0phase, extents) =
        img_0hz_0phase.reshape(extents); // 使用 reshape 匹配维度
    Eigen::array<Eigen::Index, 5> offsets_0hz_120phase = {0, 0, 0, 1, 0};
    inputdata_ptr->slice(offsets_0hz_120phase, extents) =
        img_0hz_120phase.reshape(extents);

    Eigen::array<Eigen::Index, 5> offsets_0hz_240phase = {0, 0, 0, 2, 0};
    inputdata_ptr->slice(offsets_0hz_240phase, extents) =
        img_0hz_240phase.reshape(extents);
    Eigen::array<Eigen::Index, 5> offsets_02hz_0phase = {0, 0, 0, 0, 1};
    inputdata_ptr->slice(offsets_02hz_0phase, extents) =
        img_02hz_0phase.reshape(extents);

    Eigen::array<Eigen::Index, 5> offsets_02hz_120phase = {0, 0, 0, 1, 1};
    inputdata_ptr->slice(offsets_02hz_120phase, extents) =
        img_02hz_120phase.reshape(extents);
    Eigen::array<Eigen::Index, 5> offsets_02hz_240phase = {0, 0, 0, 2, 1};
    inputdata_ptr->slice(offsets_02hz_240phase, extents) =
        img_02hz_240phase.reshape(extents);
    Compute_AC(*inputdata_ptr, *int_time_ptr, output_ac);
}
/// @brief 计算输入图片集反射率，用于后续反演出mua musp
/// @param input_ac
/// @return 输入图片的每个像素点的反射率
void SFDI::model_SFDI::R_compute(const SFDI::SFDI_AC &input_ac, SFDI::SFDI_AC &output_R)
{
    Eigen::TensorMap<const Eigen::Tensor<double, 4, Eigen::RowMajor>> ref_R_tensor(
        ref_R_ptr->data(), 1, 1, SFDI::WAVELENGTH_NUM, SFDI::FREQ_NUM);
    output_R = ref_R_tensor.broadcast(Eigen::array<Eigen::Index, 4>{SFDI::IMG_HEIGHT, SFDI::IMG_WIDTH, 1, 1}) / (*ref_AC_ptr) * input_ac;
}
