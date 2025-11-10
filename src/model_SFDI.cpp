#include "model_SFDI.hpp"
#include <omp.h>
#include <fstream>
#include <stdexcept>
namespace
{
    static SFDI::MC_data R_of_rho_time_mc;
    constexpr double light_speed = 299.792458; // 光速 mm/ns
    constexpr double two_pi = 2.0 * M_PI;
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
    constexpr double delta_t = 0.01, delta_rho = 0.1;
    static const Eigen::ArrayXd time_var = Eigen::ArrayXd::LinSpaced(SFDI::TIME_BIN, delta_t / 2, delta_t *SFDI::TIME_BIN - delta_t / 2);
    static const Eigen::ArrayXd rho_var = Eigen::ArrayXd::LinSpaced(SFDI::RHO_BIN, delta_rho / 2, delta_rho *SFDI::RHO_BIN - delta_rho / 2);
}
SFDI::model_SFDI::model_SFDI(
    const std::string &ref_folder,
    const std::string &R_of_rho_time_mc_path)
{
    std::cout << "Initializing SFDI model..." << std::endl;
    ref_AC_ptr = std::make_unique<SFDI_AC>();
    ref_R_ptr = std::make_unique<Reflect_wave_freq>();
    Jterm_ptr = std::make_unique<Eigen::TensorFixedSize<
        double,
        Eigen::Sizes<WAVELENGTH_NUM, FREQ_NUM, RHO_BIN>,
        Eigen::RowMajor>>();
    int_time_ptr = std::make_unique<Int_time>();
    int_time_ptr->setConstant(1.0); // 默认积分时间1s
    std::cout << "Loading Monte Carlo data from: " << R_of_rho_time_mc_path << std::endl;
    std::call_once(load_flag, load_R_of_rho_time, R_of_rho_time_mc_path);
    std::cout << "Monte Carlo data loaded" << std::endl;
    v_t.resize(SFDI::WAVELENGTH_NUM, SFDI::TIME_BIN);
    twopi_rho_drho.resize(SFDI::WAVELENGTH_NUM, SFDI::RHO_BIN);
    twopi_rho_drho = delta_rho * two_pi * (rho_var.transpose().replicate(SFDI::WAVELENGTH_NUM, 1));
    setN(SFDI::Optical_prop().Constant(1.37));
    setFrequency((SFDI::Freq() << 0.0, 0.2).finished());
    LoadAndComputeAC(ref_folder, *ref_AC_ptr);
    init_workspace();
    diff_model_for_SFDI(SFDI::Optical_prop().setConstant(0.0059), SFDI::Optical_prop().setConstant(0.9748), *ref_R_ptr);
    std::cout << *ref_R_ptr << std::endl;
    mc_model_for_SFDI(SFDI::Optical_prop().setConstant(0.0059), SFDI::Optical_prop().setConstant(0.9748), *ref_R_ptr);
    std::cout << *ref_R_ptr << std::endl;
}
SFDI::Tiff_img SFDI::open_tiff(const std::string &filename)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_UNCHANGED);
    if (img.empty())
    {
        std::cerr << "Error: Unable to open image file: " << filename << std::endl;
        return SFDI::Tiff_img(0, 0, 0);
    }
    // 保留原始浮点数：如果是 32F/64F 直接转换到 64F；如果是 16U 则提升到 64F；其它类型统一提升
    switch (img.depth())
    {
    case CV_16U:
        img.convertTo(img, CV_64F); // 16位提升为 double
        break;
    case CV_32F:
        img.convertTo(img, CV_64F); // float -> double 保留数值
        break;
    case CV_64F:
        // 已经是 double 保持不变
        break;
    default:
        // 其它整型类型提升到 double（可能丢失原本的动态范围但保持值）
        img.convertTo(img, CV_64F);
        break;
    }
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
void SFDI::model_SFDI::diff_model_for_SFDI(const SFDI::Optical_prop mua, const SFDI::Optical_prop musp, Reflect_wave_freq &dst)
{

    auto mutrans = mua + musp;
    auto Reff = -1.440 / this->n.square() + 0.710 / this->n + 0.668 + 0.0636 * this->n;
    auto A = (1 - Reff) / (2 * (1 + Reff));
    auto term1 = 3.0 * mua * mutrans;                     // shape (W)
    auto term2 = (2.0 * M_PI * this->frequency).square(); // shape (F)
    SFDI::Reflect_wave_freq mueff_prime =
        term2.transpose().template replicate<WAVELENGTH_NUM, 1>(); // (W,F)
    mueff_prime.colwise() += term1;                                // 广播 term1 到每一列
    mueff_prime = mueff_prime.sqrt();                              // shape (W,F)
    SFDI::Optical_prop threeA = 3.0 * A;                           // (W,1)
    SFDI::Optical_prop num_col = threeA * musp * mutrans;          // (W,1)
    SFDI::Reflect_wave_freq denom1 = mueff_prime;                  // (W,F)
    denom1.colwise() += mutrans;                                   // mueff + mutrans
    Reflect_wave_freq denom2 = mueff_prime;                        // (W,F)
    denom2.colwise() += threeA * mutrans;                          // mueff + 3*A*mutrans
    dst =
        num_col.template replicate<1, FREQ_NUM>() / (denom1 * denom2); // (W,F)
}
void SFDI::model_SFDI::mc_model_for_SFDI(const Optical_prop mua, const Optical_prop musp, SFDI::Reflect_wave_freq &dst)
{
    // (W, 1) 预计算倒数
    Optical_prop musp_inv = musp.inverse();
    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> musp_inv_map(musp_inv.data(), SFDI::WAVELENGTH_NUM, 1, 1);
    int thread_id = omp_get_thread_num();
    MC_Workspace &ws = workspaces[thread_id];
    // 计算时间域衰减并转换到空间域 R_rho
    // (W, T) * (T, R) -> (W, R)
    auto decay = (-(v_t.colwise() * (mua * musp_inv))).exp();        // exp(-v * t * mua / musp )
    ws.R_rho = (decay.matrix() * R_of_rho_time_mc.matrix()).array(); // R_rho = R_of_rho_time_mc /F * decay *dt/musp *musp^3
    // 构建积分权重 term_noj:  R_rho * twopi_rho_drho  / musp^2
    ws.term_noj = ws.R_rho * twopi_rho_drho * delta_t_div_fresnel.replicate(1, SFDI::RHO_BIN); //

    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>>
        term_noj_tensor(ws.term_noj.data(), SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN);
    Eigen::TensorMap<Eigen::Tensor<double, 2, Eigen::RowMajor>> dst_map(dst.data(), SFDI::WAVELENGTH_NUM, SFDI::FREQ_NUM);
    // 执行汉克尔变换(沿 R 轴求和)
    dst_map = (term_noj_tensor.broadcast(Eigen::array<Eigen::Index, 3>{1, SFDI::FREQ_NUM, 1}) *
               ((*Jterm_ptr) * (musp_inv_map.broadcast(Eigen::array<Eigen::Index, 3>{1, SFDI::FREQ_NUM, SFDI::RHO_BIN}))).bessel_j0())
                  .sum(Eigen::array<Eigen::Index, 1>{2}); // 对 R 轴求和
}
Eigen::Map<const SFDI::Reflect_wave_freq> SFDI::AC2Model(const SFDI::SFDI_AC &ac, int h, int w)
{
    const double *ptr = ac.data();
    size_t offset = ((h * SFDI::IMG_WIDTH) + w) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM;
    return Eigen::Map<const SFDI::Reflect_wave_freq>(ptr + offset);
}
void SFDI::model_SFDI::setFrequency(const Freq &freq_input)
{
    if (!freq_input.isApprox(this->frequency))
    {
        std::cout << "Setting modulation frequency to: " << freq_input.transpose() << " mm^-1" << std::endl;
        frequency = freq_input;
        Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> frequency_tensor(
            frequency.data(), 1, SFDI::FREQ_NUM, 1);
        Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> Rho_tensor_map(
            rho_var.data(), 1, 1, SFDI::RHO_BIN);
        (*Jterm_ptr) = (
            frequency_tensor.broadcast(Eigen::array<Eigen::Index, 3>{SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN}) 
            * Rho_tensor_map.broadcast(Eigen::array<Eigen::Index, 3>{WAVELENGTH_NUM, FREQ_NUM, 1})             
            ) * two_pi;
    }
}
void SFDI::model_SFDI::setN(const Optical_prop &input_n)
{
    if (!n.isApprox(input_n))
    {
        std::cout << "Setting refractive index to: " << input_n.transpose() << std::endl;
        v = light_speed / input_n; // 光速除以折射率
        n = input_n;
        auto F = 1 - ((1 - input_n) / (1 + input_n)).square();
        delta_t_div_fresnel = (delta_t / F).eval();
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
void SFDI::model_SFDI::FreqTest(double start, double end, int num_points)
{
    Eigen::ArrayXd test_freqs = Eigen::ArrayXd::LinSpaced(num_points, start, end);
    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> test_freqs_map(test_freqs.data(), 1, num_points, 1);
    Optical_prop mua_test = Optical_prop::Constant(0.01), musp_test = Optical_prop::Constant(1.0);
    Eigen::ArrayXXd R_rho, term_noj;
    R_rho.resize(SFDI::WAVELENGTH_NUM, SFDI::RHO_BIN);
    term_noj.resize(SFDI::WAVELENGTH_NUM, SFDI::RHO_BIN);
    auto decay = (-(v_t.colwise() * mua_test)).exp();             // exp(-v * t * mua / musp ) =  exp(-v * t * mua)
    R_rho = (decay.matrix() * R_of_rho_time_mc.matrix()).array(); // R_rho = sum(R_of_rho_time*decay * dt / F ) * musp ^ 2
    // 构建积分权重 term_noj:  R_rho * twopi_rho_drho  / musp^2
    term_noj = R_rho * twopi_rho_drho * delta_t_div_fresnel.replicate(1, SFDI::RHO_BIN);
    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>>
        term_noj_tensor(term_noj.data(), SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN);
    Eigen::Tensor<double, 2, Eigen::RowMajor> R_freqs(SFDI::WAVELENGTH_NUM, num_points);
    Eigen::TensorMap<const Eigen::Tensor<double, 3, Eigen::RowMajor>> Rho_tensor_map(
        rho_var.data(), 1, 1, SFDI::RHO_BIN);
    Eigen::Tensor<double, 3, Eigen::RowMajor> Jterm(SFDI::WAVELENGTH_NUM, num_points, SFDI::RHO_BIN);
    Jterm = (test_freqs_map.broadcast(Eigen::array<Eigen::Index, 3>{SFDI::WAVELENGTH_NUM, 1, SFDI::RHO_BIN}) * // 2pi*f*rho / musp = 2pi*f*rho
             Rho_tensor_map.broadcast(Eigen::array<Eigen::Index, 3>{SFDI::WAVELENGTH_NUM, num_points, 1}) * two_pi);
    R_freqs = (term_noj_tensor.broadcast(Eigen::array<Eigen::Index, 3>{1, num_points, 1}) *
               Jterm.bessel_j0())
                  .sum(Eigen::array<Eigen::Index, 1>{2}); // 对 R 轴求和

    // 以二进制保存 R_freqs (按行主序, 行: 波长, 列: 频率点)
    std::ofstream fout_bin("R_freqs.bin", std::ios::binary);
    if (!fout_bin.is_open())
        throw std::runtime_error("Cannot open file to write R_freqs.bin");
    fout_bin.write(reinterpret_cast<const char *>(R_freqs.data()),
                   sizeof(double) * static_cast<std::size_t>(SFDI::WAVELENGTH_NUM) * static_cast<std::size_t>(num_points));
    fout_bin.close();
}
void SFDI::model_SFDI::init_workspace(void)
{
    std::cout << "Initializing workspaces for parallel execution..." << std::endl;
    int max_threads = omp_get_max_threads(); // 获取 OpenMP 将使用的最大线程数
    for (int i = 0; i < max_threads; ++i)
    {
        workspaces.emplace_back(SFDI::WAVELENGTH_NUM, SFDI::RHO_BIN);
    }
    std::cout << max_threads << " workspaces created." << std::endl;
}