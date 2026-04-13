#include "model_SFDI.hpp"
#include <tiffio.h>
#include <omp.h>
#include <vector>
#include <cstdint>
#include <chrono>
#include <iostream>
namespace
{
    constexpr double origin_n = 1.4;

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
    constexpr double delta_t = 2.0 / SFDI::TIME_BIN, delta_rho = 20.0 / SFDI::RHO_BIN;
    static const Eigen::ArrayXd time_var = Eigen::ArrayXd::LinSpaced(SFDI::TIME_BIN, delta_t / 2, delta_t *SFDI::TIME_BIN - delta_t / 2);
    static const Eigen::ArrayXd rho_var = Eigen::ArrayXd::LinSpaced(SFDI::RHO_BIN, delta_rho / 2, delta_rho *SFDI::RHO_BIN - delta_rho / 2);

    class ScopedTimer
    {
    public:
        explicit ScopedTimer(const char *label)
            : m_label(label), m_begin(std::chrono::steady_clock::now())
        {
        }

        ~ScopedTimer()
        {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_begin).count();
            std::cerr << m_label << " took " << elapsedMs << " ms" << std::endl;
        }

    private:
        const char *m_label;
        std::chrono::steady_clock::time_point m_begin;
    };
}
Eigen::ArrayXXd SFDI::open_tiff(const std::string &filename)
{
    TIFF *tif = TIFFOpen(filename.c_str(), "r");
    if (!tif)
    {
        std::cerr << "Error: Unable to open TIFF file: " << filename << std::endl;
        return Eigen::ArrayXXd(0, 0);
    }
    uint32_t width = 0, height = 0;
    uint16_t samples = 1, bps = 8, sampleformat = SAMPLEFORMAT_UINT;
    uint16_t planar = PLANARCONFIG_CONTIG;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &samples);
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &bps);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &sampleformat);
    TIFFGetFieldDefaulted(tif, TIFFTAG_PLANARCONFIG, &planar);

    if (planar != PLANARCONFIG_CONTIG)
    {
        std::cerr << "Error: Planar separate TIFF not supported: " << filename << std::endl;
        TIFFClose(tif);
        return Eigen::ArrayXXd(0, 0);
    }

    // 创建输出张量: (H, W)
    Eigen::ArrayXXd out(static_cast<int>(height), static_cast<int>(width));

    tsize_t scanline_size = TIFFScanlineSize(tif);
    std::vector<uint8_t> buf(scanline_size);

    for (uint32_t row = 0; row < height; ++row)
    {
        if (TIFFReadScanline(tif, buf.data(), row) < 0)
        {
            std::cerr << "Error: TIFFReadScanline failed at row " << row << " for file: " << filename << std::endl;
            TIFFClose(tif);
            return Eigen::ArrayXXd(0, 0);
        }

        if (sampleformat == SAMPLEFORMAT_IEEEFP && bps == 64)
        {
            const double *dptr = reinterpret_cast<const double *>(buf.data());
            for (uint32_t col = 0; col < width; ++col)
            {
                out(static_cast<int>(row), static_cast<int>(col)) = dptr[col * samples];
            }
        }
        else if (sampleformat == SAMPLEFORMAT_IEEEFP && bps == 32)
        {
            const float *fptr = reinterpret_cast<const float *>(buf.data());
            for (uint32_t col = 0; col < width; ++col)
            {
                out(static_cast<int>(row), static_cast<int>(col)) = static_cast<double>(fptr[col * samples]);
            }
        }
        else if (bps == 16)
        {
            const uint16_t *u16 = reinterpret_cast<const uint16_t *>(buf.data());
            for (uint32_t col = 0; col < width; ++col)
            {
                out(static_cast<int>(row), static_cast<int>(col)) = static_cast<double>(u16[col * samples]);
            }
        }
        else // treat as 8-bit unsigned
        {
            const uint8_t *u8 = reinterpret_cast<const uint8_t *>(buf.data());
            for (uint32_t col = 0; col < width; ++col)
            {
                out(static_cast<int>(row), static_cast<int>(col)) = static_cast<double>(u8[col * samples]);
            }
        }
    }

    TIFFClose(tif);
    return out;
}
bool SFDI::save_tiff(const std::string &filename, const Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &data)
{
    TIFF *tif = TIFFOpen(filename.c_str(), "w");
    if (!tif)
    {
        std::cerr << "Error: Unable to create TIFF file: " << filename << std::endl;
        return false;
    }

    const uint32_t width = static_cast<uint32_t>(data.cols());
    const uint32_t height = static_cast<uint32_t>(data.rows());

    // 设置 TIFF 标签
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 64);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, 0));

    // 写入扫描线
    std::vector<double> buf(width);
    for (uint32_t row = 0; row < height; ++row)
    {
        // 从 Eigen Array 复制数据到缓冲区
        std::copy(data.row(row).data(), data.row(row).data() + width, buf.begin());
        if (TIFFWriteScanline(tif, buf.data(), row) < 0)
        {
            std::cerr << "Error: TIFFWriteScanline failed at row " << row << " for file: " << filename << std::endl;
            TIFFClose(tif);
            return false;
        }
    }

    TIFFClose(tif);
    return true;
}
void SFDI::Compute_Amplitude_Envelope(const Eigen::ArrayXXd &inputp0,
                                      const Eigen::ArrayXXd &inputp120,
                                      const Eigen::ArrayXXd &inputp240,
                                      const double int_time,
                                      Eigen::ArrayXXd &out)
{
    ScopedTimer timer("[Model] Compute_Amplitude_Envelope");

    const auto sz = inputp0.size();
    if (inputp120.size() != sz || inputp240.size() != sz)
    {
        throw std::invalid_argument("Input arrays have different sizes");
    }
    if (inputp0.rows() != out.rows() || inputp0.cols() != out.cols())
    {
        out.resize(inputp0.rows(), inputp0.cols());
    }

    const double sqrt2_div_3 = std::sqrt(2.0) / 3.0;
    const double scale = sqrt2_div_3 / int_time;

#pragma omp parallel for
    for (int row = 0; row < out.rows(); ++row)
    {
        for (int col = 0; col < out.cols(); ++col)
        {
            const double diff0 = inputp0(row, col) - inputp120(row, col);
            const double diff1 = inputp120(row, col) - inputp240(row, col);
            const double diff2 = inputp240(row, col) - inputp0(row, col);
            out(row, col) = scale * std::sqrt(diff0 * diff0 + diff1 * diff1 + diff2 * diff2);
        }
    }
}
SFDI::mc_model::mc_model(const std::string &R_of_rho_time_mc_path)
{
    std::call_once(load_flag, load_R_of_rho_time, R_of_rho_time_mc_path);
    Jterm.resize(SFDI::FREQ_NUM, SFDI::RHO_BIN);
    v_t.resize(SFDI::TIME_BIN);
    twopi_rho_drho.resize(SFDI::RHO_BIN);
    twopi_rho_drho = delta_rho * two_pi * rho_var;
    setN(1.4);
    SFDI::Freq freq;
    freq << 0.0, 0.1;
    setFrequency(freq);
    Reflect dst;
    mc_model_for_SFDI(0.01, 1.0, dst);
    mc_model_for_SFDI_Dmua(0.01, 1.0, dst);
    mc_model_for_SFDI_Dmusp(0.01, 1.0, dst);
}
void SFDI::mc_model::setN(const double input_n)
{
    constexpr double mc_f = 1.0 - ((1 - origin_n) / (1 + origin_n)) * ((1 - origin_n) / (1 + origin_n));
    if (n != input_n)
    {
        v = light_speed / input_n; // 光速除以折射率
        n = input_n;
        double F = 1 - ((1 - input_n) / (1 + input_n)) * ((1 - input_n) / (1 + input_n));
        delta_t_div_fresnel = delta_t * mc_f / F;
        v_t = v * time_var; // (W,T) 预计算 v*t
    }
}
void SFDI::mc_model::setFrequency(const Freq &freq_input)
{
    if (!frequency.isApprox(freq_input))
    {
        frequency = freq_input;
        Jterm = two_pi * (frequency.matrix() * rho_var.matrix().transpose()).array(); // (F,R) 预计算 2*pi*f*rho F
    }
}
void SFDI::mc_model::mc_model_for_SFDI(const double mua, const double musp, SFDI::Reflect &dst) const
{
    // 预计算倒数
    const double musp_inv = 1.0 / musp;
    // (T,1) * (T, R) -> (R)
    auto decay = (-(v_t * (mua * musp_inv))).exp(); // exp(-v * t * mua / musp )
    // decay.matrix().transpose() 是 (1, T), R_of_rho_time_mc.matrix() 是 (T, R), 乘积是 (1, R)
    // 需要转置为 (R,) 列向量以匹配 twopi_rho_drho 的维度
    auto R_rho = (decay.matrix().transpose() * R_of_rho_time_mc.matrix()).array(); // R_rho = R_of_rho_time_mc /F * decay *dt/musp *musp^3
    // 构建积分权重 term_noj:  R_rho * twopi_rho_drho  / musp^2
    auto term_noj = R_rho.transpose() * twopi_rho_drho * delta_t_div_fresnel;
    // 执行汉克尔变换(沿 R 轴求和)
    // bessel_j0(Jterm * musp_inv) -> (F, R), term_noj -> (R,1)
    dst = bessel_j0(Jterm * musp_inv).matrix() * term_noj.matrix(); // 对 R 轴求和
}
void SFDI::mc_model::mc_model_for_SFDI_Dmua(const double mua, const double musp, SFDI::Reflect &dst) const
{
    // 预计算倒数
    const double musp_inv = 1.0 / musp;
    auto v_t_musp_inv = -(v_t * musp_inv);                       // (T,1) 预计算 -v*t/musp
    auto decay_dmua = v_t_musp_inv * (v_t_musp_inv * mua).exp(); // exp(-v * t * mua / musp )
    // (1,T) * (T, R) -> (1,R)
    auto R_rho_dmua = (decay_dmua.matrix().transpose() * R_of_rho_time_mc.matrix()).array(); // R_rho_dmua = R_of_rho_time_mc /F * decay *dt/musp *musp^3 * v*t/musp
    // 构建积分权重 term_noj:  R_rho * twopi_rho_drho  / musp^2
    auto term_noj = R_rho_dmua.transpose() * twopi_rho_drho * delta_t_div_fresnel; //(R,1)
    // 执行汉克尔变换(沿 R 轴求和)
    dst = bessel_j0(Jterm * musp_inv).matrix() * term_noj.matrix(); // 对 R 轴求和
}
void SFDI::mc_model::mc_model_for_SFDI_Dmusp(const double mua, const double musp, SFDI::Reflect &dst) const
{
    // 预计算倒数
    const double musp_inv = 1.0 / musp;
    auto v_t_musp_inv_mua = -(v_t * musp_inv * mua);
    auto decay = v_t_musp_inv_mua.exp();
    auto decay_dmusp = -v_t_musp_inv_mua * decay * musp_inv; // exp(-v * t * mua / musp ) * v * t * mua / musp**2
    // (1,T) * (T, R) -> (1,R)
    // decay_dmusp.matrix().transpose() 是 (1, T), R_of_rho_time_mc.matrix() 是 (T, R), 乘积是 (1, R)
    // 需要转置为 (R,) 列向量
    const Eigen::ArrayXd R_rho_dmusp = (decay_dmusp.matrix().transpose() * R_of_rho_time_mc.matrix()).array() * delta_t_div_fresnel; // R_rho_dmua = R_of_rho_time_mc /F * decay *dt/musp *musp^3 * v*t/musp
    // 构建积分权重 term_noj:  R_rho * twopi_rho_drho  / musp^2
    const Eigen::ArrayXd R_rho = (decay.matrix().transpose() * R_of_rho_time_mc.matrix()).array() * delta_t_div_fresnel;
    const Eigen::ArrayXXd Jterm_musp_inv = Jterm * musp_inv;
    // 先构造两个权重向量，再用矩阵乘法（(F,R) * (R,1) -> (F,1)）
    auto J0 = bessel_j0(Jterm_musp_inv);
    auto J1 = bessel_j1(Jterm_musp_inv) * Jterm_musp_inv;
    auto w0 = R_rho_dmusp * twopi_rho_drho;      // (R)
    auto w1 = R_rho * twopi_rho_drho * musp_inv; // (R)

    dst = J0.matrix() * w0.matrix() + J1.matrix() * w1.matrix(); // (F,1)
}
