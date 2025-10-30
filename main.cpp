#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACK
#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <nlopt.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/CXX11/Tensor>
#define IMG_HEIGHT 512
#define IMG_WIDTH 672
#define WAVELENGTH_NUM 1
#define PHASE_NUM 3
#define FREQ_NUM 2
typedef struct
{
    int nowheight;
    int nowwidth;
} ImageInfo;
using Tiff_img = Eigen::Tensor<double, 3, Eigen::RowMajor>;
using SFDI_data = Eigen::TensorFixedSize<
    double,
    Eigen::Sizes<IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, PHASE_NUM, FREQ_NUM>,
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
    Eigen::RowMajor>;                                              // 一维数组 包含每个波长的积分时间
using Optical_prop = Eigen::Array<double, WAVELENGTH_NUM, 1>;      // 光学特性 与 波长相关
using Freq = Eigen::Array<double, FREQ_NUM, 1>;                    // 固定长度一维向量
using SFDI_Model = Eigen::Array<double, WAVELENGTH_NUM, FREQ_NUM>; // 固定大小二维数组
static SFDI_data ref_data, sample_data;
static SFDI_AC ref_ac_data, sample_ac_data, calibrated_reflectance;
static Int_time int_time;
static Optical_prop ref_mua = (Optical_prop() << 0.0059).finished(),
                    ref_musp = (Optical_prop() << 0.9748).finished(),
                    ref_n = (Optical_prop() << 1.37).finished(),
                    mua_guess, musp_guess;
static Optical_prop_map mua_map, musp_map;
static Freq freq = (Freq() << 0, 0.2).finished();
static SFDI_Model ref_model;
static Tiff_img open_tiff(const std::string &filename)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_UNCHANGED);
    if (img.empty())
    {
        std::cerr << "Error: Unable to open image file: " << filename << std::endl;
        return Tiff_img(0, 0, 0);
    }

    // Ensure the image is 16-bit unsigned integer
    if (img.depth() != CV_16U)
    {
        img.convertTo(img, CV_16U);
    }
    img.convertTo(img, CV_64F);
    // img /= 65535.0;
    Tiff_img temp;
    cv::cv2eigen(img, temp);
    return temp;
}
static void Compute_AC(const SFDI_data &input, const Int_time &int_time, SFDI_AC &output)
{
    // 预计算常量，避免重复计算
    const double sqrt_2_over_3 = std::sqrt(2.0) / 3.0;
    
    // 优化内存访问模式
    Eigen::array<Eigen::Index, 4> reshape_dims = {1, 1, WAVELENGTH_NUM, 1};
    Eigen::array<Eigen::Index, 4> broadcast_dims = {IMG_HEIGHT, IMG_WIDTH, 1, FREQ_NUM};
    auto int_time_bcast = int_time.reshape(reshape_dims).broadcast(broadcast_dims);
    
    // 优化：减少中间计算步骤
    auto img_0 = input.chip(0, 3);
    auto img_120 = input.chip(1, 3);
    auto img_240 = input.chip(2, 3);
    
    // 直接计算最终结果，避免不必要的临时对象
    auto numerator = (img_0 - img_120).square() +
                      (img_0 - img_240).square() +
                      (img_120 - img_240).square();
    
    output.device(Eigen::DefaultDevice()) =
    (numerator.sqrt() * sqrt_2_over_3 / int_time_bcast);
}
/// @brief SFDI模型 mua musp n 与波长数量有关
/// @param mua 1维向量表示每个波长吸收系数
/// @param musp 1维向量表示每个波长约化散射系数
/// @param n 1维向量表示每个波长的折射率
/// @param frequency 1维向量表示每个频率
static SFDI_Model diff_model_for_SFDI(const Optical_prop mua, const Optical_prop musp, const Optical_prop n, const Freq frequency)
{

    auto mutrans = mua + musp;
    auto Reff = -1.440 / n.square() + 0.710 / n + 0.668 + 0.0636 * n;
    auto A = (1 - Reff) / (2 * (1 + Reff));
    auto term1 = 3.0 * mua * mutrans;               // shape (W)
    auto term2 = (2.0 * M_PI * frequency).square(); // shape (F)
    SFDI_Model mueff_prime =
        term2.transpose().template replicate<WAVELENGTH_NUM, 1>(); // (W,F)
    mueff_prime.colwise() += term1;                                // 广播 term1 到每一列
    mueff_prime = mueff_prime.sqrt();                              // shape (W,F)
    Optical_prop threeA = 3.0 * A;                                 // (W,1)
    Optical_prop num_col = threeA * musp * mutrans;                // (W,1)
    SFDI_Model denom1 = mueff_prime;                               // (W,F)
    denom1.colwise() += mutrans;                                   // mueff + mutrans
    SFDI_Model denom2 = mueff_prime;                               // (W,F)
    denom2.colwise() += threeA * mutrans;                          // mueff + 3*A*mutrans
    SFDI_Model reflectance_fx =
        num_col.template replicate<1, FREQ_NUM>() / (denom1 * denom2); // (W,F)
    return reflectance_fx;
}
static Eigen::Map<const SFDI_Model> AC2Model(const SFDI_AC &ac, int h, int w)
{
    const double *ptr = ac.data();
    size_t offset = ((h * IMG_WIDTH) + w) * WAVELENGTH_NUM * FREQ_NUM;
    return Eigen::Map<const SFDI_Model>(ptr + offset);
}
double objective(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
    // x 包含 2*WAVELENGTH_NUM 个元素：前半为 mua，后半为 musp
    const double *ptr = x.data();
    auto *data = static_cast<ImageInfo *>(f_data);

    Eigen::Map<const Optical_prop> mua_view(ptr);
    Eigen::Map<const Optical_prop> mus_view(ptr + WAVELENGTH_NUM);

    // 前向：模型反射率
    SFDI_Model result = diff_model_for_SFDI(mua_view, mus_view, ref_n, freq);
    double fval = (result - AC2Model(calibrated_reflectance, data->nowheight, data->nowwidth)).square().sum();
    return fval; // 目标值
}
static std::vector<double> run_nlopt(ImageInfo *data)
{
    const int dim = 2 * WAVELENGTH_NUM; // 参数维度：mua + musp
    std::vector<double> result_params(dim);
    
    // 优化：使用更高效的优化算法
    // LN_BOBYQA 通常比 LN_NELDERMEAD 收敛更快
    nlopt::opt opt(nlopt::LN_BOBYQA, dim);

    // 2. 设置上下界
    std::vector<double> lb(dim), ub(dim);
    for (int i = 0; i < WAVELENGTH_NUM; ++i)
    {
        lb[i] = 1e-5;                 // mua下界
        ub[i] = 0.5;                  // mua上界
        lb[i + WAVELENGTH_NUM] = 0.2; // musp下界
        ub[i + WAVELENGTH_NUM] = 4.0; // musp上界
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // 3. 设置目标函数
    opt.set_min_objective(objective, data);

    // 优化：调整终止条件，提高收敛速度
    opt.set_xtol_rel(1e-4);  // 放宽参数容忍度
    opt.set_ftol_rel(1e-6);  // 保持函数值容忍度
    opt.set_maxeval(500); // 减少最大迭代次数
    opt.set_maxtime(30.0); // 增加时间限制

    // 5. 初始值 - 优化初始猜测
    std::vector<double> x0(dim);
    for (int i = 0; i < WAVELENGTH_NUM; ++i)
    {
        x0[i] = 0.01;                 // 改进初始 mua
        x0[i + WAVELENGTH_NUM] = 0.8; // 改进初始 musp
    }

    // 6. 运行优化
    double minf;
    try
    {
        nlopt::result result = opt.optimize(x0, minf);
        for (int i = 0; i < dim; ++i)
    {
        result_params[i] = x0[i];
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return result_params;
}
int main(void)
{
    int_time.setConstant(1.0f);
    std::string ref_folder = "reference_670";
    std::string sample_folder = "sample_670";
    Tiff_img imgref_0hz_0phase = open_tiff(ref_folder + "/im01.tif"),
             imgref_0hz_120phase = open_tiff(ref_folder + "/im02.tif"),
             imgref_0hz_240phase = open_tiff(ref_folder + "/im03.tif"),
             imgref_02hz_0phase = open_tiff(ref_folder + "/im04.tif"),
             imgref_02hz_120phase = open_tiff(ref_folder + "/im05.tif"),
             imgref_02hz_240phase = open_tiff(ref_folder + "/im06.tif"); //(H,W,C)

    // auto toTensor = [&](const Eigen::MatrixXd &mat)
    // { return Eigen::TensorMap<const Eigen::Tensor<double, 2>>(mat.data(), H, W); };
    Eigen::array<Eigen::Index, 5> extents = {
        IMG_HEIGHT, IMG_WIDTH, WAVELENGTH_NUM, 1, 1};
    Eigen::array<Eigen::Index, 5> offsets_0hz_0phase = {0, 0, 0, 0, 0};
    ref_data.slice(offsets_0hz_0phase, extents) =
        imgref_0hz_0phase.reshape(extents); // 使用 reshape 匹配维度
    Eigen::array<Eigen::Index, 5> offsets_0hz_120phase = {0, 0, 0, 1, 0};
    ref_data.slice(offsets_0hz_120phase, extents) =
        imgref_0hz_120phase.reshape(extents);

    Eigen::array<Eigen::Index, 5> offsets_0hz_240phase = {0, 0, 0, 2, 0};
    ref_data.slice(offsets_0hz_240phase, extents) =
        imgref_0hz_240phase.reshape(extents);
    Eigen::array<Eigen::Index, 5> offsets_02hz_0phase = {0, 0, 0, 0, 1};
    ref_data.slice(offsets_02hz_0phase, extents) =
        imgref_02hz_0phase.reshape(extents);

    Eigen::array<Eigen::Index, 5> offsets_02hz_120phase = {0, 0, 0, 1, 1};
    ref_data.slice(offsets_02hz_120phase, extents) =
        imgref_02hz_120phase.reshape(extents);

    Eigen::array<Eigen::Index, 5> offsets_02hz_240phase = {0, 0, 0, 2, 1};
    ref_data.slice(offsets_02hz_240phase, extents) =
        imgref_02hz_240phase.reshape(extents);
    Tiff_img imgsmaple_0hz_0phase = open_tiff(sample_folder + "/im01.tif"),
             imgsmaple_0hz_120phase = open_tiff(sample_folder + "/im02.tif"),
             imgsmaple_0hz_240phase = open_tiff(sample_folder + "/im03.tif"),
             imgsmaple_02hz_0phase = open_tiff(sample_folder + "/im04.tif"),
             imgsmaple_02hz_120phase = open_tiff(sample_folder + "/im05.tif"),
             imgsmaple_02hz_240phase = open_tiff(sample_folder + "/im06.tif"); //(H,W,C)
    sample_data.slice(offsets_0hz_0phase, extents) = imgsmaple_0hz_0phase.reshape(extents);
    sample_data.slice(offsets_0hz_120phase, extents) = imgsmaple_0hz_120phase.reshape(extents);
    sample_data.slice(offsets_0hz_240phase, extents) = imgsmaple_0hz_240phase.reshape(extents);
    sample_data.slice(offsets_02hz_0phase, extents) = imgsmaple_02hz_0phase.reshape(extents);
    sample_data.slice(offsets_02hz_120phase, extents) = imgsmaple_02hz_120phase.reshape(extents);
    sample_data.slice(offsets_02hz_240phase, extents) = imgsmaple_02hz_240phase.reshape(extents);
    Compute_AC(sample_data, int_time, sample_ac_data);
    Compute_AC(ref_data, int_time, ref_ac_data);
    ref_model = diff_model_for_SFDI(ref_mua, ref_musp, ref_n, freq);

    //  将 ref_model(C,F) 扩展成 (H,W,C,F)
    // （1）用 TensorMap 包装 ref_model 数据
    Eigen::TensorMap<const Eigen::Tensor<double, 2, Eigen::RowMajor>>
        ref_model_2d(ref_model.data(), WAVELENGTH_NUM, FREQ_NUM);
    // （2）reshape + broadcast 得到 4维 Tensor
    Eigen::array<Eigen::Index, 4> rm_shape = {1, 1, WAVELENGTH_NUM, FREQ_NUM};
    Eigen::array<Eigen::Index, 4> rm_bcast = {IMG_HEIGHT, IMG_WIDTH, 1, 1};

    // （3）赋值回固定大小 Tensor
    auto ref_model_expr = ref_model_2d.reshape(rm_shape).broadcast(rm_bcast);
    calibrated_reflectance.device(Eigen::DefaultDevice()) =
        sample_ac_data / ref_ac_data * ref_model_expr;

    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();

#pragma omp parallel for collapse(2)
    for (int nowheight = 0; nowheight < IMG_HEIGHT; ++nowheight)
    {
        for (int nowwidth = 0; nowwidth < IMG_WIDTH; ++nowwidth)
        {
            ImageInfo data = {
                .nowheight = nowheight,
                .nowwidth = nowwidth};
            std::vector<double> params = run_nlopt(&data);
            Eigen::Map<const Optical_prop> mua_opt(params.data());
            Eigen::Map<const Optical_prop> musp_opt(params.data() + WAVELENGTH_NUM);

            for (int c = 0; c < WAVELENGTH_NUM; ++c)
            {
                mua_map(nowheight, nowwidth, c) = mua_opt(c);
                musp_map(nowheight, nowwidth, c) = musp_opt(c);
            }
        }
    }

    // 结束计时并输出运行时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "并行循环执行时间: " << duration.count() << "秒" << std::endl;

    std::ofstream file("sfdi_mua.bin", std::ios::binary);
    if (!file)
    {
        std::cerr << "无法打开文件！\n";
        return 1;
    }
    // 写入原始内存数据
    file.write(reinterpret_cast<const char *>(mua_map.data()),
               sizeof(double) * mua_map.size());
    file.close();
    return 0;
}
