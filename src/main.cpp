#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACK
#include "model_SFDI.hpp"
#include <fstream>
#include <omp.h>
typedef struct
{
    double mua;             // 吸收系数
    double musp;            // 约化散射系数
    SFDI::SFDI_Model model; // 计算结果 (WAVELENGTH_NUM × FREQ_NUM)
} SFDI_Result;
int main(void)
{
#pragma omp parallel
    {
#pragma omp single
        std::cout << "Threads = " << omp_get_num_threads() << std::endl;
    }
    constexpr int N_MUA = 1000;  // mua取100个点
    constexpr int N_MUSP = 1000; // musp取100个点
    constexpr int N_TOTAL = N_MUA * N_MUSP;

    std::vector<SFDI_Result> results(N_TOTAL);
    Eigen::ArrayXd mua_values = Eigen::ArrayXd::LinSpaced(N_MUA, 1e-5, 0.3);
    Eigen::ArrayXd musp_values = Eigen::ArrayXd::LinSpaced(N_MUSP, 0.5, 1.5);
    SFDI::model_SFDI comp("reference_670", "sample_670", "ROfRhoAndTime");
    comp.setN(SFDI::Optical_prop().setConstant(1.37));
    SFDI::Freq freq;
    freq << 0, 0.2;
    comp.setFrequency(freq);
#pragma omp parallel for
    for (int idx = 0; idx < N_TOTAL; ++idx)
    {
        int i = idx / N_MUSP; // mua 索引
        int j = idx % N_MUSP; // musp 索引
        results[idx].mua = mua_values[i];
        results[idx].musp = musp_values[j];
        SFDI::Optical_prop mua_prop;
        SFDI::Optical_prop musp_prop;
        mua_prop.setConstant(mua_values[i]);
        musp_prop.setConstant(musp_values[j]);
        results[idx].model = comp.mc_model_for_SFDI(
            mua_prop,
            musp_prop);
    }
    std::ofstream ofs("sfdi_results.bin", std::ios::binary);
    for (const auto &r : results)
    {
        ofs.write(reinterpret_cast<const char *>(&r.mua), sizeof(double));
        ofs.write(reinterpret_cast<const char *>(&r.musp), sizeof(double));
        ofs.write(reinterpret_cast<const char *>(r.model.data()),
                  sizeof(double) * r.model.size());
    }
    ofs.close();
    return 0;
}