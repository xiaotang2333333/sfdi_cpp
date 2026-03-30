#include "grid_forward.hpp"

SFDI::GridForwardSolver::GridForwardSolver(SFDI::mc_model &model, int mua_n, int musp_n, double mua_min, double mua_max, double musp_min, double musp_max)
    : model(model), mua_n(mua_n), musp_n(musp_n), mua_min(mua_min), mua_max(mua_max), musp_min(musp_min), musp_max(musp_max)
{
    build_grid();
}
void SFDI::GridForwardSolver::build_grid()
{
    // 生成Reflect_wave_freq空间的均匀网格，所有波长都做mua/musp均匀网格

    grid_results.clear();
    grid_results.resize(mua_n * musp_n);
    constexpr int F = SFDI::FREQ_NUM;
    int total = mua_n * musp_n;
#pragma omp parallel for
    for (int k = 0; k < total; ++k)
    {
        int i = k / musp_n;
        int j = k % musp_n;
        double fi = (mua_n > 1) ? static_cast<double>(i) / (mua_n - 1) : 0.0;
        double fj = (musp_n > 1) ? static_cast<double>(j) / (musp_n - 1) : 0.0;
        double mua = mua_min + (mua_max - mua_min) * fi;
        double musp = musp_min + (musp_max - musp_min) * fj;

        Reflect reflect = Reflect::Zero();
        model.mc_model_for_SFDI(mua, musp, reflect);
        grid_results[k] = {mua, musp, reflect};
    }
}
void SFDI::GridForwardSolver::compute_and_save(const std::string &output_bin)
{
    std::ofstream ofs(output_bin, std::ios::binary);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + output_bin);
    }

    for (const auto &result : grid_results)
    {
        if((result.model(0)<0.01||result.model(1)<0.01)||result.model(0)>1.0||result.model(1)>1.0)
        {
            std::cout<<"Warning: FALSE reflectance at mua="<<result.mua<<", musp="<<result.musp<<" RDC="<<result.model(0)<<", RAC="<<result.model(1)<<std::endl;
        }
        ofs.write(reinterpret_cast<const char *>(&result.mua), sizeof(double));
        ofs.write(reinterpret_cast<const char *>(&result.musp), sizeof(double));
        ofs.write(reinterpret_cast<const char *>(result.model.data()), sizeof(double) * result.model.size());
    }
    ofs.close();
}