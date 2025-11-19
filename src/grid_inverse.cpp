#include "grid_inverse.hpp"
#include <nlopt.hpp>
#include <fstream>
#include <limits>
#include <iostream>

using namespace SFDI;

GridInverseSolver::GridInverseSolver(model_SFDI &model_, int rdc_n_, int rac_n_, double rdc_min_, double rdc_max_, double rac_min_, double rac_max_)
    : model(model_), rdc_n(rdc_n_), rac_n(rac_n_), rdc_min(rdc_min_), rdc_max(rdc_max_), rac_min(rac_min_), rac_max(rac_max_)
{
    build_grid();
}

void GridInverseSolver::build_grid()
{
    // 生成Reflect_wave_freq空间的均匀网格，所有波长都做RDC/RAC均匀网格
    grid_results.clear();
    constexpr int W = SFDI::WAVELENGTH_NUM;
    constexpr int F = SFDI::FREQ_NUM;
    for (int i = 0; i < rdc_n; ++i)
    {
        double rdc = rdc_min + (rdc_max - rdc_min) * i / (rdc_n - 1);
        for (int j = 0; j < rac_n; ++j)
        {
            double rac = rac_min + (rac_max - rac_min) * j / (rac_n - 1);
            Reflect_wave_freq reflect = Reflect_wave_freq::Zero();
            for (int c = 0; c < W; ++c)
            {
                reflect(c, 0) = rdc;
                reflect(c, 1) = rac;
            }
            Optical_prop mua = Optical_prop::Zero();
            Optical_prop musp = Optical_prop::Zero();
            // 仅保留 RDC > RAC 的组合（结构光中 RAC 必然小于 RDC）
            if (rdc > rac) {
                grid_results.push_back({mua, musp, reflect});
            }
        }
    }
}

void GridInverseSolver::solve(const Reflect_wave_freq &target, Optical_prop &dst_mua, Optical_prop &dst_musp) const
{
    constexpr int W = SFDI::WAVELENGTH_NUM;
    constexpr int F = SFDI::FREQ_NUM;
    int n_params = 2 * W;
    std::vector<double> lb(n_params), ub(n_params), x0(n_params);
    double mua_min = 1e-5, mua_max = 2, musp_min = 1e-5, musp_max = 15.0;
    for (int c = 0; c < W; ++c)
    {
        lb[2 * c] = mua_min;
        ub[2 * c] = mua_max;
        lb[2 * c + 1] = musp_min;
        ub[2 * c + 1] = musp_max;
        x0[2 * c] = (mua_min + mua_max) / 2;
        x0[2 * c + 1] = (musp_min + musp_max) / 2;
    }
    nlopt::opt opt(nlopt::LN_BOBYQA, n_params);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    struct Ctx
    {
        model_SFDI *model;
        const Reflect_wave_freq *target;
    } ctx = {&model, &target};
    opt.set_min_objective(
        [](const std::vector<double> &x, std::vector<double> &grad, void *data) -> double
        {
            auto *ctx = static_cast<Ctx *>(data);
            constexpr int W = SFDI::WAVELENGTH_NUM;
            constexpr int F = SFDI::FREQ_NUM;
            SFDI::Optical_prop mua, musp;
            for (int c = 0; c < W; ++c)
            {
                mua(c) = x[2 * c];
                musp(c) = x[2 * c + 1];
            }
            SFDI::Reflect_wave_freq pred;
            ctx->model->mc_model_for_SFDI(mua, musp, pred);
            return (pred - *(ctx->target)).square().sum();
        },
        &ctx);
    double minf = 0.0;
    try
    {
        opt.optimize(x0, minf);
    }
    catch (std::exception &e)
    {
        std::cerr << "NLOPT failed: " << e.what() << std::endl;
    }
    for (int c = 0; c < W; ++c)
    {
        dst_mua(c) = x0[2 * c];
        dst_musp(c) = x0[2 * c + 1];
    }
}

void GridInverseSolver::solve_and_save(const std::string &output_bin)
{
    std::ofstream fout(output_bin, std::ios::binary);
    if (!fout)
    {
        std::cerr << "Failed to open output file: " << output_bin << std::endl;
        return;
    }
#pragma omp parallel for
    for (size_t i = 0; i < grid_results.size(); ++i)
    {
        solve(grid_results[i].reflect, grid_results[i].mua, grid_results[i].musp);
    }
    for (const auto &item : grid_results)
    {
        fout.write(reinterpret_cast<const char *>(item.mua.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.musp.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.reflect.data()), sizeof(double) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM);
    }
    fout.close();
}
