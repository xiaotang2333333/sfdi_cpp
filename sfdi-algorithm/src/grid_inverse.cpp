#include "grid_inverse.hpp"
#include <nlopt.hpp>
#include <fstream>
#include <iostream>

using namespace SFDI;
namespace
{
    constexpr double mua_min = 1e-5, mua_max = 5, musp_min = 1e-5, musp_max = 25;
    struct Ctx
    {
        const mc_model *model;
        SFDI::Reflect_wave_freq target;
    };
}
GridInverseSolver::GridInverseSolver(int rdc_n_, int rac_n_, double rdc_min_, double rdc_max_, double rac_min_, double rac_max_)
    : rdc_n(rdc_n_), rac_n(rac_n_), rdc_min(rdc_min_), rdc_max(rdc_max_), rac_min(rac_min_), rac_max(rac_max_)
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
            // 仅保留 RDC > RAC 的组合（结构光中 RAC 必然小于 RDC）
            if (rdc > rac)
            {
                Reflect_wave_freq reflect = Reflect_wave_freq::Zero();
                for (int c = 0; c < W; ++c)
                {
                    reflect(c, 0) = rdc;
                    reflect(c, 1) = rac;
                }
                Optical_prop mua = Optical_prop::Zero();
                Optical_prop musp = Optical_prop::Zero();
                grid_results.push_back({mua, musp, reflect});
            }
        }
    }
}

void GridInverseSolver::solve(const SFDI::mc_model &model, const Reflect_wave_freq &target, Optical_prop &dst_mua, Optical_prop &dst_musp) const
{
    constexpr int W = SFDI::WAVELENGTH_NUM;
    constexpr int F = SFDI::FREQ_NUM;
    int n_params = 2 * W; // mua 和 musp 每个波长各一个
    std::vector<double> lb(n_params), ub(n_params), x0(n_params);
    for (int c = 0; c < W; ++c)
    {
        lb[2 * c] = mua_min;
        ub[2 * c] = mua_max;
        lb[2 * c + 1] = musp_min;
        ub[2 * c + 1] = musp_max;
        x0[2 * c] = 0.1;
        x0[2 * c + 1] = 2.0;
    }
    nlopt::opt opt(nlopt::LD_LBFGS, n_params);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    Ctx ctx = {&model, target};
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

            // 基准前向
            SFDI::Reflect_wave_freq pred;
            ctx->model->mc_model_for_SFDI(mua, musp, pred);

            Eigen::Array<double, W, F> residual = (pred - ctx->target).array();
            double loss = residual.square().sum();

            // 下面的梯度函数已经测试了，速度很慢
            if (!grad.empty())
            {
                SFDI::Reflect_wave_freq FDmua, FDmusp;
                ctx->model->mc_model_for_SFDI_Dmua(mua, musp, FDmua);
                ctx->model->mc_model_for_SFDI_Dmusp(mua, musp, FDmusp);
                for (int c = 0; c < W; ++c)
                {
                    double dL_dmua = 0.0;
                    double dL_dmusp = 0.0;
                    for (int f = 0; f < F; ++f)
                    {
                        double r = pred(c, f) - ctx->target(c, f);
                        dL_dmua += 2 * r * FDmua(c, f);
                        dL_dmusp += 2 * r * FDmusp(c, f);
                    }
                    grad[2 * c] = dL_dmua;
                    grad[2 * c + 1] = dL_dmusp;
                }
            }

            return loss;
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
    const int total = static_cast<int>(grid_results.size());
    SFDI::mc_model model_comp;
#pragma omp parallel for
    for (int i = 0; i < total; ++i)
    {
        solve(model_comp, grid_results[i].model, grid_results[i].mua, grid_results[i].musp);
    }
    for (const auto &item : grid_results)
    {
        fout.write(reinterpret_cast<const char *>(item.mua.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.musp.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.model.data()), sizeof(double) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM);
    }
    fout.close();
}
