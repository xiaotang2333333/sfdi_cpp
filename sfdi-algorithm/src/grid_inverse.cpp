#include "grid_inverse.hpp"
#include <nlopt.hpp>
#include <fstream>
#include <iostream>

using namespace SFDI;
namespace
{
    constexpr double mua_min = 1e-5, mua_max = 1, musp_min = 1e-5, musp_max = 5;
    struct Ctx
    {
        const mc_model *model;
        SFDI::Reflect target;
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
                Reflect reflect = Reflect::Zero();
                reflect(0) = rdc;
                reflect(1) = rac;
                grid_results.push_back({0, 0, reflect});
            }
        }
    }
}

void GridInverseSolver::solve(const SFDI::mc_model &model, const Reflect &target, double &dst_mua, double &dst_musp) const
{
    constexpr int F = SFDI::FREQ_NUM;
    int n_params = 2; // mua 和 musp 各一个参数
    std::vector<double> lb(n_params), ub(n_params), x0(n_params);
    lb[0] = mua_min;
    ub[0] = mua_max;
    lb[1] = musp_min;
    ub[1] = musp_max;
    x0[0] = 0.1;
    x0[1] = 2.0;
    nlopt::opt opt(nlopt::LD_LBFGS, n_params);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    Ctx ctx = {&model, target};
    opt.set_min_objective(
        [](const std::vector<double> &x, std::vector<double> &grad, void *data) -> double
        {
            auto *ctx = static_cast<Ctx *>(data);
            constexpr int F = SFDI::FREQ_NUM;

            double mua, musp;
            mua = x[0];
            musp = x[1];
            // 基准前向
            SFDI::Reflect pred;
            ctx->model->mc_model_for_SFDI(mua, musp, pred);

            SFDI::Reflect residual = (pred - ctx->target).array();
            double loss = residual.square().sum();

            if (!grad.empty())
            {
                SFDI::Reflect FDmua, FDmusp;
                ctx->model->mc_model_for_SFDI_Dmua(mua, musp, FDmua);
                ctx->model->mc_model_for_SFDI_Dmusp(mua, musp, FDmusp);
                double dL_dmua = 0.0;
                double dL_dmusp = 0.0;
                for (int f = 0; f < F; ++f)
                {
                    double r = pred(f) - ctx->target(f);
                    dL_dmua += 2 * r * FDmua(f);
                    dL_dmusp += 2 * r * FDmusp(f);
                }
                grad[0] = dL_dmua;
                grad[1] = dL_dmusp;
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
    dst_mua = x0[0];
    dst_musp = x0[1];
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
    for (const SFDI::SFDI_Result &item : grid_results)
    {
        fout.write(reinterpret_cast<const char *>(&item.mua), sizeof(double));
        fout.write(reinterpret_cast<const char *>(&item.musp), sizeof(double));
        fout.write(reinterpret_cast<const char *>(item.model.data()), sizeof(double) * SFDI::FREQ_NUM);
    }
    fout.close();
}
