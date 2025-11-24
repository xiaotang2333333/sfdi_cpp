#include "grid_inverse.hpp"
#include <nlopt.hpp>
#include <fstream>
#include <limits>
#include <iostream>

using namespace SFDI;
namespace
{
    constexpr double mua_min = 1e-5, mua_max = 5.0, musp_min = 1e-5, musp_max = 25.0;
}
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
            // 仅保留 RDC > RAC 的组合（结构光中 RAC 必然小于 RDC）
            if (rdc > rac)
            {
                Reflect_freq reflect = Reflect_freq::Zero();
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

void GridInverseSolver::solve(const Reflect_freq &target, Optical_prop &dst_mua, Optical_prop &dst_musp) const
{
    constexpr int W = SFDI::WAVELENGTH_NUM;
    constexpr int F = SFDI::FREQ_NUM;
    int n_params = F * W;
    std::vector<double> lb(n_params), ub(n_params), x0(n_params);
    for (int c = 0; c < W; ++c)
    {
        lb[2 * c] = mua_min;
        ub[2 * c] = mua_max;
        lb[2 * c + 1] = musp_min;
        ub[2 * c + 1] = musp_max;
        x0[2 * c] = 1.0;
        x0[2 * c + 1] = 5.0;
    }
    nlopt::opt opt(nlopt::LN_BOBYQA, n_params);
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    struct Ctx
    {
        model_SFDI *model;
        SFDI::Reflect_freq target;
    } ctx = {&model, target.replicate(1, F).eval()};
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
            SFDI::Reflect_freq pred;
            ctx->model->mc_model_for_SFDI(mua, musp, pred);

            Eigen::Array<double, W, F> residual = (pred - ctx->target).array();
            double loss = residual.square().sum();
            
            
            // 下面的梯度函数已经测试了，速度很慢
        
            if (!grad.empty())
            {
                grad.assign(2 * W, 0.0);

                // 步长初值
                SFDI::Optical_prop h_mua, h_musp;
                for (int c = 0; c < W; ++c)
                {
                    h_mua(c) = std::max(1e-4 * std::max(mua(c), mua_min), 1e-7);
                    h_musp(c) = std::max(1e-4 * std::max(musp(c), musp_min), 1e-7);
                }

                // 构造全局扰动向量
                SFDI::Optical_prop mua_plus = mua;
                SFDI::Optical_prop mua_minus = mua;
                SFDI::Optical_prop musp_plus = musp;
                SFDI::Optical_prop musp_minus = musp;

                // 实际步长（考虑边界裁剪）
                SFDI::Optical_prop h_mua_plus, h_mua_minus;
                SFDI::Optical_prop h_musp_plus, h_musp_minus;

                for (int c = 0; c < W; ++c)
                {
                    double raw_hp = h_mua(c);
                    double raw_hm = h_mua(c);
                    mua_plus(c) = std::min(mua(c) + raw_hp, mua_max - 1e-8);
                    mua_minus(c) = std::max(mua(c) - raw_hm, mua_min + 1e-8);
                    h_mua_plus(c) = mua_plus(c) - mua(c);
                    h_mua_minus(c) = mua(c) - mua_minus(c);

                    raw_hp = h_musp(c);
                    raw_hm = h_musp(c);
                    musp_plus(c) = std::min(musp(c) + raw_hp, musp_max - 1e-8);
                    musp_minus(c) = std::max(musp(c) - raw_hm, musp_min + 1e-8);
                    h_musp_plus(c) = musp_plus(c) - musp(c);
                    h_musp_minus(c) = musp(c) - musp_minus(c);
                }

                // 四次前向（同时扰动）
                SFDI::Reflect_freq pred_mua_plus, pred_mua_minus;
                SFDI::Reflect_freq pred_musp_plus, pred_musp_minus;
                ctx->model->mc_model_for_SFDI(mua_plus, musp, pred_mua_plus);
                ctx->model->mc_model_for_SFDI(mua_minus, musp, pred_mua_minus);
                ctx->model->mc_model_for_SFDI(mua, musp_plus, pred_musp_plus);
                ctx->model->mc_model_for_SFDI(mua, musp_minus, pred_musp_minus);

                // 构造导数：中心差分 (不对称时自动使用真实步长和)
                Eigen::Array<double, W, F> dpred_dmua;
                Eigen::Array<double, W, F> dpred_dmusp;

                for (int c = 0; c < W; ++c)
                {
                    double denom_mua = h_mua_plus(c) + h_mua_minus(c);
                    double denom_musp = h_musp_plus(c) + h_musp_minus(c);

                    // 行差分 (F 维)
                    dpred_dmua.row(c) =
                        (pred_mua_plus.row(c) - pred_mua_minus.row(c)).array() / std::max(denom_mua, 1e-12);

                    dpred_dmusp.row(c) =
                        (pred_musp_plus.row(c) - pred_musp_minus.row(c)).array() / std::max(denom_musp, 1e-12);
                }

                // 梯度装配
                for (int c = 0; c < W; ++c)
                {
                    double g_mua = (2.0 * residual.row(c) * dpred_dmua.row(c)).sum();
                    double g_musp = (2.0 * residual.row(c) * dpred_dmusp.row(c)).sum();
                    grad[2 * c] = g_mua;
                    grad[2 * c + 1] = g_musp;
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
#pragma omp parallel for
    for (size_t i = 0; i < grid_results.size(); ++i)
    {
        solve(grid_results[i].model, grid_results[i].mua, grid_results[i].musp);
    }
    for (const auto &item : grid_results)
    {
        fout.write(reinterpret_cast<const char *>(item.mua.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.musp.data()), sizeof(double) * SFDI::WAVELENGTH_NUM);
        fout.write(reinterpret_cast<const char *>(item.model.data()), sizeof(double) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM);
    }
    fout.close();
}
