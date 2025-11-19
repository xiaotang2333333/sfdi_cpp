#pragma once
#include "model_SFDI.hpp"
#include <string>
#include <vector>

namespace SFDI
{
    // 保存优化结果
    struct GridOptResult
    {
        Optical_prop mua;
        Optical_prop musp;
        SFDI::Reflect_wave_freq reflect;
    };

    // 反演主控
    class GridInverseSolver
    {
    public:
        // step为网格步长，n为网格数
        GridInverseSolver(SFDI::model_SFDI &model, int rdc_n, int rac_n, double rdc_min = 0.0, double rdc_max = 1.0, double rac_min = 0.0, double rac_max = 1.0);
        // 多波长联合优化: 一次性优化所有波长的mua和musp，结果写入dst_mua, dst_musp
        void solve(const SFDI::Reflect_wave_freq &target, Optical_prop &dst_mua, Optical_prop &dst_musp) const;
        void solve_and_save(const std::string &output_bin);
        void build_grid();
        std::vector<GridOptResult> grid_results;

    private:
        SFDI::model_SFDI &model;
        int rdc_n, rac_n;
        double rdc_min, rdc_max, rac_min, rac_max;
    };
}
