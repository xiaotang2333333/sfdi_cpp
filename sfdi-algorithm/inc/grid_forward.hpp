#pragma once
#include "model_SFDI.hpp"
#include <string>
#include <vector>

namespace SFDI
{

    // 反演主控
    class GridForwardSolver
    {
    public:
        // step为网格步长，n为网格数
        GridForwardSolver(SFDI::model_SFDI &model, int mua_n, int musp_n, double mua_min = 1e-5, double mua_max =5.0, double musp_min = 1e-5, double musp_max = 25.0);
        void compute_and_save(const std::string &output_bin);
        void build_grid();
        std::vector<SFDI::SFDI_Result> grid_results;

    private:
        SFDI::model_SFDI &model;
        int mua_n, musp_n;
        double mua_min, mua_max, musp_min, musp_max;
    };
}