#pragma once
#include <vector>
#include "model_SFDI.hpp"

namespace SFDI
{
    class SFDI_Lookup
    {
    private:
        int step_count;
        double drdc, drac;
        std::vector<SFDI::SFDI_Result> results_table; // 存储完整结果，作为成员数据
    public:
        SFDI_Lookup(
            int step_count,
            std::string lookup_path = "sfdi_results.bin");
        ~SFDI_Lookup() = default;

        // 查找接口：给定测量的 model，返回最近的 (mua, musp)
        void query(const Reflect &measured, double &mua_dst, double &musp_dst)const;
    };
}
