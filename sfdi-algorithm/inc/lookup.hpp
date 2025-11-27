#ifndef LOOKUP_HPP
#define LOOKUP_HPP
#include <vector>
#include "model_SFDI.hpp"

namespace SFDI
{
    class SFDI_Lookup
    {
    private:
        int step_count;
        double drdc, drac;

    public:
        SFDI_Lookup(
            int step_count,
            std::string lookup_path = "sfdi_results.bin");
        ~SFDI_Lookup() = default;

        // 查找接口：给定测量的 model，返回最近的 (mua, musp)
        void query(const Reflect_wave_freq &measured, Optical_prop &mua_dst, Optical_prop &musp_dst)const;
    };
}
#endif // LOOKUP_HPP