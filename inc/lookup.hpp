#ifndef LOOKUP_HPP
#define LOOKUP_HPP
#include <nanoflann.hpp>
#include <memory>
#include <vector>
#include "model_SFDI.hpp"

namespace SFDI
{
    using namespace nanoflann;
    struct SFDI_Result
    {
        Optical_prop mua;       // 吸收系数
        Optical_prop musp;      // 约化散射系数
        SFDI::Reflect_wave_freq model; // 计算结果 (WAVELENGTH_NUM × FREQ_NUM)
    };
    // KD-Tree 适配器：直接基于 SFDI_Result 的 model 字段
    struct ResultCloud
    {
        const std::vector<SFDI_Result> &results;

        ResultCloud(const std::vector<SFDI_Result> &r) : results(r) {}

        // 返回样本数量                                    
        inline size_t kdtree_get_point_count() const { return results.size(); }

        // 返回第 idx 个样本的第 dim 维特征
        // dim=0: model(0,0), dim=1: model(0,1), ...
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            // 查找键是 model 的所有元素展平：[model(0,0), model(0,1), ...]
            const auto &m = results[idx].model;
            int row = static_cast<int>(dim) / FREQ_NUM;
            int col = static_cast<int>(dim) % FREQ_NUM;
            return m(row, col);
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
    };

    // KD-Tree 类型定义
    using KDTree = KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, ResultCloud>,
        ResultCloud,
        WAVELENGTH_NUM * FREQ_NUM, // 维度：model 的总元素数
        size_t>;

    class SFDI_Lookup
    {
    private:
        int mua_dim_num;
        int musp_dim_num;
        std::unique_ptr<ResultCloud> cloud;
        std::unique_ptr<KDTree> index;

    public:
        SFDI_Lookup(
            int mua_dim_num = -1,
            int musp_dim_num = -1,
            std::string lookup_path = "sfdi_results.bin");
        ~SFDI_Lookup() = default;

        // 查找接口：给定测量的 model，返回最近的 (mua, musp)
        std::pair<Optical_prop, Optical_prop> query(const Reflect_wave_freq &measured) const;
    };
}
#endif // LOOKUP_HPP