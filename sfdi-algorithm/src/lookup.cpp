#include "lookup.hpp"
#include "model_SFDI.hpp"
#include <fstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>
constexpr double mua_start = 1e-5;
constexpr double mua_end = 2;
constexpr double musp_start = 1e-5;
constexpr double musp_end = 15;
constexpr double rac_min = 0.0, rac_max = 1.0;
constexpr double rdc_min = 0.0, rdc_max = 1.0;
/// @brief 查表法实现，已知生成表时按等间递增RAC再递增RDC存储，所以展平成一维时，
/// 仅有下三角有效，即 rdc > rac
/// 所以 rac_index = round(rac/drac),rdc_index = round(rdc/drdc)
/// 取出方格4个点按距离加权平均
/// 最终定位公式为: index = rac_index + (rdc_index*(rdc_index-1))/2
namespace
{
    // 生成表时先递增RAC再递增RDC,
    void load_samples(const std::string &path, std::vector<SFDI::SFDI_Result> &results_table)
    {
        // 直接顺序读取grid_inverse生成的二进制文件内容，展平到results_table
        std::cout << "Checking for lookup table file: " << path << std::endl;
        std::ifstream fin(path, std::ios::binary);
        if (!fin.is_open())
        {
            throw std::runtime_error("Failed to open lookup table file: " + path);
        }
        fin.seekg(0, std::ios::end);
        std::streamsize file_size = fin.tellg();
        if (file_size == -1)
        {
            throw std::runtime_error("Failed to determine file size for: " + path);
        }
        if (file_size == 0)
        {
            throw std::runtime_error("Lookup table file is empty: " + path);
        }
        fin.seekg(0, std::ios::beg);
        // 每条记录的大小: mua + musp + reflect
        const std::size_t record_size = sizeof(double) * (1 + 1 + SFDI::FREQ_NUM);
        const std::size_t num_records = static_cast<std::size_t>(file_size) / record_size;
        std::cout << "File size: " << file_size << " bytes\nRecord size: " << record_size << " bytes\nLoading " << num_records << " records from lookup table..." << std::endl;
        if (static_cast<std::size_t>(file_size) % record_size != 0)
        {
            throw std::runtime_error("Lookup table file size mismatch");
        }
        results_table.resize(num_records);
        for (std::size_t i = 0; i < num_records; ++i)
        {
            fin.read(reinterpret_cast<char *>(&results_table[i].mua), sizeof(double));
            fin.read(reinterpret_cast<char *>(&results_table[i].musp), sizeof(double));
            fin.read(reinterpret_cast<char *>(results_table[i].model.data()), sizeof(double) * SFDI::FREQ_NUM);
            if (!fin)
            {
                throw std::runtime_error("Failed to read lookup table record " + std::to_string(i));
            }
        }
        fin.close();
    }
    /**
     * @brief 根据 rac_index 和 rdc_index 计算在下三角矩阵展平为一维后的索引位置
     *
     * @param rac_index
     * @param rdc_index
     * @return int
     */
    int compute_flat_index(int rac_index, int rdc_index)
    {
        if (rac_index < 0 || rdc_index < 1 || rac_index >= rdc_index)
        {
            return -1; // 无效索引
        }
        else
        {
            return rac_index + (rdc_index * (rdc_index - 1)) / 2;
        }
        /**
         * 0 1 -> 0
         * 0 2 -> 1
         * 1 2 -> 2
         * 0 3 -> 3
         *
         */
    }
}

SFDI::SFDI_Lookup::SFDI_Lookup(int step_count, std::string lookup_path)
    : step_count(step_count)
{
    // 加载数据到全局 results_table
    drdc = (rdc_max - rdc_min) / (step_count - 1);
    drac = (rac_max - rac_min) / (step_count - 1);
    std::cout << "drdc: " << drdc << ", drac: " << drac << std::endl;
    load_samples(lookup_path, results_table);
    // 验证读取的记录数是否与预期的下三角记录数一致
    const std::size_t expected_records = static_cast<std::size_t>(step_count) * static_cast<std::size_t>(step_count - 1) / 2;
    if (results_table.size() != expected_records)
    {
        throw std::runtime_error("Lookup table record count mismatch: expected " + std::to_string(expected_records) + ", got " + std::to_string(results_table.size()));
    }
}

void SFDI::SFDI_Lookup::query(const Reflect &measured, double &mua_dst, double &musp_dst) const
{
    // 检查 NaN 和 Inf，这些值会导致后续计算产生未定义行为
    mua_dst = 0.0;
    musp_dst = 0.0;
    if (
        measured(0) < rdc_min || measured(1) < rac_min || measured(0) > rdc_max || measured(1) > rac_max || measured(0) <= measured(1))
    {
        return; // 超出表格范围，返回错误值
    }
    std::array<int, 4> indices = {-1, -1, -1, -1};
    std::array<double, 4> weights = {0.0, 0.0, 0.0, 0.0};

    double rac_loc = (measured(1) - rac_min) / drac; // m = 0.02 -> 0 1 2
    double rdc_loc = (measured(0) - rdc_min) / drdc;

    // 精确命中处理：若落在格点上，直接返回该点结果，避免除以零
    constexpr double eps = 1e-12;
    double rac_round = std::round(rac_loc);
    double rdc_round = std::round(rdc_loc);
    if (std::abs(rac_loc - rac_round) < eps && std::abs(rdc_loc - rdc_round) < eps)
    {
        int idx = compute_flat_index(static_cast<int>(rac_round), static_cast<int>(rdc_round));
        if (idx != -1 && idx < static_cast<int>(results_table.size()))
        {
            mua_dst = results_table[idx].mua;
            musp_dst = results_table[idx].musp;
            return;
        }
    }

    // 使用 floor/ceil 邻点选择，避免 ceil 导致的不对称
    int rac_down = std::clamp(static_cast<int>(std::floor(rac_loc)), 0, step_count - 1); // 向下取整
    int rac_up = std::clamp(rac_down + 1, 0, step_count - 1);
    int rdc_left = std::clamp(static_cast<int>(std::floor(rdc_loc)), 0, step_count - 1);
    int rdc_right = std::clamp(rdc_left + 1, 0, step_count - 1);

    auto add_candidate = [&](int rac_i, int rdc_i, int slot) -> bool
    {
        int idx = compute_flat_index(rac_i, rdc_i);
        if (idx == -1 || idx >= static_cast<int>(results_table.size()))
            return false;
        double dx = static_cast<double>(rac_i) - rac_loc;
        double dy = static_cast<double>(rdc_i) - rdc_loc;
        double dist2 = dx * dx + dy * dy;
        if (dist2 < eps)
        {
            mua_dst = results_table[idx].mua;
            musp_dst = results_table[idx].musp;
            return true; // exact hit
        }
        indices[slot] = idx;
        weights[slot] = 1.0 / dist2;
        return false;
    };

    // 左上 (rac_up, rdc_left)
    if (add_candidate(rac_up, rdc_left, 0))
        return;
    // 左下 (rac_down, rdc_left)
    if (add_candidate(rac_down, rdc_left, 1))
        return;
    // 右上 (rac_up, rdc_right)
    if (add_candidate(rac_up, rdc_right, 2))
        return;
    // 右下 (rac_down, rdc_right)
    if (add_candidate(rac_down, rdc_right, 3))
        return;

    double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (weight_sum > 0.0)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (indices[i] == -1)
                continue;
            mua_dst += results_table[indices[i]].mua * (weights[i] / weight_sum);
            musp_dst += results_table[indices[i]].musp * (weights[i] / weight_sum);
        }
    }
    return; // 权重和为零，无法插值，返回错误值
}