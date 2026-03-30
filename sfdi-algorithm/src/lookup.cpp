#include "lookup.hpp"
#include "model_SFDI.hpp"
#include <fstream>
#include <cmath>
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
    }
}

SFDI::SFDI_Lookup::SFDI_Lookup(int step_count, std::string lookup_path)
    : step_count(step_count)
{
    // 加载数据到全局 results_table
    drdc = (rdc_max - rdc_min) / (step_count - 1);
    drac = (rac_max - rac_min) / (step_count - 1);
    std::cout << "drdc: " << drdc << ", drac: " << drac << std::endl;
    results_table.resize(step_count * step_count); // 预分配空间
    load_samples(lookup_path, results_table);
}

void SFDI::SFDI_Lookup::query(const Reflect &measured, double &mua_dst, double &musp_dst) const
{
    // 检查 NaN 和 Inf，这些值会导致后续计算产生未定义行为
    if (std::isnan(measured(0)) || std::isnan(measured(1)) ||
        std::isinf(measured(0)) || std::isinf(measured(1)))
    {
        mua_dst = 0;
        musp_dst = 0;
        return;
    }
    if (measured(0) < rdc_min || measured(1) < rac_min || measured(0) > rdc_max || measured(1) > rac_max || measured(0) >= measured(1))
    {
        mua_dst = 0;
        musp_dst = 0;
        return; // 超出表格范围，返回错误值
    }
    std::array<int, 4> indices;
    std::vector<double> weights(4, 0.0);
    indices.fill(-1);
    double rac_loc = (measured(1) - rac_min) / drac;
    double rdc_loc = (measured(0) - rdc_min) / drdc;
    // 限制索引范围在 [0, step_count-1] 内，防止越界
    int rdc_index_right = std::min(static_cast<int>(std::ceil(rdc_loc)), step_count - 1);
    int rdc_index_left = std::max(rdc_index_right - 1, 0);
    int rac_index_up = std::min(static_cast<int>(std::ceil(rac_loc)), step_count - 1);
    int rac_index_down = std::max(rac_index_up - 1, 0);
    // 计算四个邻近点的索引并加权
    if (rac_index_up < rdc_index_left) // 左上点不能出下三角形
    {
        indices[0] = compute_flat_index(rac_index_up, rdc_index_left);
        if (indices[0] != -1 && indices[0] < static_cast<int>(results_table.size()))
        {
            weights[0] = 1.0 / (std::pow(rac_index_up - rac_loc, 2) +
                                std::pow(rdc_index_left - rdc_loc, 2));
        }
    }
    if (rac_index_down < rdc_index_left) // 左下角
    {
        indices[1] = compute_flat_index(rac_index_down, rdc_index_left);
        if (indices[1] != -1 && indices[1] < static_cast<int>(results_table.size()))
        {
            weights[1] = 1.0 / (std::pow(rac_index_down - rac_loc, 2) +
                                std::pow(rdc_index_left - rdc_loc, 2));
        }
    }
    if (rac_index_up < rdc_index_right) // 右上角
    {
        indices[2] = compute_flat_index(rac_index_up, rdc_index_right);
        if (indices[2] != -1 && indices[2] < static_cast<int>(results_table.size()))
        {
            weights[2] = 1.0 / (std::pow(rac_index_up - rac_loc, 2) +
                                std::pow(rdc_index_right - rdc_loc, 2));
        }
    }
    if (rac_index_down < rdc_index_right) // 右下角
    {
        if (indices[3] != -1 && indices[3] < static_cast<int>(results_table.size()))
        {
            indices[3] = compute_flat_index(rac_index_down, rdc_index_right);
            weights[3] = 1.0 / (std::pow(rac_index_down - rac_loc, 2) +
                                std::pow(rdc_index_right - rdc_loc, 2));
        }
    }
    // 归一化权重
    double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    // 计算加权平均
    mua_dst = 0.0;
    musp_dst = 0.0;
    if (weight_sum > 0)
    {
        for (int i = 0; i < 4; ++i)
        {
            const SFDI::SFDI_Result &res = results_table[indices[i]];
            mua_dst += res.mua * weights[i] / weight_sum;
            musp_dst += res.musp * weights[i] / weight_sum;
        }
    }
}