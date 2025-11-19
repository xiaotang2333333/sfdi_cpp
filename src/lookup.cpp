#include "lookup.hpp"
#include "model_SFDI.hpp"
#include <fstream>
#include <cstring> // for std::memcpy
constexpr double mua_start = 1e-5;
constexpr double mua_end = 2;
constexpr double musp_start = 1e-5;
constexpr double musp_end = 15;

namespace
{
    std::once_flag load_flag;
    static std::vector<SFDI::SFDI_Result> results_table; // 存储完整结果，作为全局数据

    void load_samples(int mua_dim_num, int musp_dim_num, const std::string &path)
    {
        if (mua_dim_num <= 0 || musp_dim_num <= 0)
        {
            throw std::runtime_error("mua_dim_num and musp_dim_num must be positive integers when first loading samples.");
        }
        std::cout << "Checking for lookup table file: " << path << std::endl;
        std::ifstream fin(path);
        if (!fin) // 没有文件就生成
        {
            std::cout << "Lookup table not found, generating new table..." << std::endl;
            SFDI::model_SFDI comp("reference_670", "ROfRhoAndTime");
            const int N_MUA = mua_dim_num;    // 
            const int N_MUSP = musp_dim_num; // 
            const int N_TOTAL = N_MUA * N_MUSP;

            std::cout << "Generating " << N_TOTAL << " samples (" << N_MUA << " x " << N_MUSP << ")..." << std::endl;
            std::vector<SFDI::SFDI_Result> results(N_TOTAL);
            Eigen::ArrayXd mua_values = Eigen::ArrayXd::LinSpaced(N_MUA, mua_start, mua_end);
            Eigen::ArrayXd musp_values = Eigen::ArrayXd::LinSpaced(N_MUSP, musp_start, musp_end);
            std::cout << "Starting parallel computation..." << std::endl;
#pragma omp parallel for
            for (int idx = 0; idx < N_TOTAL; ++idx)
            {
                int i = idx / N_MUSP; // mua 索引
                int j = idx % N_MUSP; // musp 索引
                results[idx].mua = mua_values[i];
                results[idx].musp = musp_values[j];
                SFDI::Optical_prop mua_prop;
                SFDI::Optical_prop musp_prop;
                mua_prop.setConstant(mua_values[i]);
                musp_prop.setConstant(musp_values[j]);

                comp.mc_model_for_SFDI(
                    mua_prop,
                    musp_prop,
                    results[idx].model);
            }
            std::cout << "Computation completed, saving to file..." << std::endl;
            std::ofstream ofs(path, std::ios::binary);
            for (const auto &r : results)
            {
                ofs.write(reinterpret_cast<const char *>(r.mua.data()), sizeof(double) * r.mua.size());
                ofs.write(reinterpret_cast<const char *>(r.musp.data()), sizeof(double) * r.musp.size());
                ofs.write(reinterpret_cast<const char *>(r.model.data()),
                          sizeof(double) * r.model.size());
            }
            ofs.close();
        }

        // 关闭之前的流并重新打开为二进制模式
        fin.close();
        fin.clear(); // 清除错误标志
        fin.open(path, std::ios::binary);

        // 读取数据并还原到 results_table
        if (!fin.is_open())
        {
            throw std::runtime_error("Failed to open lookup table file: " + path);
        }

        // 计算文件中的记录数
        fin.seekg(0, std::ios::end);
        std::streamsize file_size = fin.tellg();

        // 检查 tellg() 是否失败
        if (file_size == -1)
        {
            throw std::runtime_error("Failed to determine file size for: " + path);
        }

        if (file_size == 0)
        {
            throw std::runtime_error("Lookup table file is empty: " + path);
        }

        fin.seekg(0, std::ios::beg);

        // 每条记录的大小: mua + musp + model
        const std::size_t record_size = sizeof(double) * (SFDI::WAVELENGTH_NUM + SFDI::WAVELENGTH_NUM + SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM);
        const std::size_t num_records = static_cast<std::size_t>(file_size) / record_size;

        std::cout << "File size: " << file_size << " bytes" << std::endl;
        std::cout << "Record size: " << record_size << " bytes" << std::endl;
        std::cout << "Loading " << num_records << " records from lookup table..." << std::endl;

        if (static_cast<std::size_t>(file_size) % record_size != 0)
        {
            throw std::runtime_error("Lookup table file size mismatch");
        }

        results_table.resize(num_records);

        for (std::size_t i = 0; i < num_records; ++i)
        {
            // 读取 mua
            fin.read(reinterpret_cast<char *>(results_table[i].mua.data()),
                     sizeof(double) * SFDI::WAVELENGTH_NUM);

            // 读取 musp
            fin.read(reinterpret_cast<char *>(results_table[i].musp.data()),
                     sizeof(double) * SFDI::WAVELENGTH_NUM);

            // 读取 model
            fin.read(reinterpret_cast<char *>(results_table[i].model.data()),
                     sizeof(double) * SFDI::WAVELENGTH_NUM * SFDI::FREQ_NUM);

            if (!fin)
            {
                throw std::runtime_error("Failed to read lookup table record " + std::to_string(i));
            }
        }

        fin.close();
    }

    // 获取全局 results_table 的引用
    const std::vector<SFDI::SFDI_Result> &get_results_table()
    {
        return results_table;
    }
}

SFDI::SFDI_Lookup::SFDI_Lookup(int mua_dim_num, int musp_dim_num, std::string lookup_path)
    : mua_dim_num(mua_dim_num), musp_dim_num(musp_dim_num)
{
    // 加载数据到全局 results_table
    std::call_once(load_flag, load_samples, mua_dim_num, musp_dim_num, lookup_path);

    // 构建 KD-Tree
    cloud = std::make_unique<ResultCloud>(get_results_table());
    index = std::make_unique<KDTree>(
        WAVELENGTH_NUM * FREQ_NUM,                    // 维度
        *cloud,                                       // 数据源
        nanoflann::KDTreeSingleIndexAdaptorParams(10) // 叶节点最大大小
    );
    index->buildIndex();
}

std::pair<SFDI::Optical_prop, SFDI::Optical_prop>
SFDI::SFDI_Lookup::query(const Reflect_wave_freq &measured) const
{
    // KNN 查找（k=1，即最近邻）
    const size_t num_results = 1;
    size_t ret_index;
    double out_dist_sqr;

    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr);

    index->findNeighbors(resultSet,  measured.data());

    // 返回最近点的 (mua, musp)
    const auto &results = get_results_table();
    return {results[ret_index].mua, results[ret_index].musp};
}