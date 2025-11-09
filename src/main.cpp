#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACK
#include "model_SFDI.hpp"
#include "lookup.hpp"
#include <fstream>
#include <omp.h>
#include <chrono>
#include <iostream>

static SFDI::SFDI_AC output_AC, calibrated_reflectance;
static SFDI::Optical_prop_map mua_map, musp_map;

int main(void)
{
    std::cout << "Starting SFDI parameter lookup using KD-Tree..." << std::endl;
#pragma omp parallel
    {
#pragma omp single
        std::cout << "Threads = " << omp_get_num_threads() << std::endl;
    }

    // 1. 初始化模型和查找表
    std::cout << "Loading model and building lookup table..." << std::endl;
    SFDI::SFDI_Lookup lookup(500, 500);
    SFDI::model_SFDI model_comp("reference_670", "ROfRhoAndTime");
        // 2. 计算校准后的反射率
    std::cout << "Computing calibrated reflectance..." << std::endl;
    model_comp.LoadAndComputeAC("sample_670", output_AC);
    model_comp.R_compute(output_AC, calibrated_reflectance);
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting lookup for all pixels..." << std::endl;

    // 3. 对每个像素点查找 (mua, musp)
#pragma omp parallel for collapse(2)
    for (int h = 0; h < SFDI::IMG_HEIGHT; h++)
    {
        for (int w = 0; w < SFDI::IMG_WIDTH; w++)
        {
            // 获取该像素的测量反射率
            SFDI::Reflect_wave_freq measured = SFDI::AC2Model(calibrated_reflectance, h, w);

            // 查找最近的 (mua, musp)
            auto [mua, musp] = lookup.query(measured);

            // 保存结果
            for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c)
            {
                mua_map(h, w, c) = mua(c);
                musp_map(h, w, c) = musp(c);
            }
        }
    }

    // 4. 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Lookup completed in " << duration.count() << " ms" << std::endl;

    // 5. 保存 mua 到二进制文件
    std::ofstream mua_file("sfdi_mua.bin", std::ios::binary);
    if (!mua_file)
    {
        std::cerr << "Failed to open mua file for writing!" << std::endl;
        return 1;
    }
    mua_file.write(reinterpret_cast<const char *>(mua_map.data()),
                   sizeof(double) * mua_map.size());
    mua_file.close();
    std::cout << "mua saved to sfdi_mua.bin" << std::endl;

    // 6. 保存 musp 到二进制文件
    std::ofstream musp_file("sfdi_musp.bin", std::ios::binary);
    if (!musp_file)
    {
        std::cerr << "Failed to open musp file for writing!" << std::endl;
        return 1;
    }
    musp_file.write(reinterpret_cast<const char *>(musp_map.data()),
                    sizeof(double) * musp_map.size());
    musp_file.close();
    std::cout << "musp saved to sfdi_musp.bin" << std::endl;

    std::cout << "All done!" << std::endl;
    return 0;
}