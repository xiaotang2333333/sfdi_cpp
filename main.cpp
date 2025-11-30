#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACK
#include "model_SFDI.hpp"
#include "lookup.hpp"
#include "grid_inverse.hpp"
#include "grid_forward.hpp"
#include <fstream>
#include <omp.h>
#include <chrono>
#include <iostream>
#include "MainWindow.h"
#include <QApplication>
static SFDI::SFDI_Reflect output_AC, calibrated_reflectance;
static SFDI::Optical_prop_map mua_map, musp_map;
static SFDI::Optical_prop_map rdc_map, rac_map; // 输出RDC和RAC

int main(int argc, char *argv[])
{
    
#pragma omp parallel
    {
#pragma omp single
        std::cout << "Threads = " << omp_get_num_threads() << std::endl;
    }

    //初始化模型
    //std::cout << "Loading model and building lookup table..." << std::endl;
    //SFDI::model_SFDI model_comp("reference_670", "ROfRhoAndTime");

    //model_comp.setN(SFDI::Optical_prop().setConstant(1.4));
    //model_comp.FreqTest(0, 0.3, 31);
    //model_comp.setFrequency((SFDI::Freq() << 0.0, 0.2).finished());
    //model_comp.setN(SFDI::Optical_prop().setConstant(1.37));
#ifdef GENERATE_LOOKUP
    for (int i = 1; i <= 3; i++)
    {
        double RAC_FREQ = 0.1 * i;
        model_comp.setFrequency((SFDI::Freq() << 0.0, RAC_FREQ).finished());
        std::cout << "Computing grid for RAC frequency " << RAC_FREQ << "..." << std::endl;
        SFDI::GridForwardSolver grid_solver(model_comp, 1000, 1000, 1e-5, 1.5, 1e-5, 15.0);
        std::string output_bin = "grid_forward_" + std::to_string(i) + ".bin";
        grid_solver.compute_and_save(output_bin);
    }
#endif
#ifdef INVERSE_TEST
    //1. 初始化查找表
    SFDI::SFDI_Lookup lookup(1001, "output.bin");
    //2. 计算校准后的反射率
    std::cout << "Computing calibrated reflectance..." << std::endl;
    model_comp.LoadAndComputeAC("sample_670", output_AC);
    model_comp.R_compute(output_AC, calibrated_reflectance);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting lookup for all pixels..." << std::endl;
    std::ofstream cal("cal.bin", std::ios::binary);
    if (!cal)
        std::cerr << "Failed to open cal file for writing!" << std::endl;
    cal.write(reinterpret_cast<const char *>(calibrated_reflectance.data()),
              sizeof(double) * calibrated_reflectance.size());
    //3. 对每个像素点查找 (mua, musp)
    #pragma omp parallel for collapse(2)
        for (int h = 0; h < SFDI::IMG_HEIGHT; h++)
        {
            for (int w = 0; w < SFDI::IMG_WIDTH; w++)
            {
                // 获取该像素的测量反射率
                SFDI::Reflect_wave_freq measured = SFDI::AC2Model(calibrated_reflectance, h, w);
                // 查找最近的 (mua, musp)
                SFDI::Optical_prop mua_pixel, musp_pixel;
                lookup.query(measured, mua_pixel, musp_pixel);
                // 将结果按波长通道写回到映射
                for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c)
                {
                    mua_map(h, w, c) = mua_pixel(c);
                    musp_map(h, w, c) = musp_pixel(c);
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
#endif
#ifdef FORWARD_TEST    //读取TIF文件为mua_map和musp_map
        std::cout << "Reading Absorption.tif and Reduced_scattering.tif..." << std::endl;
        SFDI::Tiff_img mua_img = SFDI::open_tiff("Absorption.tif");
        SFDI::Tiff_img musp_img = SFDI::open_tiff("Reduced_scattering.tif");
        if (mua_img.dimension(0) != SFDI::IMG_HEIGHT || mua_img.dimension(1) != SFDI::IMG_WIDTH ||
            musp_img.dimension(0) != SFDI::IMG_HEIGHT || musp_img.dimension(1) != SFDI::IMG_WIDTH) {
            std::cerr << "TIF image size mismatch!" << std::endl;
            return 1;
        }
        // 填充mua_map和musp_map
        for (int h = 0; h < SFDI::IMG_HEIGHT; ++h) {
            for (int w = 0; w < SFDI::IMG_WIDTH; ++w) {
                for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c) {
                    double mua_val = mua_img(h, w, c);
                    double musp_val = musp_img(h, w, c);
                    if (mua_val == 0.0) mua_val = 1e-8;
                    if (musp_val == 0.0) musp_val = 1e-8;
                    mua_map(h, w, c) = mua_val;
                    musp_map(h, w, c) = musp_val;
                }
            }
        }
        // 遍历每个像素，调用mc_model_for_SFDI，分离RDC和RAC

        std::cout << "Running mc_model_for_SFDI for all pixels..." << std::endl;

    #pragma omp parallel for collapse(2)
        for (int h = 0; h < SFDI::IMG_HEIGHT; ++h) {
            for (int w = 0; w < SFDI::IMG_WIDTH; ++w) {
                SFDI::Optical_prop mua_pixel, musp_pixel;
                for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c) {
                    mua_pixel(c) = mua_map(h, w, c);
                    musp_pixel(c) = musp_map(h, w, c);
                }
                SFDI::Reflect_wave_freq result;
                model_comp.mc_model_for_SFDI(mua_pixel, musp_pixel, result);
                for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c) {
                    rdc_map(h, w, c) = result(c, 0); // RDC
                    rac_map(h, w, c) = result(c, 1); // RAC
                }
            }
        }
        std::ofstream rdc_file("RDC.bin", std::ios::binary);
        if (!rdc_file) {
            std::cerr << "Failed to open RDC.bin for writing!" << std::endl;
            return 1;
        }
        rdc_file.write(reinterpret_cast<const char *>(rdc_map.data()), sizeof(double) * rdc_map.size());
        rdc_file.close();
        std::cout << "RDC saved to RDC.bin" << std::endl;
        // 保存RAC
        std::ofstream rac_file("RAC.bin", std::ios::binary);
        if (!rac_file) {
            std::cerr << "Failed to open RAC.bin for writing!" << std::endl;
            return 1;
        }
        rac_file.write(reinterpret_cast<const char *>(rac_map.data()), sizeof(double) * rac_map.size());
        rac_file.close();
        std::cout << "RAC saved to RAC.bin" << std::endl;
        std::cout << "All done!" << std::endl;
#endif
        // 启动 Qt 窗口以显示 UI
        QApplication app(argc, argv);
        MainWindow w;
        w.show();
        return app.exec();
}