#include "model_SFDI.hpp"
#include "lookup.hpp"
#include "grid_inverse.hpp"
#include "grid_forward.hpp"
#include <fstream>
#include <omp.h>
#include <chrono>
#include <iostream>
#include "MainWindow.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    // 初始化模型
    //     std::cout << "Loading model and building lookup table..." << std::endl;
    //     SFDI::mc_model model_comp;
    #ifdef GENERATE_LOOKUP
        SFDI::GridInverseSolver grid_solver(1001, 1001);

        grid_solver.solve_and_save("test.bin");
    #endif
    // double mua,musp;
    // SFDI::mc_model model_mc;
    // while (std::cin>>mua>>musp)
    // {

    // SFDI::Reflect ans1,ans2,ans3;
    // model_mc.mc_model_for_SFDI(mua,musp,ans1);
    // model_mc.mc_model_for_SFDI_Dmusp(mua,musp,ans2);
    // model_mc.mc_model_for_SFDI_Dmua(mua,musp,ans3);
    // std::cout<<ans1<<std::endl<<ans2<<std::endl<<ans3<<std::endl;
    // }
#ifdef INVERSE_TEST
    // 1. 初始化查找表
    SFDI::SFDI_Lookup lookup(101, "test.bin");
    // 2. 计算校准后的反射率
    std::cout << "Computing calibrated reflectance..." << std::endl;
    SFDI::mc_model model_comp;
    std::array<Eigen::ArrayXXd, 6> mea_pic, cal_pic;
    for (int i = 0; i < 6; ++i)
    {
        mea_pic[i] = SFDI::open_tiff("003/frame_" + std::to_string(i) + ".tiff");
        cal_pic[i] = SFDI::open_tiff("calibration_frames/frame_" + std::to_string(i) + ".tiff");
    }

    Eigen::ArrayXXd cal_mac, cal_mdc, meas_mac, meas_mdc;
    double eps = 1e-8;
    SFDI::Reflect calibrated;
    model_comp.setN(1.4);
    model_comp.mc_model_for_SFDI(0.0059, 0.9748, calibrated);
    std::cout << calibrated << std::endl;
    SFDI::Compute_Amplitude_Envelope(mea_pic[0], mea_pic[1], mea_pic[2], 1.0, meas_mdc);
    SFDI::Compute_Amplitude_Envelope(mea_pic[3], mea_pic[4], mea_pic[5], 1.0, meas_mac);
    SFDI::Compute_Amplitude_Envelope(cal_pic[0], cal_pic[1], cal_pic[2], 1.0, cal_mdc);
    SFDI::Compute_Amplitude_Envelope(cal_pic[3], cal_pic[4], cal_pic[5], 1.0, cal_mac);
    Eigen::ArrayXXd Rac = meas_mac * 0.5 / (cal_mac + eps), Rdc = meas_mdc * 0.5 / (cal_mdc + eps);
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting lookup for all pixels..." << std::endl;
    Eigen::ArrayXXd mua_map(Rac.rows(), Rac.cols()), musp_map(Rac.rows(), Rac.cols());
// 3. 对每个像素点查找 (mua, musp)
#pragma omp parallel for collapse(2)
    for (int h = 0; h < Rac.rows(); h++)
    {
        for (int w = 0; w < Rac.cols(); w++)
        {
            // 获取该像素的测量反射率
            SFDI::Reflect measured = {Rdc(h, w), Rac(h, w)};
            // 查找最近的 (mua, musp)
            lookup.query(measured, mua_map(h, w), musp_map(h, w));
            // 将结果按波长通道写回到映射
        }
    }

    // 4. 结束计时
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Lookup completed in " << duration.count() << " ms" << std::endl;

    // 5. 保存 mua 到tiff
    if (!SFDI::save_tiff("003/mua.tiff", mua_map))
    {
        std::cerr << "Failed to save mua to TIFF!" << std::endl;
        return 1;
    }

    // 6. 保存 musp 到tiff
    if (!SFDI::save_tiff("003/musp.tiff", musp_map))
    {
        std::cerr << "Failed to save musp to TIFF!" << std::endl;
        return 1;
    }
    std::cout << "musp saved to sfdi_musp.tif" << std::endl;

    std::cout << "All done!" << std::endl;
#endif
#ifdef FORWARD_TEST // 读取TIF文件为mua_map和musp_map
    std::cout << "Reading Absorption.tif and Reduced_scattering.tif..." << std::endl;
    SFDI::Tiff_img mua_img = SFDI::open_tiff("Absorption.tif");
    SFDI::Tiff_img musp_img = SFDI::open_tiff("Reduced_scattering.tif");
    if (mua_img.dimension(0) != SFDI::IMG_HEIGHT || mua_img.dimension(1) != SFDI::IMG_WIDTH ||
        musp_img.dimension(0) != SFDI::IMG_HEIGHT || musp_img.dimension(1) != SFDI::IMG_WIDTH)
    {
        std::cerr << "TIF image size mismatch!" << std::endl;
        return 1;
    }
    // 填充mua_map和musp_map
    for (int h = 0; h < SFDI::IMG_HEIGHT; ++h)
    {
        for (int w = 0; w < SFDI::IMG_WIDTH; ++w)
        {
            for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c)
            {
                double mua_val = mua_img(h, w, c);
                double musp_val = musp_img(h, w, c);
                if (mua_val == 0.0)
                    mua_val = 1e-8;
                if (musp_val == 0.0)
                    musp_val = 1e-8;
                mua_map(h, w, c) = mua_val;
                musp_map(h, w, c) = musp_val;
            }
        }
    }
    // 遍历每个像素，调用mc_model_for_SFDI，分离RDC和RAC

    std::cout << "Running mc_model_for_SFDI for all pixels..." << std::endl;

    // #pragma omp parallel for collapse(2)
    for (int h = 0; h < SFDI::IMG_HEIGHT; ++h)
    {
        for (int w = 0; w < SFDI::IMG_WIDTH; ++w)
        {
            SFDI::Optical_prop mua_pixel, musp_pixel;
            for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c)
            {
                mua_pixel(c) = mua_map(h, w, c);
                musp_pixel(c) = musp_map(h, w, c);
            }
            SFDI::Reflect_wave_freq result;
            model_comp.mc_model_for_SFDI(mua_pixel, musp_pixel, result);
            for (int c = 0; c < SFDI::WAVELENGTH_NUM; ++c)
            {
                rdc_map(h, w, c) = result(c, 0); // RDC
                rac_map(h, w, c) = result(c, 1); // RAC
            }
        }
    }
    std::ofstream rdc_file("RDC.bin", std::ios::binary);
    if (!rdc_file)
    {
        std::cerr << "Failed to open RDC.bin for writing!" << std::endl;
        return 1;
    }
    rdc_file.write(reinterpret_cast<const char *>(rdc_map.data()), sizeof(double) * rdc_map.size());
    rdc_file.close();
    std::cout << "RDC saved to RDC.bin" << std::endl;
    // 保存RAC
    std::ofstream rac_file("RAC.bin", std::ios::binary);
    if (!rac_file)
    {
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