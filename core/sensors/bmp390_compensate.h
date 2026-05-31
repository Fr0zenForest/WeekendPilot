#pragma once
#include <cstdint>

namespace wp {

namespace bmp390 {
    constexpr uint8_t kI2cAddr    = 0x76;
    constexpr uint8_t kI2cAddrAlt = 0x77;
    constexpr uint8_t kChipId     = 0x60;   // BMP390
    constexpr uint8_t kChipIdBmp388 = 0x50; // BMP388（寄存器+补偿与 390 兼容，仅芯片ID不同）
    constexpr uint8_t kRegChipId  = 0x00;
    constexpr uint8_t kRegErr     = 0x02;
    constexpr uint8_t kRegData0   = 0x04;   // press(3) + temp(3) = 6 字节
    constexpr uint8_t kRegEvent   = 0x10;
    constexpr uint8_t kRegPwrCtrl = 0x1B;
    constexpr uint8_t kRegOsr     = 0x1C;
    constexpr uint8_t kRegOdr     = 0x1D;
    constexpr uint8_t kRegConfig  = 0x1F;
    constexpr uint8_t kRegNvmPar  = 0x31;   // 21 字节 trim
    constexpr uint8_t kRegCmd     = 0x7E;
    constexpr uint8_t kCmdSoftReset = 0xB6;
    constexpr int     kLenNvm     = 21;
    constexpr int     kLenData    = 6;
}

// ⚠️ NVM trim 解析 + 温压补偿全部移植自 ElrsRX baro_bmp390.cpp（datasheet §8.4/§8.6
//    float 版），未在本硬件实测；给定 trim+raw 的补偿数学可测，真实气压噪声/温漂待实物。
// datasheet §8.4 float trim（从 21 字节 NVM 解出）。
struct Bmp390Calib {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin;   // compensateTemperature 写，compensatePressure 读
};

// 21 字节 NVM → float trim（datasheet §8.4）。
Bmp390Calib bmp390ParseCalib(const uint8_t nvm[21]);
// datasheet §8.6 温度补偿，返回 °C，并更新 calib.t_lin（顺序依赖：先调它）。
float bmp390CompensateTemperature(uint32_t uncomp_temp, Bmp390Calib& calib);
// datasheet §8.6 压力补偿，返回 Pa（依赖上一步的 t_lin）。
float bmp390CompensatePressure(uint32_t uncomp_press, const Bmp390Calib& calib);
// 便捷：6 字节 DATA → (压力 Pa, 温度 °C)。raw_press/raw_temp 为 24-bit 小端。
struct Bmp390Reading { float pressure_pa; float temperature_c; bool valid; };
Bmp390Reading bmp390Decode(const uint8_t data[6], Bmp390Calib& calib);

}  // namespace wp
