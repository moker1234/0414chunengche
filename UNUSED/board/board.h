//
// Created by forlinx on 2025/12/23.
//

#ifndef ENERGYSTORAGE_BOARD_H
#define ENERGYSTORAGE_BOARD_H
/*
 * 板级配置结构体, 包含CAN, 串口, RS485, GPIO, ADC设备的配置信息
 */
#include <string>
#include <vector>

// CAN设备信息结构体
struct CanDeviceInfo {
    int canDev_idx;
    std::string name;      // CAN设备名称，如 "can0", "can1"
    std::string interface; // 接口名称，如 "/dev/can0"
    int bitrate;          // 波特率
    bool enabled;         // 是否启用

    CanDeviceInfo(const std::string& n, const std::string& iface, int br, bool en = true)
        : name(n), interface(iface), bitrate(br), enabled(en) {}
};

// 串口设备基础信息结构体
struct SerialDeviceInfo {
    int serialDev_idx;
    std::string name;      // 串口名称，如 "uart1", "uart2"
    std::string device;    // 设备路径，如 "/dev/ttyS4", "/dev/ttyUSB0"
    int baudrate;         // 波特率
    int data_bits;        // 数据位
    int stop_bits;        // 停止位
    char parity;          // 奇偶校验 ('N'-无, 'E'-偶, 'O'-奇)
    bool enabled;         // 是否启用

    SerialDeviceInfo(const std::string& n, const std::string& dev, int br,
                     int db = 8, int sb = 1, char p = 'N', bool en = true)
        : name(n), device(dev), baudrate(br), data_bits(db), stop_bits(sb), parity(p), enabled(en) {}
};

// RS485设备信息结构体（继承自串口设备）
struct Rs485DeviceInfo {
    int serialDev_idx;
    std::string name;      // RS485设备名称
    std::string device;    // 设备路径，如 "/dev/ttyS4", "/dev/ttyUSB0"
    int baudrate;         // 波特率
    int data_bits;        // 数据位
    int stop_bits;        // 停止位
    char parity;          // 奇偶校验 ('N'-无, 'E'-偶, 'O'-奇)
    bool enabled;         // 是否启用
    int de_pin;           // RS485方向控制引脚（DE/RE引脚）
    bool half_duplex;     // 是否为半双工模式

    Rs485DeviceInfo(const std::string& n, const std::string& dev, int br,
                    int db = 8, int sb = 1, char p = 'N', bool en = true,
                    int de = -1, bool hd = true)
        : name(n), device(dev), baudrate(br), data_bits(db), stop_bits(sb),
          parity(p), enabled(en), de_pin(de), half_duplex(hd) {}
};

// GPIO设备信息结构体
struct GpioDeviceInfo {
    int gpioDev_idx;
    std::string name;      // GPIO名称，如 "GPIO_1", "INPUT_1"
    int pin;              // 引脚编号
    std::string direction; // 方向：input, output
    bool enabled;         // 是否启用

    GpioDeviceInfo(const std::string& n, int p, const std::string& dir, bool en = true)
        : name(n), pin(p), direction(dir), enabled(en) {}
};

// ADC设备信息结构体
struct AdcDeviceInfo {
    int adcDev_idx;
    std::string name;      // ADC通道名称
    int channel;          // 通道号
    std::string device;   // 设备路径
    bool enabled;         // 是否启用

    AdcDeviceInfo(const std::string& n, int ch, const std::string& dev, bool en = true)
        : name(n), channel(ch), device(dev), enabled(en) {}
};

// 板级外设信息结构体
struct BoardPeripherals {
    std::vector<CanDeviceInfo> can_devices;           // CAN设备列表
    std::vector<SerialDeviceInfo> serial_devices;     // 普通串口设备列表
    std::vector<Rs485DeviceInfo> rs485_devices;       // RS485设备列表
    std::vector<GpioDeviceInfo> gpio_devices;         // GPIO设备列表
    std::vector<AdcDeviceInfo> adc_devices;           // ADC设备列表

    // 添加CAN设备
    void addCanDevice(const std::string& name, const std::string& interface, int bitrate, bool enabled = true) {
        can_devices.emplace_back(name, interface, bitrate, enabled);
    }

    // 添加普通串口设备
    void addSerialDevice(const std::string& name, const std::string& device, int baudrate,
                         int data_bits = 8, int stop_bits = 1, char parity = 'N', bool enabled = true) {
        serial_devices.emplace_back(name, device, baudrate, data_bits, stop_bits, parity, enabled);
    }

    // 添加RS485设备
    void addRs485Device(const std::string& name, const std::string& device, int baudrate,
                        int data_bits = 8, int stop_bits = 1, char parity = 'N', bool enabled = true,
                        int de_pin = -1, bool half_duplex = true) {
        rs485_devices.emplace_back(name, device, baudrate, data_bits, stop_bits, parity, enabled, de_pin, half_duplex);
    }

    // 添加GPIO设备
    void addGpioDevice(const std::string& name, int pin, const std::string& direction, bool enabled = true) {
        gpio_devices.emplace_back(name, pin, direction, enabled);
    }

    // 添加ADC设备
    void addAdcDevice(const std::string& name, int channel, const std::string& device, bool enabled = true) {
        adc_devices.emplace_back(name, channel, device, enabled);
    }

    // 获取设备数量统计
    size_t getTotalDeviceCount() const {
        return can_devices.size() + serial_devices.size() + rs485_devices.size() + gpio_devices.size() + adc_devices.size();
    }

    // 获取串口设备数量
    size_t getSerialDeviceCount() const {
        return serial_devices.size();
    }

    // 获取RS485设备数量
    size_t getRs485DeviceCount() const {
        return rs485_devices.size();
    }
};

// 预定义的板级外设配置
namespace BoardConfig {
    // 获取默认的板级外设配置
    inline BoardPeripherals getDefaultBoardPeripherals() {
        BoardPeripherals peripherals;

        // 添加CAN设备
        peripherals.addCanDevice("can0", "can0", 500000);  // 500kbps

        // 添加普通串口设备
        peripherals.addSerialDevice("uart1", "/dev/ttyS4", 115200, 8, 1, 'N');
        // peripherals.addSerialDevice("uart2", "/dev/ttyS1", 9600, 8, 1, 'N');

        // 添加RS485设备 - 包括您在driver_manager.cpp中看到的ttyS4
        // peripherals.addRs485Device("rs485_1", "/dev/ttyS4", 115200, 8, 1, 'N', true, 105, true); // 使用GPIO105作为方向控制引脚
        // peripherals.addRs485Device("rs485_2", "/dev/ttyS2", 19200, 8, 1, 'E', true, 106, true); // 使用GPIO106作为方向控制引脚


        // 添加GPIO设备
        peripherals.addGpioDevice("DI_1", 100, "input");
        peripherals.addGpioDevice("DI_2", 101, "input");
        peripherals.addGpioDevice("DO_1", 110, "output");
        peripherals.addGpioDevice("DO_2", 111, "output");

        // 添加ADC设备
        // peripherals.addAdcDevice("ADC_1", 0, "/dev/adc0");
        // peripherals.addAdcDevice("ADC_2", 1, "/dev/adc1");

        return peripherals;
    }
}

#endif //ENERGYSTORAGE_BOARD_H