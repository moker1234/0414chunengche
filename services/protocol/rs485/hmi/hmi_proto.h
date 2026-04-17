#pragma once
#include <cstdint>
#include <functional>
#include <vector>

#include "hmi_address_table.h"
#include "./protocol_base.h"

struct HmiWriteEvent {
    uint16_t addr{0};      //  屏幕地址（不做映射）
    bool     is_bool{false};
    uint16_t value_u16{0};
    bool     is_multi{false};
};

class HMIProto : public ProtocolBase {
public:
    explicit HMIProto(uint8_t slave_id);

    // HMI 从站不走 parse 上报（我们在 ProtocolParserThread 里走 handleRs485Slave）
    uint8_t slaveAddr() const override { return addr_; }
    bool parse(const std::vector<uint8_t>& rx, DeviceData& out) override { (void)rx; (void)out; return false; }
    std::vector<uint8_t> buildReadCmd() override { return {}; }

    // ✅ 不再支持 compat / 映射
    void setCompatMode(bool) {}          // 兼容旧接口：空实现
    bool compatMode() const { return false; }

    // ====== 控制器侧“写表”接口（给 SnapshotMapper 用）======
    // read区（屏幕只能读：02/04）
    void setBoolRead(uint16_t addr, bool v)      { table_.setBoolRead(addr, v); }
    void setIntRead (uint16_t addr, uint16_t v)  { table_.setIntRead (addr, v); }

    // rw区（屏幕可写：01/03/05/06/0F/10）——用于“给屏幕显示默认值/同步当前值”
    void setBoolRw  (uint16_t addr, bool v)      { table_.writeBoolRw(addr, v); }
    void setIntRw   (uint16_t addr, uint16_t v)  { table_.writeIntRw (addr, v); }

    void setOnWrite(std::function<void(const HmiWriteEvent&)> cb) { on_write_ = std::move(cb); }

    bool handleRequest(const std::vector<uint8_t>& req, std::vector<uint8_t>& resp);

    // ProtocolBase: 从站处理入口（Router 统一调用这个）
    bool handleSlaveRequest(const std::vector<uint8_t>& frame,
                            std::vector<uint8_t>& resp) override
    {
        return handleRequest(frame, resp);
    }

private:
    void appendCrc(std::vector<uint8_t>& out);
    static uint16_t be16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | uint16_t(p[1]); }

    bool handleReadBits(uint8_t func, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp);
    bool handleReadRegs(uint8_t func, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp);

    bool handleWriteSingleCoil(const std::vector<uint8_t>& req, uint16_t addr, uint16_t value, std::vector<uint8_t>& resp);
    bool handleWriteSingleReg (const std::vector<uint8_t>& req, uint16_t addr, uint16_t value, std::vector<uint8_t>& resp);

    bool handleWriteMultiCoils(const std::vector<uint8_t>& req, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp);
    bool handleWriteMultiRegs (const std::vector<uint8_t>& req, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp);

private:
    uint8_t addr_{3};
    HmiAddressTable table_;
    std::function<void(const HmiWriteEvent&)> on_write_;

};
