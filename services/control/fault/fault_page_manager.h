//
// Created by lxy on 2026/3/2.
//

#ifndef ENERGYSTORAGE_FAULT_PAGE_MANAGER_H
#define ENERGYSTORAGE_FAULT_PAGE_MANAGER_H

#pragma once

#include <cstdint>

class HMIProto;

namespace control {

    class FaultCenter;

    /**
     * FaultPageManager
     * - 第四批迁移后：仅负责将 FaultCenter 当前页/历史页视图写入 HMI
     * - 不再维护 active/history/page/db/catalog
     */
    class FaultPageManager {
    public:
        void bindCenter(const FaultCenter* center) { center_ = center; }

        void flushToHmi(HMIProto& hmi) const;

    private:
        const FaultCenter* center_{nullptr};
    };

} // namespace control

#endif // ENERGYSTORAGE_FAULT_PAGE_MANAGER_H