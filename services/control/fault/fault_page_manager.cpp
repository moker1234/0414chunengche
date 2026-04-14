//
// Created by lxy on 2026/3/2.
//

#include "fault_page_manager.h"

#include "fault_addr_layout.h"
#include "../fault/fault_center.h"

#include "../../protocol/rs485/hmi/hmi_proto.h"

namespace control {

    void FaultPageManager::flushToHmi(HMIProto& hmi) const
    {
        if (!center_) {
            // 当前页清零
            hmi.setIntRead(fault::ADDR_CUR_TOTAL_PAGES, 0);
            hmi.setIntRead(fault::ADDR_CUR_PAGE_INDEX, 0);
            for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i) {
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_CUR_SEQ_BASE + i), 0);
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_CUR_CODE_BASE + i), 0);
            }

            // 历史页清零
            hmi.setIntRead(fault::ADDR_HIS_TOTAL_PAGES, 0);
            hmi.setIntRead(fault::ADDR_HIS_PAGE_INDEX, 0);
            for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i) {
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_SEQ_BASE + i), 0);
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_CODE_BASE + i), 0);
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_ON_TIME_BASE + i), 0);
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_OFF_TIME_BASE + i), 0);
                hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_STATE_BASE + i), 0);
            }
            return;
        }

        center_->flushToHmi(hmi);
    }

} // namespace control