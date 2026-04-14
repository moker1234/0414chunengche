//
// Created by lxy on 2026/3/10.
//

#include "bms_name_map.h"

namespace proto::bms {

    std::string BmsNameMap::canonicalMsgName(const char* proto_name)
    {
        if (!proto_name || !proto_name[0]) {
            return "UNKNOWN";
        }

        const std::string s(proto_name);

        // 协议表里常见的命名不一致先在这里统一
        if (s == "B2V_Fult1" || s == "B2V_Fult1_32960") return "B2V_Fault1";
        if (s == "B2V_Fult2" || s == "B2V_Fult2_32961") return "B2V_Fault2";

        // 后续如果 Excel / JSON 里还出现别名，继续加在这里
        return s;
    }

} // namespace proto::bms