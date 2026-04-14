#include "model_path_exporter.h"

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace control {

nlohmann::json ModelPathExporter::buildMergedModel_(const agg::SystemSnapshot& snap,
                                                    const nlohmann::json& logic_view)
{
    nlohmann::json j = snap.toJson();

    if (!j.is_object()) {
        j = nlohmann::json::object();
    }

    if (!j.contains("system") || !j["system"].is_object()) {
        j["system"] = nlohmann::json::object();
    }

    // 与 NormalHmiWriter::flushFromModel 保持一致：
    // HMI 实际使用的逻辑视图挂到 system.logic_view.*
    j["system"]["logic_view"] = logic_view.is_object()
                                ? logic_view
                                : nlohmann::json::object();

    return j;
}

bool ModelPathExporter::exportLatest(const agg::SystemSnapshot& snap,
                                     const nlohmann::json& logic_view,
                                     uint64_t now_ms)
{
    if (output_path_.empty()) return false;

    if (last_export_ms_ != 0 && now_ms >= last_export_ms_) {
        if ((now_ms - last_export_ms_) < min_interval_ms_) {
            return true; // 节流，不视为失败
        }
    }

    try {
        nlohmann::json root = buildMergedModel_(snap, logic_view);

        root["__export_meta"] = {
            {"generated_at_ms", now_ms},
            {"purpose", "flatten_to_csv_paths"},
            {"note", "full merged model for HMI path alignment"}
        };

        const fs::path out_path(output_path_);
        const fs::path dir = out_path.parent_path();
        if (!dir.empty()) {
            fs::create_directories(dir);
        }

        const std::string tmp = output_path_ + ".tmp";

        {
            std::ofstream ofs(tmp, std::ios::out | std::ios::trunc);
            if (!ofs.is_open()) return false;
            ofs << root.dump(2) << "\n";
            ofs.flush();
        }

        fs::rename(tmp, output_path_);
        last_export_ms_ = now_ms;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace control