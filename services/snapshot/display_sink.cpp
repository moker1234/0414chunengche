//
// Created by lxy on 2026/1/24.
//

#include "display_sink.h"

DisplaySink::DisplaySink(DisplayRs485Session& session)
    : session_(session) {}

void DisplaySink::onSnapshot(const agg::SystemSnapshot& snap) {
    session_.onSnapshot(snap);
}
