// Auto-generated. Do not edit.
// Source: config/protocol/bms_j1939.json
// Proto ver: 5aa45fdb295b8520

#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

namespace proto_bms_gen {

static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct EnumPair { uint32_t raw; const char* text; };

struct SignalDef {
  const char* name;
  uint16_t startbit_lsb;
  uint16_t length;
  double factor;
  double offset;
  double initial; // NaN if missing
  double min;     // NaN if missing
  double max;     // NaN if missing
  const char* unit;
  const char* receiver;
  const char* comment;
  const EnumPair* enums;
  uint16_t enum_len;
};

struct MessageDef {
  const char* name;
  uint32_t id;
  uint8_t dlc;
  uint8_t byte_order; // 0=Intel,1=Motorola
  int32_t cycle_ms;   // -1 if missing
  const char* sender;
  const char* receiver;
  const char* tx;
  const char* comment;
  const SignalDef* signals;
  uint16_t signal_len;
};

extern const char* kProtoVersion;
extern const MessageDef kMessages[];
extern const size_t kMessageCount;

const MessageDef* findMessage(uint32_t id);

} // namespace proto_bms_gen
