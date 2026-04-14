#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
JSON -> C++ 静态协议表生成器（适配你的单表聚合JSON）

输入JSON结构（要点）：
{
  "messages":[
    {
      "name":"V2B_CMD",
      "id": 0x1802F3EF,
      "dlc": 8,
      "byte_order":"Intel",
      "cycle_ms": 100,
      "sender":"VCU",
      "receiver":"BMU",
      "tx":"Cyclic",
      "comment":"...",
      "signals":[
        {
          "name":"V2B_CMD_LifeSignal",
          "startbit_lsb":0,
          "length":8,
          "factor":1,
          "offset":0,
          "initial":0,
          "unit":"",
          "min":0,
          "max":255,
          "receiver":"BMU",
          "comment":"...",
          "val_table":{"255":"Signal Invalid"}
        }
      ]
    }
  ]
}
"""

import argparse
import hashlib
import json
import os
import re
from typing import Any, Dict, List, Tuple, Optional


def sha256_short(path: str, n: int = 16) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(1024 * 1024)
            if not b:
                break
            h.update(b)
    return h.hexdigest()[:n]


# def c_escape(s: str) -> str:
#     return str(s).replace("\\", "\\\\").replace('"', '\\"')
def c_escape(s: str) -> str:
    # 只删除换行符，不做替换
    s = "" if s is None else str(s)
    s = s.replace("\n", "").replace("\r", "")
    # 再做C++字符串必要转义
    return s.replace("\\", "\\\\").replace('"', '\\"')


def ident(s: str) -> str:
    s = str(s)
    out = []
    for ch in s:
        if ch.isalnum() or ch == "_":
            out.append(ch)
        else:
            out.append("_")
    r = "".join(out)
    if r and r[0].isdigit():
        r = "_" + r
    return r


def as_int(x: Any, default: int = 0) -> int:
    return int(x) if isinstance(x, int) else default


def as_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


def gen(proto: Dict[str, Any], json_path: str, out_dir: str, namespace: str, base_name: str):
    os.makedirs(out_dir, exist_ok=True)

    msgs = proto.get("messages", [])
    if not isinstance(msgs, list):
        raise ValueError("messages must be list")

    ver = sha256_short(json_path, 16)
    hdr_path = os.path.join(out_dir, f"{base_name}.h")
    cpp_path = os.path.join(out_dir, f"{base_name}.cpp")

    enum_arrays: List[Tuple[str, List[Tuple[int, str]]]] = []
    sig_arrays: List[Tuple[str, List[Dict[str, Any]]]] = []
    msg_defs: List[Dict[str, Any]] = []

    for mi, m in enumerate(msgs):
        mname = m.get("name", f"msg_{mi}")
        mid = m.get("id")
        if not isinstance(mid, int):
            raise ValueError(f"Message '{mname}' id must be int")

        dlc = as_int(m.get("dlc"), 8)
        bo = m.get("byte_order", "Intel")
        cycle_ms = m.get("cycle_ms")
        cycle_ms_i = int(cycle_ms) if isinstance(cycle_ms, int) else -1

        sender = m.get("sender", "")
        receiver = m.get("receiver", "")
        tx = m.get("tx", "")
        comment = m.get("comment", "")

        sigs = m.get("signals", [])
        if sigs is None:
            sigs = []
        if not isinstance(sigs, list):
            raise ValueError(f"Message '{mname}': signals must be list")

        sig_arr_name = f"kSignals_{ident(mname)}"
        sig_entries: List[Dict[str, Any]] = []

        for si, s in enumerate(sigs):
            sname = s.get("name", f"sig_{si}")
            start = as_int(s.get("startbit_lsb"), 0)
            length = as_int(s.get("length"), 1)
            factor = as_float(s.get("factor"), 1.0)
            offset = as_float(s.get("offset"), 0.0)
            initial = s.get("initial", None)
            minv = s.get("min", None)
            maxv = s.get("max", None)
            unit = s.get("unit", "")
            srecv = s.get("receiver", "")
            scomment = s.get("comment", "")

            # optional numbers -> store as double, use NaN when missing
            # (C++里用 std::isnan 判断有没有)
            def to_double_or_nan(v: Any) -> str:
                try:
                    if v is None:
                        return "kNaN"
                    return str(float(v))
                except Exception:
                    return "kNaN"

            init_s = to_double_or_nan(initial)
            min_s = to_double_or_nan(minv)
            max_s = to_double_or_nan(maxv)

            # enum table
            vt = s.get("val_table")
            enum_ref_name = "nullptr"
            enum_len = 0
            if isinstance(vt, dict) and len(vt) > 0:
                enum_name = f"kEnum_{ident(mname)}_{ident(sname)}"
                pairs: List[Tuple[int, str]] = []
                for k, v in vt.items():
                    # key is string like "255" or "0xFF"
                    ks = str(k).strip()
                    raw = int(ks, 16) if ks.lower().startswith("0x") else int(ks)
                    pairs.append((raw, str(v)))
                pairs.sort(key=lambda x: x[0])
                enum_arrays.append((enum_name, pairs))
                enum_ref_name = enum_name
                enum_len = len(pairs)

            sig_entries.append({
                "name": sname,
                "start": start,
                "length": length,
                "factor": factor,
                "offset": offset,
                "initial": init_s,
                "min": min_s,
                "max": max_s,
                "unit": unit,
                "receiver": srecv,
                "comment": scomment,
                "enum_ref": enum_ref_name,
                "enum_len": enum_len,
            })

        sig_arrays.append((sig_arr_name, sig_entries))

        msg_defs.append({
            "name": mname,
            "id": mid,
            "dlc": dlc,
            "byte_order": bo,
            "cycle_ms": cycle_ms_i,
            "sender": sender,
            "receiver": receiver,
            "tx": tx,
            "comment": comment,
            "sig_arr": sig_arr_name,
            "sig_len": len(sig_entries),
        })

    # ---------- write header ----------
    with open(hdr_path, "w", encoding="utf-8") as f:
        f.write("// Auto-generated. Do not edit.\n")
        f.write(f"// Source: {json_path}\n")
        f.write(f"// Proto ver: {ver}\n\n")
        f.write("#pragma once\n\n")
        f.write("#include <cstdint>\n")
        f.write("#include <cstddef>\n")
        f.write("#include <cmath>\n\n")
        f.write(f"namespace {namespace} {{\n\n")
        f.write("static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();\n\n")
        f.write("struct EnumPair { uint32_t raw; const char* text; };\n\n")
        f.write("struct SignalDef {\n")
        f.write("  const char* name;\n")
        f.write("  uint16_t startbit_lsb;\n")
        f.write("  uint16_t length;\n")
        f.write("  double factor;\n")
        f.write("  double offset;\n")
        f.write("  double initial; // NaN if missing\n")
        f.write("  double min;     // NaN if missing\n")
        f.write("  double max;     // NaN if missing\n")
        f.write("  const char* unit;\n")
        f.write("  const char* receiver;\n")
        f.write("  const char* comment;\n")
        f.write("  const EnumPair* enums;\n")
        f.write("  uint16_t enum_len;\n")
        f.write("};\n\n")
        f.write("struct MessageDef {\n")
        f.write("  const char* name;\n")
        f.write("  uint32_t id;\n")
        f.write("  uint8_t dlc;\n")
        f.write("  uint8_t byte_order; // 0=Intel,1=Motorola\n")
        f.write("  int32_t cycle_ms;   // -1 if missing\n")
        f.write("  const char* sender;\n")
        f.write("  const char* receiver;\n")
        f.write("  const char* tx;\n")
        f.write("  const char* comment;\n")
        f.write("  const SignalDef* signals;\n")
        f.write("  uint16_t signal_len;\n")
        f.write("};\n\n")
        f.write("extern const char* kProtoVersion;\n")
        f.write("extern const MessageDef kMessages[];\n")
        f.write("extern const size_t kMessageCount;\n\n")
        f.write("const MessageDef* findMessage(uint32_t id);\n\n")
        f.write(f"}} // namespace {namespace}\n")

    # ---------- write cpp ----------
    with open(cpp_path, "w", encoding="utf-8") as f:
        f.write("// Auto-generated. Do not edit.\n")
        f.write(f'#include "{base_name}.h"\n')
        f.write("#include <limits>\n\n")
        f.write(f"namespace {namespace} {{\n\n")
        f.write(f'const char* kProtoVersion = "{ver}";\n\n')

        # enums
        for enum_name, pairs in enum_arrays:
            f.write(f"static const EnumPair {enum_name}[] = {{\n")
            for raw, txt in pairs:
                f.write(f'  {{ {raw}u, "{c_escape(txt)}" }},\n')
            f.write("};\n\n")

        # signals arrays
        for sig_arr_name, sig_entries in sig_arrays:
            f.write(f"static const SignalDef {sig_arr_name}[] = {{\n")
            for s in sig_entries:
                f.write("  {\n")
                f.write(f'    "{c_escape(s["name"])}",\n')
                f.write(f"    {s['start']}u,\n")
                f.write(f"    {s['length']}u,\n")
                f.write(f"    {s['factor']},\n")
                f.write(f"    {s['offset']},\n")
                f.write(f"    {s['initial']},\n")
                f.write(f"    {s['min']},\n")
                f.write(f"    {s['max']},\n")
                f.write(f'    "{c_escape(s["unit"])}",\n')
                f.write(f'    "{c_escape(s["receiver"])}",\n')
                f.write(f'    "{c_escape(s["comment"])}",\n')
                f.write(f"    {s['enum_ref']},\n")
                f.write(f"    {s['enum_len']}u,\n")
                f.write("  },\n")
            f.write("};\n\n")

        # messages
        f.write("const MessageDef kMessages[] = {\n")
        for m in msg_defs:
            bo_val = 0 if str(m["byte_order"]) == "Intel" else 1
            f.write("  {\n")
            f.write(f'    "{c_escape(m["name"])}",\n')
            f.write(f"    0x{m['id']:08X}u,\n")
            f.write(f"    {m['dlc']}u,\n")
            f.write(f"    {bo_val}u,\n")
            f.write(f"    {m['cycle_ms']},\n")
            f.write(f'    "{c_escape(m["sender"])}",\n')
            f.write(f'    "{c_escape(m["receiver"])}",\n')
            f.write(f'    "{c_escape(m["tx"])}",\n')
            f.write(f'    "{c_escape(m["comment"])}",\n')
            f.write(f"    {m['sig_arr']},\n")
            f.write(f"    {m['sig_len']}u,\n")
            f.write("  },\n")
        f.write("};\n\n")

        f.write("const size_t kMessageCount = sizeof(kMessages) / sizeof(kMessages[0]);\n\n")

        f.write("const MessageDef* findMessage(uint32_t id) {\n")
        f.write("  for (size_t i = 0; i < kMessageCount; ++i) {\n")
        f.write("    if (kMessages[i].id == id) return &kMessages[i];\n")
        f.write("  }\n")
        f.write("  return nullptr;\n")
        f.write("}\n\n")

        f.write(f"}} // namespace {namespace}\n")

    print(f"[OK] Wrote: {hdr_path}")
    print(f"[OK] Wrote: {cpp_path}")
    print(f"     messages={len(msg_defs)} enums_arrays={len(enum_arrays)} proto_ver={ver}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json_path", help="input protocol json")
    ap.add_argument("out_dir", help="output dir")
    ap.add_argument("--namespace", default="proto_generated", help="C++ namespace")
    ap.add_argument("--base-name", default="proto_table", help="output base filename without extension")
    args = ap.parse_args()

    with open(args.json_path, "r", encoding="utf-8") as f:
        proto = json.load(f)

    gen(proto, args.json_path, args.out_dir, args.namespace, args.base_name)


if __name__ == "__main__":
    main()
# python3 utils/tools/json_to_cpp.py config/protocol/bms_j1939.json   services/protocol/can/bms/generated   --namespace proto_bms_gen   --base-name proto_bms_table