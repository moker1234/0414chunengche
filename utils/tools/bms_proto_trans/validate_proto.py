#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
JSON 协议校验器（适合 CI）
- 打印错误时附带 message.name / signal.name
"""

import argparse
import json
from typing import Any, Dict, List, Tuple


def _fmt_id(mid: Any) -> str:
    return f"0x{mid:08X}" if isinstance(mid, int) else str(mid)


def _err(errors: List[str], msg_name: str, msg_id: Any, sig_name: str, detail: str):
    prefix = f"[msg={msg_name or '?'} id={_fmt_id(msg_id)}]"
    if sig_name:
        prefix += f" [sig={sig_name}]"
    errors.append(f"{prefix} {detail}")


def _warn(warnings: List[str], msg_name: str, msg_id: Any, sig_name: str, detail: str):
    prefix = f"[msg={msg_name or '?'} id={_fmt_id(msg_id)}]"
    if sig_name:
        prefix += f" [sig={sig_name}]"
    warnings.append(f"{prefix} {detail}")


def _is_int_like(s: str) -> bool:
    try:
        int(s, 10)
        return True
    except Exception:
        try:
            int(s, 16)
            return True
        except Exception:
            return False


def validate(proto: Dict[str, Any], allow_overlap: bool, allow_duplicate_id: bool) -> Tuple[bool, List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []

    msgs = proto.get("messages")
    if not isinstance(msgs, list) or len(msgs) == 0:
        errors.append("[msg=? id=?] messages[] missing or empty")
        return False, errors, warnings

    seen_ids: Dict[int, str] = {}

    for i, m in enumerate(msgs):
        # message context
        mname = m.get("name") if isinstance(m.get("name"), str) else ""
        mid = m.get("id")

        dlc = m.get("dlc")
        bo = m.get("byte_order")

        if not mname:
            _err(errors, mname, mid, "", f"message name missing/empty at messages[{i}]")

        if not isinstance(mid, int):
            _err(errors, mname, mid, "", f"message id must be int, got {type(mid)}")
        else:
            if not allow_duplicate_id:
                if mid in seen_ids:
                    _err(errors, mname, mid, "", f"duplicate msg_id, also used by {seen_ids[mid]}")
                else:
                    seen_ids[mid] = f"messages[{i}]({mname})"

        if not isinstance(dlc, int) or not (0 <= dlc <= 64):
            _err(errors, mname, mid, "", f"dlc invalid: {dlc}")

        if bo not in ("Intel", "Motorola"):
            _err(errors, mname, mid, "", f"byte_order invalid: {bo} (expected Intel/Motorola)")

        sigs = m.get("signals", [])
        if sigs is None:
            sigs = []
        if not isinstance(sigs, list):
            _err(errors, mname, mid, "", "signals must be a list")
            continue

        bit_limit = (int(dlc) * 8) if isinstance(dlc, int) else 64
        # used bits for overlap check
        used = [False] * max(bit_limit, 64)

        for j, s in enumerate(sigs):
            sname = s.get("name") if isinstance(s.get("name"), str) else ""

            start = s.get("startbit_lsb")
            length = s.get("length")

            if not sname:
                _err(errors, mname, mid, sname, f"signal name missing/empty at signals[{j}]")

            if not isinstance(start, int) or start < 0:
                _err(errors, mname, mid, sname, f"startbit_lsb invalid: {start}")
                continue

            if not isinstance(length, int) or length <= 0:
                _err(errors, mname, mid, sname, f"length invalid: {length}")
                continue

            if start + length > bit_limit:
                _err(errors, mname, mid, sname, f"bit range out of dlc: start={start}, len={length}, dlc_bits={bit_limit}")

            if not allow_overlap:
                overlapped = False
                for b in range(start, start + length):
                    if 0 <= b < len(used) and used[b]:
                        _err(errors, mname, mid, sname, f"overlaps bit {b}")
                        overlapped = True
                        break
                if not overlapped:
                    for b in range(start, start + length):
                        if 0 <= b < len(used):
                            used[b] = True

            # enum table check
            vt = s.get("val_table")
            if vt is not None:
                if not isinstance(vt, dict):
                    _err(errors, mname, mid, sname, "val_table must be object/dict")
                else:
                    for k, v in vt.items():
                        if not isinstance(k, str):
                            _warn(warnings, mname, mid, sname, f"val_table has non-string key: {k}")
                            continue
                        if not _is_int_like(k):
                            _warn(warnings, mname, mid, sname, f"val_table key not int-like: '{k}'")
                        if not isinstance(v, str):
                            _warn(warnings, mname, mid, sname, f"val_table value not string for key '{k}'")

    return len(errors) == 0, errors, warnings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json_path", help="protocol json path")
    ap.add_argument("--allow-overlap", action="store_true", help="allow signals bit overlap")
    ap.add_argument("--allow-duplicate-id", action="store_true", help="allow duplicate message id")
    args = ap.parse_args()

    with open(args.json_path, "r", encoding="utf-8") as f:
        proto = json.load(f)

    ok, errors, warnings = validate(proto, args.allow_overlap, args.allow_duplicate_id)

    for w in warnings:
        print("[WARN]", w)
    for e in errors:
        print("[ERR ]", e)

    if ok:
        print("[OK] validate passed")
        raise SystemExit(0)
    else:
        print(f"[FAIL] validate failed: errors={len(errors)} warnings={len(warnings)}")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
# python3 utils/tools/validate_proto.py config/protocol/bms_j1939.json