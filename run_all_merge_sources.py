#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess


def main():
    root_dir = os.path.abspath(".")
    python_exe = sys.executable  # 使用当前Python解释器

    found = 0
    success = 0
    failed = 0

    print(f"当前根目录: {root_dir}")
    print("开始扫描一级子文件夹中的 merge_sources.py ...\n")

    for name in sorted(os.listdir(root_dir)):
        sub_path = os.path.join(root_dir, name)

        # 只处理一级子文件夹
        if not os.path.isdir(sub_path):
            continue

        target_script = os.path.join(sub_path, "merge_sources.py")
        if not os.path.isfile(target_script):
            continue

        found += 1
        print(f"[发现] {target_script}")
        print(f"[执行] {python_exe} {target_script}")

        try:
            result = subprocess.run(
                [python_exe, target_script],
                cwd=root_dir,
                check=True
            )
            success += 1
            print(f"[成功] {name}\n")
        except subprocess.CalledProcessError as e:
            failed += 1
            print(f"[失败] {name}，返回码: {e.returncode}\n")
        except Exception as e:
            failed += 1
            print(f"[异常] {name}，错误: {e}\n")

    print("========== 执行完成 ==========")
    print(f"发现脚本数量: {found}")
    print(f"成功数量: {success}")
    print(f"失败数量: {failed}")

    if found == 0:
        print("未在一级子文件夹中找到 merge_sources.py")


if __name__ == "__main__":
    main()